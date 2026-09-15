//! IEC processor, UCI command interface, ACIA, C2N tape (T0).
//! Spec: docs/specs/S04-board-t0.md. Registers: docs/hw/11-drives-iec-periph.md.

use std::sync::{Arc, Mutex};
use crate::devices::board::{add_table, at, span, Reg, Span, RAM};
use crate::io::{IoCtx, IoDevice, IoMap};

use crate::machine::MachineConfig;

/// IEC processor 0x10028000 (iec_processor_io.vhd). Registers decode `address(3:0)`, CODE RAM is bit 11.
const IEC: &[Span] = &[
    // VERSION, only printed (iec_interface.cc:73).
    at(0x00, Reg::Const(0x25)),
    // 00 §2 C22, 11 H11/H12: idle FIFOs. TX_FIFO_STATUS 0x01 = down FIFO empty, not full; RX_FIFO_STATUS
    // 0x01 = up FIFO empty, so the "IEC Server" poll every 2 ticks (iec_interface.cc:182-189) reads nothing.
    at(0x01, Reg::Const(0x01)),
    at(0x02, Reg::Const(0x01)),
    // 00 §1c M7, 11 H10: the slot[3] `dst[-1]` write to 0x100287FF (iec_interface.cc:121-126,141) is a no-op.
    // CODE RAM: 0x768-byte microcode plus the patched device address bytes (iec_interface.cc:71-81,128-145).
    span(0x800, 0x1000, RAM),
];


// UCI (UltiCommand interface). Bit-exact port of command_protocol.vhd (`gideon`
// architecture), from 1541ultimate/fpga/io/command_interface/vhdl_source/.
// FW-visible window: doc 11 lines 118-136 (io_req in the VHDL).
// C64-visible window: doc 11 §"UCI (command interface)" (slot_req in the VHDL).
// Buffer layout: command 0x000-0x37F, response 0x380-0x6FF, status 0x700-0x7FF
// (command_protocol.vhd:95,117-119,131,136,178-184).

const CMD_ADDR: u16 = 0x000;
const CMD_END: u16 = 0x37F;
const RESP_ADDR: u16 = 0x380;
const RESP_END: u16 = 0x6FF;
const STATUS_ADDR: u16 = 0x700;
const STATUS_END: u16 = 0x7FF;

/// C64-side sub-offsets within the slot's 8-byte window, derived from doc 11's $DF18-based
/// example (SLOT_BASE=0x47 → $DF18-$DF1F): $DF1B=bus-id, $DF1C=control, $DF1D=command,
/// $DF1E=response, $DF1F=status. 0/1/2 are unused (command_protocol.vhd:97-103).
mod slot {
    pub const BUS_ID: u32 = 3;
    pub const CONTROL: u32 = 4;
    pub const COMMAND: u32 = 5;
    pub const RESPONSE: u32 = 6;
    pub const STATUS: u32 = 7;
}

/// The 2-bit protocol state (command_protocol.vhd:48-52,87): 00 idle, 01 processing,
/// 11 data-more, 10 data-last. Kept as the raw encoding so `state(1)`/`state(0)` in the
/// VHDL translate directly into `.hi()`/`.lo()` here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct St(u8);
impl St {
    fn hi(self) -> bool {
        self.0 & 0b10 != 0
    }
    fn lo(self) -> bool {
        self.0 & 0b01 != 0
    }
    fn bits(self) -> u8 {
        self.0
    }
}

/// Everything both sides of the bridge touch. `Arc<Mutex<_>>` because the FW-visible window
/// (this module, in `ue2-core`) and the C64-visible window (a `CartMapper` composite in
/// `c64-bridge`, S14 §11 Phase 2) live on different buses in different crates — exactly
/// mirroring the VHDL's two register interfaces (`io_req` vs `slot_req`) onto one block RAM.
pub struct UciShared {
    enabled: bool,
    slot_base: u8, // bits 6:1 (command_protocol.vhd:66,202)
    bus_id: u8,    // bits 4:0, FW offset 0x01 bit7=1 (line 207)
    irq_mask: u8,  // bits 2:0, reset 0b111 (line 297)
    handshake_in: u8, // bits 2:0 = abort/data-accepted/new-command (line 89)
    state: St,
    error_busy: bool,
    freeze_i: bool,
    trigger: bool,
    cmd_irq_en: bool,
    command_pointer: u16,
    response_pointer: u16,
    status_pointer: u16,
    response_length: u16,
    status_length: u16,
    ram: [u8; 0x800],
}

impl Default for UciShared {
    fn default() -> Self {
        UciShared {
            enabled: false,
            slot_base: 0,
            bus_id: 0,
            irq_mask: 0b111,
            handshake_in: 0,
            state: St(0b00),
            error_busy: false,
            freeze_i: false,
            trigger: false,
            cmd_irq_en: false,
            command_pointer: CMD_ADDR,
            response_pointer: RESP_ADDR,
            status_pointer: STATUS_ADDR,
            response_length: 0,
            status_length: 0,
            ram: [0; 0x800],
        }
    }
}

pub type UciHandle = Arc<Mutex<UciShared>>;

impl UciShared {
    /// Whether this UCI instance claims C64 address `addr`, and its sub-offset within the
    /// 8-byte window if so (command_protocol.vhd:108,141: `bus_address(8:3) = slot_base` and
    /// `enabled`). `addr` is the absolute C64 address; IO1/IO2 span $DE00-$DFFF.
    pub fn c64_claims(&self, addr: u16) -> Option<u32> {
        if !self.enabled || !(0xDE00..=0xDFFF).contains(&addr) {
            return None;
        }
        let rel = addr - 0xDE00;
        ((rel >> 3) as u8 == self.slot_base).then(|| (rel & 0x07) as u32)
    }
    
    fn reset_response(&mut self) {
        self.response_pointer = RESP_ADDR;
        self.status_pointer = STATUS_ADDR;
    }

    /// command_protocol.vhd:131-135. No separate "next" register: like the rest of this
    /// project (see c64-bridge/cart.rs's docstring), a byte-level emulator can compute this
    /// live on access instead of replicating the 1-cycle latch — no C64 program can tell.
    fn response_valid(&self) -> bool {
        self.state.hi() && self.handshake_in & 0b100 == 0 && (self.response_pointer - RESP_ADDR) < self.response_length
    }
    fn status_valid(&self) -> bool {
        self.state.hi() && self.handshake_in & 0b100 == 0 && (self.status_pointer - STATUS_ADDR) < self.status_length
    }

    /// command_protocol.vhd:310.
    fn irq_line(&self) -> bool {
        self.handshake_in & !self.irq_mask & 0b111 != 0
    }

    fn status_byte(&self) -> u8 {
        (self.response_valid() as u8) << 7
            | (self.status_valid() as u8) << 6
            | self.state.bits() << 4
            | (self.error_busy as u8) << 3
            | (self.handshake_in & 0b111)
    }

    // ---- C64 side (slot_req, command_protocol.vhd:97-190) ----
    // `off` is 0..=7 within the slot's own 8-byte window (Phase 2 wires this from the
    // absolute C64 address once `slot_base`/`enabled` decode matches).

    pub fn c64_read(&mut self, off: u32) -> u8 {
        if !self.enabled {
            return 0xFF; // undecoded: the real slot just isn't there
        }
        match off {
            slot::CONTROL => self.status_byte(),
            // irq_n & "1001001" (line 99,111): 0xC9 idle, 0x49 while this UCI's IRQ is up.
            slot::COMMAND => {
                if self.irq_line() {
                    0x49
                } else {
                    0xC9
                }
            }
            slot::RESPONSE => {
                let v = if self.response_valid() { self.ram[self.response_pointer as usize] } else { 0x00 };
                self.cmd_irq_en = false; // line 177
                if self.response_pointer != RESP_END {
                    self.response_pointer += 1; // line 178-180
                }
                v
            }
            slot::STATUS => {
                let v = if self.status_valid() { self.ram[self.status_pointer as usize] } else { 0x00 };
                self.cmd_irq_en = false; // line 182
                if self.status_pointer != STATUS_END {
                    self.status_pointer += 1; // line 183-184
                }
                v
            }
            slot::BUS_ID => self.bus_id,
            _ => 0xFF, // line 103
        }
    }

    /// Returns the new `freeze` line (command_protocol.vhd:311) — wire this into whatever
    /// drives the NMI/menu-overlay freeze in Phase 2; a UCI command can trigger it same as
    /// the physical menu button.
    pub fn c64_write(&mut self, off: u32, val: u8) -> bool {
        if self.enabled {
            match off {
                slot::COMMAND => {
                    self.ram[self.command_pointer as usize] = val; // `do_write`, line 113-121
                    if self.command_pointer != CMD_END {
                        self.command_pointer += 1; // line 144-147
                    }
                }
                slot::CONTROL => {
                    if val & 0x08 != 0 {
                        self.error_busy = false; // line 149-151
                    }
                    if val & 0x01 != 0 {
                        self.freeze_i = val & 0x80 != 0; // line 153
                        self.trigger = val & 0x40 != 0; // line 154
                        if self.state.bits() == 0b00 {
                            self.state = St(0b01);
                            self.handshake_in |= 0b001; // new command (line 155-157)
                        } else {
                            self.error_busy = true; // line 159: C64 started a cmd while busy
                        }
                        self.cmd_irq_en = val & 0x20 != 0; // line 161
                    }
                    if val & 0x02 != 0 && self.state.hi() {
                        // data accept (line 163-167): only firmware clears handshake_in(1)
                        if self.state.lo() {
                            self.handshake_in |= 0b010;
                        }
                        self.state = St(self.state.bits() & 0b01); // clear state(1)
                        self.cmd_irq_en = false;
                    }
                    if val & 0x04 != 0 {
                        self.handshake_in |= 0b100; // abort (line 168-170)
                    }
                }
                _ => {} // command/control are the only C64-writable offsets (line 171-172)
            }
        }
        self.freeze_i
    }

    // ---- Firmware side (io_req, command_protocol.vhd:197-289) ----

    pub fn fw_read(&self, off: u32) -> u8 {
        match off {
            0x00 => self.slot_base << 1,
            0x01 => self.enabled as u8,
            0x02 => (self.freeze_i as u8) << 7 | (self.trigger as u8) << 6 | self.state.bits() << 4,
            0x03 => self.status_byte(), // line 261-262: identical byte the C64 sees at CONTROL
            0x04 => 0x00, // command buffer start (line 263-264)
            0x05 => 0x6F, // command buffer end (line 265-266)
            0x06 => 0x70, // response buffer start (line 267-268)
            0x07 => 0xDF, // response buffer end (line 269-270)
            0x08 => 0xE0, // status buffer start (line 271-272)
            0x09 => 0xFF, // status buffer end (line 273-274)
            0x0A => self.status_pointer as u8, // line 275-276 (upstream FIXME, 8 bits only)
            0x0B => self.irq_mask,
            0x0C => self.response_pointer as u8,
            0x0D => (self.response_pointer >> 8) as u8 & 0x07,
            0x0E => self.command_pointer.wrapping_sub(CMD_ADDR) as u8,
            0x0F => (self.command_pointer.wrapping_sub(CMD_ADDR) >> 8) as u8 & 0x07,
            _ => 0,
        }
    }

    pub fn fw_write(&mut self, off: u32, val: u8) {
        match off {
            0x00 => self.slot_base = (val & 0x7E) >> 1,
            0x01 => {
                if val & 0x80 == 0 {
                    self.enabled = val & 0x01 != 0;
		    eprintln!("uci: fw_write(0x01, {val:#04x}) -> enabled={}", self.enabled); //rix debug
                } else {
                    self.bus_id = val & 0x1F;
		    eprintln!("uci: fw_write(0x01, {val:#04x}) -> bus_id={:#04x}", self.bus_id); //rix debug
                }
            }
            0x02 => {
                if val & 0x01 != 0 {
                    self.handshake_in &= !0b001;
                    self.command_pointer = CMD_ADDR;
                }
                if val & 0x02 != 0 {
                    self.handshake_in &= !0b010;
                }
                if val & 0x04 != 0 {
                    self.handshake_in &= !0b100;
                }
                if val & 0x10 != 0 {
                    self.trigger = false;
                    self.freeze_i = false;
                    self.state = St(0b10 | ((val >> 5) & 1)); // state(1)<=1, state(0)<=bit5
                    self.reset_response();
                }
                if val & 0x80 != 0 {
                    self.freeze_i = false;
                    self.trigger = false;
                    self.reset_response();
                    self.state = St(0b00);
                }
            }
            0x04 => self.irq_mask |= val & 0x07,   // IRQMASK_SET, same offset as COMMAND_START read
            0x05 => self.irq_mask &= !(val & 0x07), // IRQMASK_CLEAR, same offset as COMMAND_END read
            0x0A => {
                self.status_pointer = STATUS_ADDR;
                self.status_length = val as u16;
            }
            0x0C => {
                self.response_pointer = RESP_ADDR;
                self.response_length = (self.response_length & 0x700) | val as u16;
            }
            0x0D => self.response_length = (self.response_length & 0xFF) | ((val as u16 & 0x07) << 8),
            0x0B => self.irq_mask = val & 0x07,
            _ => {}
        }
    }

    pub fn reset(&mut self) {
        *self = UciShared::default();
    }
}

pub struct UciDevice {
    shared: UciHandle,
}

impl UciDevice {
    pub fn new(shared: UciHandle) -> Self {
        UciDevice { shared }
    }
}

impl IoDevice for UciDevice {
    fn name(&self) -> &'static str {
        "uci"
    }

    fn read8(&mut self, off: u32, _ctx: &mut IoCtx) -> u8 {
        let s = self.shared.lock().unwrap();
        match off {
            0x800..=0xFFF => s.ram[(off - 0x800) as usize],
            _ => s.fw_read(off),
        }
    }

    fn write8(&mut self, off: u32, val: u8, ctx: &mut IoCtx) {
        let mut s = self.shared.lock().unwrap();
        match off {
            0x800..=0xFFF => s.ram[(off - 0x800) as usize] = val,
            _ => s.fw_write(off, val),
        }
        ctx.irq.set_level(4, s.irq_line()); // ITU_INTERRUPT_CMDIF, bit4 (doc 11 line 261)
    }

    fn peek8(&self, off: u32) -> u8 {
        let s = self.shared.lock().unwrap();
        match off {
            0x800..=0xFFF => s.ram[(off - 0x800) as usize],
            _ => s.fw_read(off),
        }
    }

    fn reset(&mut self) {
        self.shared.lock().unwrap().reset();
    }

    crate::impl_as_any!();
}

/// ACIA 6551 0x1004A000 (acia6551.vhd). No C64 side, so its registers stay at reset and irq_source is 0:
/// high IRQ 0 is never raised.
const ACIA: &[Span] = &[
    // rx_head / tx_tail: the app-owned ring indices.
    at(0x00, RAM),
    at(0x03, RAM),
    // 0x01 rx_tail, 0x02 tx_head, 0x04 control, 0x06 status read 0. command resets to 0x02.
    at(0x05, Reg::Const(0x02)),
    // enable + IRQ enables (acia.cc:18-20,72-84).
    at(0x07, Reg::Latch { mask: 0x1F, init: 0 }),
    // handsh: CTS 0, DSR 2, DCD 4, RTS disable 5, RX pushback 6 (reset 1); RTS/DTR come from the C64.
    at(0x08, Reg::Latch { mask: 0x75, init: 0x40 }),
    // TX ring 0x800 and RX ring 0xA00. The +0x100 mirrors of the 512-byte BRAM are not modelled; the
    // firmware never uses them.
    span(0x800, 0x900, RAM),
    span(0xA00, 0xB00, RAM),
];

/// C2N playback 0x100A0000: every read returns PLAYBACK_STATUS (c2n_playback_io.vhd). 00 §2 C26, 11 H15:
/// idle = FIFO empty (bit7), not enabled.
const TAPE_PLAY: &[Span] = &[span(0x000, 0x1000, Reg::Const(0x80))];

pub fn install(map: &mut IoMap, _cfg: &MachineConfig) -> UciHandle {
    add_table(map, 0x1002_8000, 0x1000, "iec", IEC);
    let uci = UciHandle::default();
    map.add(0x1004_4000, 0x1000, Box::new(UciDevice::new(uci.clone())));
    add_table(map, 0x1004_A000, 0x1000, "acia", ACIA);
    add_table(map, 0x100A_0000, 0x1000, "tape-play", TAPE_PLAY);
    // C2N record 0x100C0000: RECORD_STATUS 0 (bit7 = FIFO non-empty) and FIFO reads 0, so `flush()`
    // (tape_recorder.cc:250-262) exits and ITU bit 3 stays low (00 §2 C27, 11 H8/H16).
    add_table(map, 0x100C_0000, 0x1000, "tape-record", &[]);
    uci
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::devices::board::rig::Rig;

    #[test]
    fn c22_iec_registers() {
        let mut rig = Rig::new(install);
        assert_eq!([0, 1, 2].map(|o| rig.r8(0x1002_8000 + o)), [0x25, 0x01, 0x01]);
        // IecInterface ctor: reset, code load (iec_interface.cc:71-81).
        rig.w8(0x1002_8003, 0x00);
        for i in 0..0x768 {
            rig.w8(0x1002_8800 + i, i as u8);
        }
        // configure(): slot 0 listener/talker, then the stray slot 3 write.
        rig.w8(0x1002_8844, 0x3F);
        rig.w8(0x1002_8828, 0x5F);
        rig.w8(0x1002_87FF, 0x3F);
        rig.w8(0x1002_87FF, 0x5F);
        assert_eq!(rig.r8(0x1002_87FF), 0);
        assert_eq!((rig.r8(0x1002_8844), rig.r8(0x1002_8828), rig.r8(0x1002_8F67)), (0x3F, 0x5F, 0x67));
        assert_eq!([1, 2].map(|o| rig.r8(0x1002_8000 + o)), [0x01, 0x01], "still idle");
    }

    #[test]
    fn a5_uci_buffer_bases() {
        let mut rig = Rig::new(install);
        // CommandInterface ctor (command_intf.cc:44-53).
        rig.w8(0x1004_4000, 0x47);
        rig.w8(0x1004_4002, 0x87);
        assert_eq!([4, 5, 6, 7, 8, 9].map(|o| rig.r8(0x1004_4000 + o)), [0x00, 0x6F, 0x70, 0xDF, 0xE0, 0xFF]);
        assert_eq!((rig.r8(0x1004_4002), rig.r8(0x1004_4003)), (0, 0), "11 H7/H14");
        assert_eq!(rig.r8(0x1004_4000), 0x46);
        // C3: IRQ mask clear at UCI task start, ISR mask set (command_intf.cc:85-96,117-118).
        assert_eq!(rig.r8(0x1004_400B), 0x07);
        rig.w8(0x1004_4005, 0x07);
        assert_eq!(rig.r8(0x1004_400B), 0x00);
        rig.w8(0x1004_4004, 0x05);
        assert_eq!(rig.r8(0x1004_400B), 0x05);
        assert_eq!(rig.r8(0x1004_4004), 0x00);
        // Bus ID write does not enable the slot.
        rig.w8(0x1004_4001, 0x8B);
        assert_eq!(rig.r8(0x1004_4001), 0);
        rig.w8(0x1004_4001, 0x01);
        assert_eq!(rig.r8(0x1004_4001), 1);
        rig.w32(0x1004_4B80, 0x1234_5678);
        assert_eq!(rig.r32(0x1004_4B80), 0x1234_5678);
    }

    /// Drives a full command/response round trip purely through `UciShared`'s two faces
    /// (`c64_read`/`c64_write` and `fw_read`/`fw_write`), simulating both a C64 program and
    /// the firmware without needing a real firmware boot or the Phase 2 `CartMapper` bridge.
    #[test]
    fn uci_c64_round_trip() {
        let mut s = UciShared::default();
        s.fw_write(0x01, 0x01); // enable
        assert_eq!(s.fw_read(0x01), 1);

	// C64: write a 1-byte command, then trigger (control bit0).
        s.c64_write(slot::COMMAND, 0x01);
        s.c64_write(slot::CONTROL, 0x01);
        assert_eq!(s.fw_read(0x03) & 0x01, 0x01, "new-command flag visible to firmware");
        assert_eq!(s.fw_read(0x0E), 1, "command length");
        assert_eq!(s.ram[0], 0x01, "command byte landed in the shared RAM");

	// Firmware: ack the command byte, drop a 2-byte response into RAM, validate as "last".
        s.ram[RESP_ADDR as usize] = b'O';
        s.ram[RESP_ADDR as usize + 1] = b'K';
        s.fw_write(0x0C, 2); // response length low byte
        s.fw_write(0x0D, 0); // response length high bits
        s.fw_write(0x02, 0x01); // ack new-command, reset command pointer
        s.fw_write(0x02, 0x10); // validate, "last" (bit5=0)
        assert_eq!((s.fw_read(0x02) >> 4) & 0x03, 0b10, "state now data-last");

        // C64: response is flagged available, and reads back exactly what firmware wrote.
        assert_eq!(s.c64_read(slot::CONTROL) & 0x80, 0x80, "response-available flag");
        assert_eq!(s.c64_read(slot::RESPONSE), b'O');
        assert_eq!(s.c64_read(slot::RESPONSE), b'K');
        assert_eq!(s.c64_read(slot::RESPONSE), 0x00, "past response_length reads 0");

	// C64: accept the (last) data. Firmware sees the state return to idle.
        s.c64_write(slot::CONTROL, 0x02);
        assert_eq!((s.fw_read(0x02) >> 4) & 0x03, 0b00, "back to idle");
    }

    #[test]
    fn a6_acia_idle() {
        let mut rig = Rig::new(install);
        // Acia ctor (acia.cc:18-20).
        rig.w8(0x1004_A00A, 0x00);
        rig.w8(0x1004_A007, 0x00);
        assert_eq!(rig.r8(0x1004_A009), 0, "irq_source");
        assert_eq!(rig.r8(0x1004_A008), 0x40);
        rig.w8(0x1004_A007, 0x07);
        rig.w8(0x1004_A003, 0x10);
        rig.w8(0x1004_AA00, 0x41);
        assert_eq!((rig.r8(0x1004_A007), rig.r8(0x1004_A003), rig.r8(0x1004_AA00)), (0x07, 0x10, 0x41));
        assert_eq!((rig.r8(0x1004_A002), rig.r8(0x1004_A00A)), (0, 0));
    }

    #[test]
    fn c26_tape_idle() {
        let mut rig = Rig::new(install);
        // TapeController::stop (tape_controller.cc:111-115), TapeRecorder::stop (tape_recorder.cc:138-158).
        rig.w8(0x100A_0000, 0x06);
        rig.w8(0x100A_0000, 0x00);
        rig.w8(0x100C_0000, 0x00);
        rig.w8(0x100C_0000, 0x06);
        assert_eq!(rig.r8(0x100A_0000), 0x80);
        assert_eq!(rig.r8(0x100A_0800), 0x80);
        assert_eq!(rig.r8(0x100C_0000), 0x00);
        assert_eq!(rig.r32(0x100C_0800), 0);
    }
}
