# S37 — Keys as joystick (WASD in the window)

**Status:** draft patch (2026-09-26). The core wiring (§3, no fire) was confirmed compiling and passing the real
test suite on the author's machine (`cargo test -p ue2-core --lib devices::c64::` and
`-p ue2emu --bin ue2emu control::`, both 100% pass). The fire-key extension (§2, §3, §4's `[fire]` argument) was
added afterward and is only checked by hand-tracing and a standalone `rustc` reproduction in the authoring
sandbox, same as the rest of the original draft — not yet run through the real suite. §2's open questions are
answered with explicit, flagged, **runtime-overridable** defaults rather than confirmed against the firmware. The
window/menu enable toggle (originally listed under §4) is still not built. Filed against the gap S36 left open:
"Not covered: joystick keys in the window."

**Owns:**
- `crates/ue2-core/src/devices/c64.rs` (`MATRIX_WASD_TO_JOY` decode, `C64Port`) — all the actual logic
- `crates/ue2-core/src/host.rs` (`HostInput::WasdToJoy`)
- `crates/ue2-core/src/machine.rs` (dispatches `WasdToJoy` to `C64Port`)
- `crates/ue2emu/src/control.rs` (`wasd-joy` command)
- `crates/ue2emu/src/window.rs`, `crates/ue2emu/src/keymap.rs` — **not touched.** No live-window UI sets
  `MATRIX_WASD_TO_JOY`; it's only reachable via the `wasd-joy` control command or a direct register poke. Pressing
  WASD in the window still only types the letters w/a/s/d unless something else configures the register first.

**Reads:** `docs/specs/S36-joystick.md`, `keyboard_c64.cc` (cited by S36 as `keyboard_c64.cc:204-227` and the SPEC-07
branch at `:285-305`), `u64_config.cc:1060-1066`. None of these firmware sources are vendored in this repo; every
claim in §2 that depends on them is still marked as an assumption to check, not a fact — building the patch
resolved *how to make the assumptions overridable*, not *which value is correct*.

## 1. The gap

`C64Port` stores 4 bytes at `MATRIX_WASD_TO_JOY` (`0x1010_030B`-`0x30E`, right after `MATRIX_KEYB` at
`0x1010_0300`). Before this patch it was a plain latch: `matrix_write` only special-cased offsets 0-7
(`set_matrix_keyb`), 9 (`MATRIX_RESTORE`) and 10 (`MATRIX_FREEZE`); offsets 11-14 fell through to storage with no
side effect. Nothing consumed the four bytes, and nothing translated a WASD keypress into joystick lines.

## 2. Design decisions (defaults, not confirmed answers)

§2 originally listed four unconfirmed items against the real firmware. Building the patch turned three of them
into **explicit, named, runtime-settable defaults** instead of leaving them as blockers — the fourth (enable) fell
out for free:

- **Encoding:** `MATRIX_WASD_TO_JOY`'s four bytes are keyCodes, `row * 8 + col` (same convention as `MatrixKey`
  everywhere else), slot order up/down/left/right, `0xFF` (`WASD_NONE`) meaning "no key assigned." **Still
  unconfirmed**, but no longer blocking: this is what `C64Port::set_wasd_to_joy` and the `wasd-joy` command both
  assume, and it's easy to revise once you check.
- **Fire has no register at all.** The four-byte register has no documented fifth slot, and nothing vendored here
  confirms the real firmware maps a fire key anywhere. `wasd_fire`/`set_wasd_fire` is therefore not modeled on any
  known hardware behavior — it's a plain ue2emu-only extension, `C64Port` state with no backing register byte,
  added because keys-as-joystick without a fire key tests badly against anything that needs one. If the real
  firmware turns out to have an actual fire mechanism (a fifth byte elsewhere, a fixed key, whatever), this should
  be replaced rather than kept alongside it.
- **Target port:** was going to be a firmware-behavior question (own selection vs. reusing `U64II_KEYB_JOY`'s bit,
  vs. always port 2). Resolved pragmatically instead of answered: `wasd_to_joy_port` is a runtime field,
  `WASD_JOY_PORT_DEFAULT = 1` (port 2, matching the pre-S36 single-port `HostInput::Joystick`) until
  `C64Port::set_wasd_to_joy_port` (or `wasd-joy <port> ...`) says otherwise. This is what let testing proceed
  against a port-1 game without knowing the real answer.
- **Suppression:** does a WASD_TO_JOY match also still deliver the keyboard-matrix keypress? Decided as "yes, both"
  — `C64Port::set_key` calls `apply_wasd_to_joy` and then still forwards to the backend unconditionally. Flagged
  at the call site as the harder-to-undo direction if this turns out wrong (suppressing later is easy; if the real
  firmware suppresses and this patch doesn't, some game logic might see an unexpected keypress).
- **Enable:** resolved implicitly. `wasd_to_joy` defaults to all-`WASD_NONE`, so keys-as-joystick is off until
  something writes real codes to it; no separate enable bit needed in the model.

## 3. What's actually wired

Entirely inside `C64Port` — no cross-device `Arc` sharing was needed, unlike the original sketch (which assumed
the logic had to straddle `U64Io::set_key`, since that's where the S36-era author believed real keypresses first
land). `C64Port::set_key` turned out to already be the function that forwards every real keypress to the TRX64
backend, and it already owns both `MATRIX_WASD_TO_JOY`'s storage and `apply_joysticks`:

- `matrix_write` offsets 11-14 → `wasd_to_joy: [u8; 4]`.
- `C64Port::set_key(row, col, down)` calls `apply_wasd_to_joy` first (checks `row*8+col` against `wasd_to_joy`,
  folds a match into `keys_joy`, calls `apply_joysticks`), then forwards to the backend as before (§2 suppression
  answer).
- `apply_joysticks` ANDs `keys_joy` into `wasd_to_joy_port`'s line alongside the existing physical-stick/SWOUT
  wiring (S36), so a physical stick and keys-as-joystick on the same port compose correctly.
- `wasd_fire` matches the same way as the four direction slots but folds into bit 4 instead of a `wasd_to_joy`
  index; checked first in `apply_wasd_to_joy`, so if it were ever set to the same code as a direction (it
  shouldn't be), fire would win. Clearing it (`set_wasd_fire(WASD_NONE)`) is a plain field write with no immediate
  backend call — like `matrix_write`'s direction-slot comment already notes, a bit already folded into `keys_joy`
  stays folded until its own key is released; only the *next* press stops matching.
- `set_wasd_to_joy_port` re-runs `apply_joysticks` on switch, so a key already held follows the port immediately
  rather than waiting for its next press — see `s37_wasd_to_joy_port_is_settable`'s test comment.

## 4. Control command (S08)

Built: `wasd-joy <port> <up> <down> <left> <right> [fire]` (`control.rs`). Each direction (and fire) is a key name
(`key_by_name`, same table `key`/`hold`/`type` use) or `none` for `WASD_NONE` — not a raw numeric keyCode as
originally sketched, since key names are what a person writing a `.ctl` script actually has. `port` is 1 or 2 and
calls `set_wasd_to_joy_port`, so a script can point keys-as-joystick at whichever port the software under test
reads without a rebuild. `fire` is optional (omitted or `none` both mean `WASD_NONE`) and calls `set_wasd_fire`,
the extension described in §2. Existing `joy`/`joy-hold`/`joy-release` (S36) are unchanged.

## 5. Checks

- `s37_wasd_to_joy_folds_into_the_keys_port` (`c64.rs`): a configured keyCode folds into the default port and
  still delivers the keypress; an unassigned keyCode does neither; a physical stick on the other port is
  unaffected.
- `s37_wasd_to_joy_port_is_settable` (`c64.rs`): switching `wasd_to_joy_port` recombines immediately, including a
  key already held.
- `s37_wasd_fire_folds_into_bit_4` (`c64.rs`): fire folds and combines with directions normally; clearing the
  config doesn't touch an already-held bit, only future matches.
- `wasd_joy_parses_keys_and_none` (`control.rs`): command parsing, including the `none` and error-case paths.
- Still missing: a `scripts/smoke-joy-keys.ctl` end-to-end script (configure via `wasd-joy`, send a `key`, assert
  `PEEK(56320)`/`PEEK(56321)` the way `smoke-joy.ctl` does for `joy-hold`); a keys-as-joystick indicator in the
  window UI (S07/S08's overlay, out of scope here); the live-window enable path itself (§ "Owns" above).

## 6. Open before this leaves draft

- Confirm §2 against `keyboard_c64.cc`/`u64_config.cc` and update the defaults (or leave them, if they turn out
  right — S37 §2's flagged comments in `c64.rs` are exactly the lines to revisit).
- Run the real test suite (`cargo test -p ue2-core --lib devices::c64::` and `-p ue2emu --lib control::`) on a
  working toolchain; this revision has only been checked by hand-tracing and a standalone `rustc` reproduction of
  the bit logic, not the actual crate.
- Add the `scripts/smoke-joy-keys.ctl` end-to-end script from §5.
