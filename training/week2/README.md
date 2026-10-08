# Week 2: Parse the remote yourself, then make some noise with the buzzer

**Goal:** throw away taproot's remote code and read the DT7 remote **yourself**, straight from the UART bytes. Then use your parser to drive the dev board's buzzer:

- **Right switch DOWN** → the buzzer beeps.
- **Right stick up/down** → pitch (frequency).
- **Right stick left/right** → note length (beep on for L ms, off for L ms, repeat).
- **Right switch UP** → your week 1 LED still blinks, but now through **your** parser.

You only parse the **right stick** and the **right switch**.

Almost every spot you need to change is marked `TODO(week2)`. **The one exception is `training_control.cpp`:** there are no week 2 TODOs in it (you already edited it in week 1, and new TODOs would cause merge conflicts). You add your buzzer command, its mapping, and its `addMap` call there yourself, **the same way you added your `BlinkLedCommand` in week 1** (section 8). 

---

## 0. Get the week 2 starter code

The starter files were added to `training-2026` after you branched. Merge them into your branch:

```bash
git fetch
git switch training/<your-name>
git merge origin/training-2026
```

New files you get:

```
aimbots-src/src/robots/training/
├── training_config.hpp          ← the TRAINING_USE_MY_REMOTE switch (section 5)
├── remote/
│   ├── dt7_remote.hpp           ← given: the class shape, constants and getters
│   └── dt7_remote.cpp           ← YOUR parser
└── commands/
    ├── buzzer_command.hpp       ← YOUR command (add members)
    └── buzzer_command.cpp
```

You'll also fill in  `TODO(week2)` spots in two files outside of training folder, `src/drivers.hpp` and `src/main.cpp`. This is the first time you touch shared code (used by every robot).
Build before you change anything:

```bash
cd aimbots-src
pipenv run scons build robot=TRAINING
```

`drivers.hpp` changed, so this is a full rebuild (~5 minutes). After that, builds are incremental again.

**Read first:** [`dt7_dr16_protocol_v1.4_en.pdf`](dt7_dr16_protocol_v1.4_en.pdf), an English translation of DJI's *RoboMasters Remote Controller Control Protocol V1.4*. The Chinese original is next to it: [`dt7_dr16_protocol_v1.4_cn.pdf`](dt7_dr16_protocol_v1.4_cn.pdf).

>  Taproot has its own parser in `taproot/src/tap/communication/serial/remote.cpp`. **Don't open it until you're checked off.** The point is to go from *document → bytes → code* yourself, like the LED pin in week 1.

---

## 1. UART in one screen

**UART** sends bytes one bit at a time over a single wire (TX on one side → RX on the other), with no shared clock. Both sides just agree on the speed ahead of time.

One byte on the wire looks like this:

```
idle ─┐   ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬───── idle
      └───┘0│1│2│3│4│5│6│7│P│stop
     start   8 data bits     parity
            (LSB first)
```

| Setting | Meaning | DBUS uses |
|---|---|---|
| **Baud rate** | Bits per second. Both sides must match, or every byte is garbage. | look it up |
| **Data bits** | Bits per byte | 8 |
| **Parity** | One extra bit so the number of 1s is even (or odd). It's a cheap check that catches single-bit errors. | look it up |
| **Stop bits** | How long the line goes idle before the next byte | 1 |

People write this as `8E1` (8 data, Even parity, 1 stop) or `8N1` (No parity). With start + 8 + parity + stop, **one byte takes 11 bits on the wire**. Remember that for section 4.

**How does that compare to the rest of the robot?**

| Bus | Wires | Used for on our robot |
|---|---|---|
| UART | TX, RX (point to point) | Remote (DBUS), referee system, Jetson |
| CAN | CAN-H, CAN-L (many devices share it) | Every DJI motor, board-to-board |
| SPI | clock, MOSI, MISO, chip select | The IMU on the dev board |
| I²C | clock, data (addressed devices) | Actually doesn't work on devboard type c due to hardware bug 😭 |

---

## 2. The DBUS frame

The receiver sends one **18-byte frame every 7 ms** (~140 frames per second). Each frame holds every stick, switch, the mouse and the keyboard. There's **no header byte and no checksum**; you'll see why that matters in section 4.

The sticks are **11-bit numbers** (0–2047), but bytes are 8 bits. So DJI packs them back to back as one long stream of bits, **least significant bit first**, ignoring byte boundaries:

```
bit:   0          10 11         21 22 ...                   43 44 45 46 47
       [   ch0 (11)  ][   ch1 (11)  ][ ch2 (11) ][ ch3 (11) ][S2 ][S1 ]
byte:  |  byte 0  |  byte 1  |  byte 2  |  byte 3  |  byte 4  |  byte 5  |
```

You only need these:

| Field | Bits | What it is | Valid values |
|---|---|---|---|
| ch0 | 0–10 | right stick, horizontal | 364 … **1024 (center)** … 1684 |
| ch1 | 11–21 | right stick, vertical | 364 … **1024 (center)** … 1684 |
| S2 | 44–45 | right switch | 1 = UP, 3 = MID, 2 = DOWN |

(Double-check the numbers in the protocol PDF.)

> 🕵️ **But sometimes the source document is wrong too.** DJI's own protocol contradicts itself about the switches. Its table puts **S1** at bits 44–45 and **S2** at 46–47. But its picture of the remote (page 3 of the Chinese original) puts S1 on the **left** and S2 on the **right**, and its own reference code reads bits 44–45 as the **right** switch. We checked on a real board: **bits 44–45 are the right switch (S2)**, so the table has S1 and S2 swapped. This happens all the time with datasheets. When two parts of a document disagree, test it on hardware: flip one switch at a time and watch byte 5 in MCUViewer.
>
> The protocol has a few more mistakes in parts you don't need this week. If you try the "parse the rest" goal, note that the mouse buttons and keyboard are really at bit offsets 96, 104 and 112 (bytes 12, 13 and 14–15, as the struct's byte comments say), not 86, 94 and 102.

### The tools: shift and mask

- `x >> n` slides the bits of `x` right by `n`, throwing away the bottom `n` bits.
- `x << n` slides them left, making room at the bottom.
- `x & 0x7FF` keeps only the bottom 11 bits (`0x7FF` = `0b111_1111_1111`). That's called a **mask**.
- `a | b` glues two pieces together when they don't overlap.

---

## 3. Worked example: decode one frame by hand

Here's a frame (hex) with the right stick pushed half right and fully up, and the right switch DOWN:

```
4A A5 34 00 01 E8 00 00 00 00 00 00 00 00 00 00 00 04
```

The first six bytes in binary (bit 7 on the left, bit 0 on the right, the way you normally write numbers):

```
byte 0 = 0x4A = 0100 1010
byte 1 = 0xA5 = 1010 0101
byte 2 = 0x34 = 0011 0100
byte 5 = 0xE8 = 1110 1000
```

**ch0 (bits 0–10)** is all 8 bits of byte 0, plus the **low 3 bits** of byte 1 on top:

```
byte 1 & 0b111  = 101          → goes in bits 8-10
byte 0          = 0100 1010    → bits 0-7
ch0             = 101 0100 1010 = 1354
```

1354 − 1024 = **+330**, so the stick is halfway (330 / 660) to the right. ✔

**ch1 (bits 11–21)** starts at bit 3 of byte 1. Take byte 1's **top 5 bits** as ch1's low bits, then byte 2's **low 6 bits** on top:

```
byte 1 >> 3       = 1 0100       = 20      → ch1 bits 0-4
byte 2 & 0b111111 = 11 0100      = 52      → ch1 bits 5-10
ch1 = 20 | (52 << 5) = 20 + 1664 = 1684
```

1684 − 1024 = **+660**: fully up. ✔

**S2, the right switch (bits 44–45)**, is bits 4–5 of byte 5 (44 = 5 × 8 + 4):

```
byte 5 = 1110 1000
(byte 5 >> 4) & 0b11 = 10 = 2 → DOWN ✔
```

Now write that in C++. Two common styles (pick one):
1. **Per field:** combine the 2–3 bytes each field touches, shift, mask (exactly what you just did by hand).
2. **One big number:** glue bytes 0–5 into one `uint64_t` (`bits |= uint64_t(byte[i]) << (8 * i)`). Then every field is just `(bits >> offset) & mask`, using the bit offsets straight from the table.

Before you flash anything, check your code by hand against this example frame.

---

## 4. From a stream of bytes to frames

The UART hands you bytes, not frames (frames mean messages). If you start reading halfway through a frame and just count to 18, **every frame after that is shifted**, and your "ch0" is really the end of ch2. You need to know where a frame **starts**.

**How other protocols do it:** most protocols start each message with a **magic/header byte** and end with a **checksum**. For example, the referee system's frames start with `0xA5` and carry a CRC. You search for the header, read the length, and verify the checksum.

**DBUS has neither, so we use timing.** At 100,000 baud, 18 bytes × 11 bits = 198 bits ≈ **2 ms** to send a frame. Then the line is quiet until the next frame, 7 ms after the last one started:

```
|<- 2 ms ->|<-- ~5 ms silence -->|<- 2 ms ->|
[ 18 bytes ]                              [ 18 bytes ]
```

So: **if more than a few ms passed since the last byte, this byte is the start of a new frame.** Throw away any partial frame and start counting from 0. (`FRAME_GAP_MS = 3` in the starter.)

**Validate before you trust a frame.** Without a checksum, garbage can still look like a frame. These can never happen on a good frame, so reject them:
- a stick value outside 364–1684
- a switch value of 0

Count rejects in `badFramesDisplay`, keep the last good values, and wait for the next frame.

**Handle unplugging.** If no good frame arrives for 100 ms, the remote is gone (turned off, out of range, receiver unplugged). Reset the sticks to center and the switch to `UNKNOWN`, and tell the command mapper, so every running command ends and the robot goes to a safe state. On a real robot this is what stops it driving away when the remote dies.

---

## 5. Hooking your class into the robot (and what the `#` lines are)

### The preprocessor

Before the compiler sees a `.cpp` file, the **preprocessor** runs over it. Every line starting with `#` is an instruction for the preprocessor, not C++:

| Directive | What it does |
|---|---|
| `#include "x.hpp"` | Paste the contents of `x.hpp` right here. |
| `#define NAME value` | From here on, replace `NAME` with `value` (text substitution). |
| `#ifdef NAME` … `#endif` | Keep these lines only if `NAME` is defined (to anything). Otherwise **delete them** before compiling. |
| `#if EXPR` … `#else` … `#endif` | Keep the first block if `EXPR` is non-zero, otherwise the `#else` block. |
| `#pragma once` | Only paste this header once per `.cpp`, even if it's included many times. |

**Where does `TARGET_TRAINING` come from?** Not from any file. When you run `scons build robot=TRAINING`, SCons passes the compiler `-D TARGET_TRAINING` (look for `"-D " + args["ROBOT_TYPE"]` in `aimbots-src/SConstruct`). `-D` means "act as if `#define TARGET_TRAINING` were at the top of every file". Build `robot=SENTRY_SWERVE` and it's `TARGET_SENTRY_SWERVE` instead.

That's why every file in `robots/training/` is wrapped in `#ifdef TARGET_TRAINING`. For every other robot, the preprocessor deletes your whole file before the compiler sees it, so **nothing you write can break another robot's build**. The fenced blocks in `drivers.hpp` and `main.cpp` work the same way.

### Why only one remote can run

Taproot's remote (`drivers->remote`) and yours both want the bytes from `Uart3`. A byte read by one is gone for the other, so if both ran, each would get random halves of frames. `training_config.hpp` has a switch:

```cpp
#define TRAINING_USE_MY_REMOTE 0   // 0 = taproot's remote, 1 = yours
```

And `main.cpp` uses `#if` to compile in **exactly one** of them:

```cpp
#if TRAINING_USE_MY_REMOTE
    // TODO(week2): read YOUR remote here
#else
    drivers->remote.read();
#endif
```

### What is `Drivers`?

`src::Drivers` (in `src/drivers.hpp`) is one big object that owns every piece of shared hardware code: the UARTs, CAN, PWM, IMU, the command scheduler and the command mapper. Everything gets to it through the `drivers` pointer. To make your remote reachable as `drivers->dt7Remote`, add it there.

### Steps

**In `src/drivers.hpp`** (three fenced spots, each marked `TODO(week2)`):
- [ ] Include `"robots/training/remote/dt7_remote.hpp"`.
- [ ] Declare the member: `src::Training::Dt7Remote dt7Remote;`
- [ ] Construct it in the constructor list: `dt7Remote(this),`

  Keep both exactly where the TODOs are. C++ constructs members **in the order they're declared**, not the order you list them in the constructor. If the two orders differ, you get a `-Wreorder` warning.

**In `src/main.cpp`** (two fenced spots):
- [ ] `initializeIo()`: call `drivers->dt7Remote.initialize();`
- [ ] `updateIo()`: call `drivers->dt7Remote.read();`. `updateIo` runs as fast as the loop can go, much faster than the 2 ms scheduler, so no byte waits long.

**In `robots/training/training_config.hpp`:**
- [ ] Set `TRAINING_USE_MY_REMOTE` to `1` once your parser is written.

> ⚠️ Only edit inside the fences. A change outside a `#ifdef TARGET_TRAINING` block in `drivers.hpp` or `main.cpp` changes **every robot**. For training it doesn't matter but for real projects it does. 
---

## 6. Write the parser (`remote/dt7_remote.cpp`)

The header already has the constants, the members you need, and getters that turn your raw values into −1…1. You write three functions:

- [ ] **`initialize()`**: start `Uart3` with the right baud rate and parity (from the protocol doc):
  ```cpp
  drivers->uart.init<Uart::UartPort::Uart3, BAUD, Uart::Parity::PARITY>();
  ```
- [ ] **`read()`**:
  - Read every waiting byte: `drivers->uart.read(Uart::UartPort::Uart3, &byte)` returns `false` when the UART is empty.
  - Do the frame sync from section 4.
  - On a full frame, call `parseFrame()`. If it's good, hand the switch to the command mapper:
    ```cpp
    drivers->commandMapper.handleKeyStateChange(0, SwitchState::UNKNOWN, rightSwitch, false, false);
    //                                          keys  left switch          right switch  mouse L/R
    ```
    **This line is what makes `HoldCommandMapping`s work.** Taproot's remote does the same thing internally, which is how your week 1 mapping ever saw the switch. Your parser doesn't read the left switch, keys or mouse, so pass "nothing".
  - Check for disconnect (section 4) and send all-`UNKNOWN` to the mapper.
- [ ] **`parseFrame()`**:
  - Decode ch0, ch1 and the right switch (section 3).
  - Validate (section 4).
  - On a good frame, store `channel - CHANNEL_CENTER` and `static_cast<SwitchState>(rightSwitchRaw)`.
  - Update `framesParsedDisplay` / `badFramesDisplay` / `rightVerticalDisplay` for MCUViewer.

**Test it before the buzzer:** flip `TRAINING_USE_MY_REMOTE` to 1, build, flash, and open MCUViewer.
- `framesParsedDisplay` should climb by ~140 per second.
- `badFramesDisplay` should stay at ~0.
- `rightVerticalDisplay` should follow the stick from −1 to 1.
- **Your week 1 LED should still blink on right switch UP.** That proves your switch → command mapper path works.

---

## 7. The buzzer command (`commands/buzzer_command.*`)

It uses the same `TrainingBoardSubsystem` as week 1 (same reason: the scheduler won't run a command with no subsystem).

### Playing sounds

The dev board has a small **passive buzzer** on a PWM pin (`Timer4`). A passive buzzer only clicks when its voltage changes, so a 50% square wave at *f* Hz gives a tone at *f* Hz. Taproot already wraps that:

```cpp
#include "tap/communication/sensors/buzzer/buzzer.hpp"

tap::buzzer::playNote(&drivers->pwm, 440);   // A4, 440 Hz. playNote(…, 0) = silence
tap::buzzer::silenceBuzzer(&drivers->pwm);
```

### Map the stick

| Stick | Range | Constant |
|---|---|---|
| Vertical −1 (down) → +1 (up) | 20 Hz → 15,000 Hz | `MIN_FREQUENCY_HZ`, `MAX_FREQUENCY_HZ` |
| Horizontal −1 (left) → +1 (right) | 50 ms → 1000 ms | `MIN_NOTE_MS`, `MAX_NOTE_MS` |

**Why 20 Hz – 15 kHz?** That's about what a ~30-year-old can hear. The low end (~20 Hz) is the same for everyone. The top end drops with age: up to ~20 kHz as a kid, ~15–16 kHz by 30, lower after that. Push the stick all the way up and see who in the room still hears it.

A linear map from −1…1 to [low, high] is: `low + (stick + 1) / 2 × (high − low)`.

### Beep on, beep off

Same pattern as the week 1 LED: a `tap::arch::MilliTimeout`, and every time it expires, flip between "note on" (play the frequency) and "note off" (silence). Then restart it with the note length from the stick.

### ⚠️ Buzzer gotchas (read this before you debug for an hour)

Taproot's buzzer function has two quirks that bite exactly this assignment:

1. **Don't call `playNote` every `execute()`.**
   - Every call reprograms the PWM timer, and reprogramming **restarts the waveform from zero**. Called every 2 ms, a note longer than one 2 ms period never finishes a cycle.
   - Result: anything below ~500 Hz turns into a 500 Hz buzz, below ~250 Hz is silent, and higher notes get a 500 Hz click on top.
   - Only call it when a note **starts**, or when the frequency **changed by more than `FREQUENCY_DEADBAND_HZ`**. The sticks jitter by a count or two even when you don't touch them.
2. **When you do call it, call it twice in a row.**
   - `playNote` sets the 50% duty cycle **before** it changes the frequency, so the duty is calculated from the *previous* note's period.
   - Jump from a low note to a high one and the "50%" can come out as 100%: a constant HIGH, which is silent.
   - The second call runs with the right period already set, so it fixes the duty.
   - Bonus question: open `taproot/src/tap/communication/sensors/buzzer/buzzer.cpp` and explain why.

Also expect this:
- **The robot plays a startup song.** Once the IMU warms up to 50 °C (a little after boot), `main.cpp` plays the robot's startup song on the same buzzer. Wait for it to finish before testing, because it will fight your command while it plays.

> 🎵 **Curious how we play actual songs?** That's the jukebox: [`utils/music/jukebox_player.cpp`](../../aimbots-src/src/utils/music/jukebox_player.cpp) plays a song note by note **without blocking**, using the same "check the clock, act when time is up" idea as your command. The note frequencies and durations are in [`jukebox_player.hpp`](../../aimbots-src/src/utils/music/jukebox_player.hpp), the songs themselves in [`sheetmusic_constants.hpp`](../../aimbots-src/src/utils/music/sheetmusic_constants.hpp), and an older version is in [`player.cpp`](../../aimbots-src/src/utils/music/player.cpp). You don't need any of it this week, but adding a song is a fun play-around goal.

---

## 8. The assignment

### Step by step

**Parser (sections 5–6):**
- [ ] Fill in the `TODO(week2)` fences in `drivers.hpp` and `main.cpp`.
- [ ] Write `initialize()`, `read()` and `parseFrame()` in `remote/dt7_remote.cpp`.
- [ ] Set `TRAINING_USE_MY_REMOTE` to `1`, then build, flash, and check MCUViewer + the week 1 LED.

**Buzzer (section 7):**
- [ ] `buzzer_command.hpp`: add the members you need (timer, note on/off, last frequency sent).
- [ ] `buzzer_command.cpp`:
  - [ ] `initialize()`: silence, then start the timer.
  - [ ] `execute()`: map the stick, flip on/off when the timer expires, and follow the stick mid-note (with the deadband).
  - [ ] `end()`: silence.
- [ ] `training_control.cpp` (**no TODOs here**; do exactly what you did for `BlinkLedCommand` in week 1):
  - [ ] `#include "robots/training/commands/buzzer_command.hpp"` next to the blink command's include.
  - [ ] Create a `BuzzerCommand`, passing it `drivers()` and `&board`, under your `BlinkLedCommand`.
  - [ ] Add a `HoldCommandMapping` on `RIGHT_SWITCH` / `DOWN` that runs it, under your week 1 mapping.
  - [ ] Register it in `registerIOMappings()` with another `drivers->commandMapper.addMap(...)`.
  - Keep your week 1 command and mapping. The LED on UP and the buzzer on DOWN both need to work.

### Checkoff
- [ ] Compiles with `pipenv run scons build robot=TRAINING` with no new errors or warnings from your files.
- [ ] `TRAINING_USE_MY_REMOTE` is `1`, and taproot's `remote.read()` is not being called.
- [ ] Right switch UP → LED blinks (week 1, through your parser).
- [ ] Right switch DOWN → buzzer beeps. Stick up/down changes the pitch, left/right changes the note length.
- [ ] Right switch MID, or turn the remote off → buzzer silent within ~100 ms.
- [ ] `badFramesDisplay` stays at ~0 while you wiggle everything.
- [ ] You can answer:
  - DBUS has no header byte. How does your code know where a frame starts?
  - What would happen to ch0 if your frame were shifted by one byte? Would your validation catch it every time?
  - Why does the `TARGET_TRAINING` fence in `drivers.hpp` keep the sentry's build safe?
  - Why must the remote call `commandMapper.handleKeyStateChange`? What breaks if it doesn't?
  - Why does calling `playNote` every 2 ms ruin a 200 Hz note?

### Play around try completing these goals
- **Musical stick:** snap the frequency to the nearest note of a scale (the `NoteFreq` values in `jukebox_player.hpp`) so the stick plays real notes.
- **Exponential pitch:** pitch sounds even to us when the frequency *doubles* (one octave), not when it goes up by a fixed number of Hz. Map the stick so every equal stick movement is an equal musical step: `f = MIN × (MAX / MIN)^((stick + 1) / 2)`. Compare it to the linear map.
- **Parse the rest:** add the left stick (ch2, ch3), the left switch (S1, bits 46–47) and the wheel, and pass the left switch to the command mapper too.
- **Your own song:** add a short song to `sheetmusic_constants.hpp` and play it with `drivers->musicPlayer.requestSong(...)` when the switch goes to MID.

---

## 9. Potential problems

| Symptom | Likely cause |
|---|---|
| `comparison of integer expressions of different signedness` (it's an **error** here, not a warning) | Comparing `int` with `size_t`/`uint32_t`, e.g. `for (int i = 0; i < FRAME_LENGTH; i++)`. Use `size_t i`. Our build turns `-Werror=sign-compare` on. |
| `'dt7Remote' is not a member of 'src::Drivers'` | You haven't done the `drivers.hpp` steps yet, or one of them is outside the `#ifdef TARGET_TRAINING` fence. |
| `'Dt7Remote' in namespace 'src::Training' does not name a type` in `drivers.hpp` | The `#include` for `dt7_remote.hpp` is missing from the first fence. |
| `will be initialized after [-Wreorder]` | The member declaration and the `dt7Remote(this),` line aren't in the same relative spot. Put them exactly where the TODOs are. |
| `reference to 'Remote' is ambiguous` | You named something `Remote`. Taproot already has a `Remote`, and `training_control.cpp` has `using namespace tap::communication::serial`. Keep the name `Dt7Remote`. |
| Errors in every robot / `TRAINING_USE_MY_REMOTE` warnings | You edited `drivers.hpp`/`main.cpp` outside the fences, or removed the fallback `#define` in `robot_specific_defines.hpp`. |
| `framesParsedDisplay` stays at 0 | Wrong baud rate or parity in `initialize()`, the flag is still `0` (taproot is eating the bytes), or `read()` isn't being called from `main.cpp`. |
| `badFramesDisplay` climbs fast | Frame sync is wrong (shifted frames). Check the gap logic and that you reset the byte count after each frame. Wrong bit offsets also look like this. |
| Stick values jump around or move the wrong axis | Bit offsets/masks are off. Decode the section 3 example frame by hand with your code. |
| LED / buzzer never start, even though `framesParsedDisplay` climbs | You never call `commandMapper.handleKeyStateChange`, or you pass the switch in the left-switch slot. |
| LED works on UP, but nothing happens on DOWN | The buzzer mapping isn't in `training_control.cpp`, or you created it but never called `addMap()` on it (same as the week 1 mistake). |
| Buzzer silent on some notes, or buzzy and clicky | The two buzzer gotchas in section 7. |
| Random tune plays over your beeps after boot | That's the startup song. Wait for it to finish. |
| Builds take 10 minutes every time | Expected once after editing `drivers.hpp` (everything includes it). Edits to only your `.cpp` files rebuild quickly. |
