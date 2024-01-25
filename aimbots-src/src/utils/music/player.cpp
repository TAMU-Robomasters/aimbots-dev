#include "player.hpp"

#include <cstdint>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

namespace utils::Music {

static constexpr uint32_t NOTE_Bb = 466;
static constexpr uint32_t NOTE_B = 494;
static constexpr uint32_t NOTE_C = 523;
static constexpr uint32_t NOTE_D = 587;
static constexpr uint32_t NOTE_Eb = 622;
static constexpr uint32_t NOTE_E = 659;
static constexpr uint32_t NOTE_F = 699;
static constexpr uint32_t NOTE_Gb = 740;
static constexpr uint32_t NOTE_G = 784;
static constexpr uint32_t NOTE_Ab2 = 880;
static constexpr uint32_t NOTE_Bb2 = 932;
static constexpr uint32_t NOTE_B2 = 988;
static constexpr uint32_t NOTE_C2 = 1047;
static constexpr uint32_t NOTE_Eb2 = 1244;

static constexpr uint32_t XP_BPM = 110;
static constexpr uint32_t XP_MS_PER_16th = (uint32_t)(((1.0f / XP_BPM) * 60.0f * 1000.0f) / 4.0f);

struct MusicNote {
    uint32_t frequency;
};

static uint32_t lastXPTime = 0;
static uint32_t currentXPNote = 0;
static uint32_t lastXPFreq = 0;
static bool xpTuneFinished = false;

static MusicNote xpStartupNotes[16] = {
    {NOTE_Eb2},
    {NOTE_Eb2},
    {NOTE_Eb2},
    {NOTE_Eb},

    {NOTE_Bb2},
    {0},
    {NOTE_Ab2},
    {NOTE_Ab2},

    {NOTE_Eb},
    {0},
    {NOTE_Eb2},
    {NOTE_Eb2},

    {NOTE_Bb2},
    {NOTE_Bb2},
    {0},
    {0}};

static constexpr size_t XP_NOTE_COUNT = sizeof(xpStartupNotes) / sizeof(MusicNote);

void continuePlayingXPStartupTune(src::Drivers* drivers) {
    if (xpTuneFinished) return;
    if (lastXPTime == 0) lastXPTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastXPTime;

    if (timeSinceLast >= XP_MS_PER_16th) {
        lastXPTime = tap::arch::clock::getTimeMilliseconds();
        if (lastXPFreq != xpStartupNotes[currentXPNote].frequency) tap::buzzer::playNote(&drivers->pwm, xpStartupNotes[currentXPNote].frequency);
        lastXPFreq = xpStartupNotes[currentXPNote].frequency;
        currentXPNote++;
        xpTuneFinished = currentXPNote == XP_NOTE_COUNT;
    }
}

static constexpr uint32_t TD_BPM = 120;
static constexpr uint32_t TD_MS_PER_16th = (uint32_t)(((1.0f / TD_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastTDTime = 0;
static uint32_t currentTDNote = 0;
static uint32_t lastTDFreq = 0;

static MusicNote tokyoDriftNotes[16] = {
    {NOTE_Bb},
    {NOTE_Bb},
    {NOTE_Bb},
    {NOTE_B},

    {NOTE_B},
    {NOTE_B},
    {NOTE_Eb},
    {NOTE_Eb},

    {NOTE_Bb},
    {0},
    {0},
    {0},

    {NOTE_Bb},
    {0},
    {0},
    {0}};

static constexpr size_t TD_NOTE_COUNT = sizeof(tokyoDriftNotes) / sizeof(MusicNote);

void continuePlayingTokyoDriftTune(src::Drivers* drivers) {
    if (lastTDTime == 0) lastTDTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastTDTime;

    if (timeSinceLast >= TD_MS_PER_16th) {
        lastTDTime = tap::arch::clock::getTimeMilliseconds();
        if (lastTDFreq != tokyoDriftNotes[currentTDNote].frequency) tap::buzzer::playNote(&drivers->pwm, tokyoDriftNotes[currentTDNote].frequency);
        lastTDFreq = tokyoDriftNotes[currentTDNote].frequency;
        currentTDNote = (currentTDNote + 1) % TD_NOTE_COUNT;
    }
}

static bool isPacManDone = false;

static constexpr uint32_t PM_BPM = 130;
static constexpr uint32_t PM_MS_PER_16th = (uint32_t)(((1.0f / TD_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastPMTime = 0;
static uint32_t currentPMNote = 0;
static uint32_t lastPMFreq = 0;

static MusicNote pacManNotes[] = {{NOTE_D}, {NOTE_E}, {NOTE_F}, {NOTE_G},
                                  {NOTE_E}, {NOTE_E}, {NOTE_C}, {NOTE_D},
                                  {NOTE_D}, {NOTE_D}, {NOTE_D}, {NOTE_D},
                                  {}};

static constexpr size_t PM_NOTE_COUNT = sizeof(pacManNotes) / sizeof(MusicNote);

void playPacMan(src::Drivers* drivers) {
    if (isPacManDone) return;
    if (lastPMTime == 0) lastPMTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastPMTime;

    if (timeSinceLast >= PM_MS_PER_16th) {
        lastPMTime = tap::arch::clock::getTimeMilliseconds();
        if (lastPMFreq != pacManNotes[currentPMNote].frequency) tap::buzzer::playNote(&drivers->pwm, pacManNotes[currentPMNote].frequency);
        lastPMFreq = pacManNotes[currentPMNote].frequency;
        currentPMNote++;
        isPacManDone = currentPMNote == PM_NOTE_COUNT;
    }
}

}  // namespace utils::Music