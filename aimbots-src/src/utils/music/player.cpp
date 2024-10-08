#include "player.hpp"

#include <cstdint>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

namespace utils::Music {

// built around A = 440 Hz
static constexpr uint32_t NOTE_E4 = 330;
static constexpr uint32_t NOTE_F4 = 340;
static constexpr uint32_t NOTE_Gb4 = 370;
static constexpr uint32_t NOTE_G4 = 392;
static constexpr uint32_t NOTE_Ab4 = 415;
static constexpr uint32_t NOTE_A4 = 440;
static constexpr uint32_t NOTE_Bb4 = 469;
static constexpr uint32_t NOTE_B4 = 494;
static constexpr uint32_t NOTE_C5 = 523;
static constexpr uint32_t NOTE_Db5 = 554;
static constexpr uint32_t NOTE_D5 = 588;
static constexpr uint32_t NOTE_Eb5 = 622;
static constexpr uint32_t NOTE_E5 = 659;
static constexpr uint32_t NOTE_F5 = 699;
static constexpr uint32_t NOTE_Gb5 = 740;
static constexpr uint32_t NOTE_G5 = 784;
static constexpr uint32_t NOTE_Ab5 = 830;
static constexpr uint32_t NOTE_A5 = 880;
static constexpr uint32_t NOTE_Bb5 = 932;
static constexpr uint32_t NOTE_B5 = 988;
static constexpr uint32_t NOTE_C6 = 1047;
static constexpr uint32_t NOTE_Db6 = 1108;
static constexpr uint32_t NOTE_D6 = 1174;
static constexpr uint32_t NOTE_Eb6 = 1244;
static constexpr uint32_t NOTE_E6 = 1318;
static constexpr uint32_t NOTE_F6 = 1396;
static constexpr uint32_t NOTE_G6 = 1567;
static constexpr uint32_t NOTE_A6 = 1760;
static constexpr uint32_t NOTE_Bb6 = 1864;
static constexpr uint32_t NOTE_D7 = 2349;

struct MusicNote {
    uint32_t frequency;
};

static bool isSongDone = false;
// We Are Number One

static constexpr uint32_t WENUM1_BPM = 168;
static constexpr uint32_t WENUM1_MS_PER_16TH = (uint32_t)(((1.0f / WENUM1_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastWeNum1Time = 0;
static uint32_t currentWeNum1Note = 0;
static uint32_t lastWeNum1Freq = 0;

static MusicNote weAreNumberOneNotes[] = {
    {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_C6},  {NOTE_C6},  {NOTE_B5},  {NOTE_C6},
    {NOTE_B5},  {NOTE_C6},  {NOTE_B5},  {NOTE_B5},  {NOTE_C6},  {NOTE_C6},  {NOTE_Ab5}, {NOTE_Ab5}, {NOTE_Ab5}, {NOTE_Ab5},
    {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_F5},  {NOTE_Ab4}, {NOTE_Ab4},
    {NOTE_C6},  {NOTE_C6},  {NOTE_Db6}, {NOTE_Db6}, {NOTE_Db6}, {NOTE_Db6}, {NOTE_Ab4}, {NOTE_Ab4}, {NOTE_Ab4}, {NOTE_Ab4},
    {NOTE_Db6}, {NOTE_Db6}, {NOTE_Db6}, {NOTE_Db6}, {NOTE_Eb6}, {NOTE_Eb6}, {NOTE_Eb6}, {NOTE_Eb6}, {NOTE_C6},  {NOTE_C6},
    {NOTE_Db6}, {NOTE_Db6}, {NOTE_C6},  {NOTE_C6},  {NOTE_Db6}, {NOTE_Db6}, {NOTE_C6},  {NOTE_C6},  {NOTE_C6},  {NOTE_C6},
    {NOTE_E6},  {NOTE_E6},  {NOTE_E6},  {NOTE_E6},  {0}};

static constexpr size_t WENUM1_NOTE_COUNT = sizeof(weAreNumberOneNotes) / sizeof(MusicNote);

void playWeAreNumberOne(src::Drivers* drivers) {
    if (isSongDone) return;
    if (lastWeNum1Time == 0) lastWeNum1Time = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastWeNum1Time;

    if (timeSinceLast >= WENUM1_MS_PER_16TH) {
        lastWeNum1Time = tap::arch::clock::getTimeMilliseconds();
        if (lastWeNum1Freq != weAreNumberOneNotes[currentWeNum1Note].frequency)
            tap::buzzer::playNote(&drivers->pwm, weAreNumberOneNotes[currentWeNum1Note].frequency);
        lastWeNum1Freq = weAreNumberOneNotes[currentWeNum1Note].frequency;
        currentWeNum1Note++;
        isSongDone = currentWeNum1Note == WENUM1_NOTE_COUNT;
    }
}

// PacMan Theme

static constexpr uint32_t PM_BPM = 130;
static constexpr uint32_t PM_MS_PER_16TH = (uint32_t)(((1.0f / PM_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastPMTime = 0;
static uint32_t currentPMNote = 0;
static uint32_t lastPMFreq = 0;

static MusicNote pacManNotes[] = {{NOTE_B4},  {NOTE_B5},  {NOTE_Gb5}, {NOTE_Eb5},

                                  {NOTE_B5},  {0},        {NOTE_Eb5}, {0},

                                  {NOTE_C5},  {NOTE_C6},  {NOTE_G5},  {NOTE_E5},

                                  {NOTE_C6},  {0},        {NOTE_E5},  {0},

                                  {NOTE_B4},  {NOTE_B5},  {NOTE_G5},  {NOTE_E5},

                                  {NOTE_B5},  {0},        {NOTE_Eb5}, {0},

                                  {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_F5},  {NOTE_F5},

                                  {NOTE_G5},  {NOTE_G5},  {NOTE_B5},  {0}};

static constexpr size_t PM_NOTE_COUNT = sizeof(pacManNotes) / sizeof(MusicNote);

void playPacMan(src::Drivers* drivers) {
    if (isSongDone) return;
    if (lastPMTime == 0) lastPMTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastPMTime;

    if (timeSinceLast >= PM_MS_PER_16TH) {
        lastPMTime = tap::arch::clock::getTimeMilliseconds();
        if (lastPMFreq != pacManNotes[currentPMNote].frequency)
            tap::buzzer::playNote(&drivers->pwm, pacManNotes[currentPMNote].frequency);
        lastPMFreq = pacManNotes[currentPMNote].frequency;
        currentPMNote++;
        isSongDone = currentPMNote == PM_NOTE_COUNT;
    }
}

// Chainsaw Man theme

static constexpr uint32_t CHNSW_BPM = 350;
static constexpr uint32_t CHNSW_MS_PER_8TH =
    (uint32_t)(((1.0f / CHNSW_BPM) * 60.0f * 1000.0f) / 2.0f);  // halves the overall size of the note array

static uint32_t lastCHNSWTime = 0;
static uint32_t currentCHNSWNote = 0;
static uint32_t lastCHNSWFreq = 0;

static MusicNote chainSawNotes[72] = {
    {NOTE_Ab5}, {NOTE_Ab5}, {NOTE_Db5}, {NOTE_Db5}, {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_E5},  {NOTE_Gb5}, {NOTE_Gb5}, {NOTE_A4},
    {NOTE_A4},  {NOTE_E5},  {NOTE_E5},  {NOTE_E5},  {NOTE_A4},  {NOTE_A4},  {NOTE_A4},  {NOTE_A4},  {NOTE_Ab4}, {NOTE_Ab4},
    {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_Ab4}, {NOTE_Gb4}, {NOTE_Gb4}, {NOTE_Ab4}, {NOTE_Ab4}, {NOTE_Db5}, {NOTE_Db5}, {NOTE_Db5},
    {0},        {0},        {NOTE_Ab4}, {NOTE_Ab4}, {NOTE_Gb4}, {NOTE_E4},  {NOTE_Gb4}, {NOTE_Gb4}, {NOTE_Ab4}, {NOTE_Ab4},
    {NOTE_Bb4}, {NOTE_B4},  {NOTE_B4},  {NOTE_A4},  {NOTE_Ab4}, {NOTE_Gb4}, {NOTE_Gb4}, {NOTE_Db5}, {NOTE_Db5}, {NOTE_Db5},
    {NOTE_Db5}, {NOTE_B4},  {NOTE_A4},  {NOTE_Ab4}, {NOTE_E5},  {NOTE_E5},  {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_Eb5}, {NOTE_Eb5},
    {NOTE_Eb5}, {0},        {0},        {NOTE_Ab5}, {NOTE_Ab5}, {NOTE_Db5}, {NOTE_Db5}, {NOTE_Db5}, {NOTE_Db5}

};

static constexpr size_t CHNSW_NOTE_COUNT = sizeof(chainSawNotes) / sizeof(MusicNote);

void playChainSawMan(src::Drivers* drivers) {
    if (isSongDone) return;
    if (lastCHNSWTime == 0) lastCHNSWTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastCHNSWTime;

    if (timeSinceLast >= CHNSW_MS_PER_8TH) {
        lastCHNSWTime = tap::arch::clock::getTimeMilliseconds();
        if (lastCHNSWFreq != chainSawNotes[currentCHNSWNote].frequency)
            tap::buzzer::playNote(&drivers->pwm, chainSawNotes[currentCHNSWNote].frequency);
        lastCHNSWFreq = chainSawNotes[currentCHNSWNote].frequency;
        currentCHNSWNote++;
        isSongDone = currentCHNSWNote == CHNSW_NOTE_COUNT;
    }
}

// Mystery Song

static constexpr uint32_t MYST_BPM = 114;
static constexpr uint32_t MYST_MS_PER_16TH = (uint32_t)(((1.0f / MYST_BPM) * 60.0f * 1000.0f) / 4.0f);

static uint32_t lastMYSTTime = 0;
static uint32_t currentMYSTNote = 0;
static uint32_t lastMYSTFreq = 0;

static MusicNote mysteryNotes[] = {
    {NOTE_G4}, {NOTE_A4}, {NOTE_C5}, {NOTE_A4}, {NOTE_E5}, {NOTE_E5}, {NOTE_E5}, {NOTE_E5}, {NOTE_E6}, {NOTE_E6}, {NOTE_D5},
    {NOTE_D5}, {NOTE_D5}, {NOTE_D5}, {NOTE_G4}, {NOTE_A4}, {NOTE_C5}, {NOTE_A4}, {NOTE_D6}, {NOTE_D6}, {NOTE_D6}, {NOTE_D5},
    {NOTE_D5}, {NOTE_D5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_B4}, {NOTE_A4}, {NOTE_A4}, {NOTE_G4}, {NOTE_A4}, {NOTE_C5},
    {NOTE_A4}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_D5}, {NOTE_D5}, {NOTE_B4}, {NOTE_B4}, {NOTE_B4}, {NOTE_A4},
    {NOTE_G4}, {NOTE_G4}, {NOTE_G4}, {NOTE_G4}, {NOTE_G5}, {NOTE_G5}, {NOTE_D5}, {NOTE_D5}, {NOTE_D5}, {NOTE_D5}, {NOTE_C5},
    {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {NOTE_C5}, {0}};

static constexpr size_t MYST_NOTE_COUNT = sizeof(mysteryNotes) / sizeof(MusicNote);

void playMystery(src::Drivers* drivers) {
    if (isSongDone) return;
    if (lastMYSTTime == 0) lastMYSTTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastMYSTTime;

    if (timeSinceLast >= MYST_MS_PER_16TH) {
        lastMYSTTime = tap::arch::clock::getTimeMilliseconds();
        if (lastMYSTFreq != mysteryNotes[currentMYSTNote].frequency)
            tap::buzzer::playNote(&drivers->pwm, mysteryNotes[currentMYSTNote].frequency);
        lastMYSTFreq = mysteryNotes[currentMYSTNote].frequency;
        currentMYSTNote++;
        isSongDone = currentMYSTNote == MYST_NOTE_COUNT;
    }
}

static constexpr uint32_t CRAB_BPM = 125;
static constexpr uint32_t CRAB_MS_PER_16TH =
    (uint32_t)(((1.0f / CRAB_BPM) * 60.0f * 1000.0f) / 4.0f);  // halves the overall size of the note array

static uint32_t lastCRABTime = 0;
static uint32_t currentCRABNote = 0;
static uint32_t lastCRABFreq = 0;

static MusicNote crabRaveNotes[] = {{NOTE_D5}, {NOTE_D5}, {NOTE_Bb5}, {NOTE_Bb5}, {NOTE_G6}, {NOTE_G6}, {NOTE_G6},
                                    {NOTE_D5}, {NOTE_D5}, {NOTE_D6},  {NOTE_A5},  {NOTE_A5}, {NOTE_F5}, {NOTE_F5},
                                    {NOTE_F5}, {NOTE_F5}, {NOTE_D5},  {NOTE_D5},  {NOTE_D6}, {NOTE_A5}, {NOTE_A5},
                                    {NOTE_F5}, {NOTE_F5}, {NOTE_F5},  {NOTE_C5},  {NOTE_C5}, {NOTE_C5}, {NOTE_C5},
                                    {NOTE_E5}, {NOTE_E5}, {NOTE_E5},  {NOTE_F5},  {0}};

static constexpr size_t CRAB_NOTE_COUNT = sizeof(crabRaveNotes) / sizeof(MusicNote);

void playCrabRave(src::Drivers* drivers) {
    if (isSongDone) return;
    if (lastCRABTime == 0) lastCRABTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - lastCRABTime;

    if (timeSinceLast >= CRAB_MS_PER_16TH) {
        lastCRABTime = tap::arch::clock::getTimeMilliseconds();
        if (lastCRABFreq != crabRaveNotes[currentCRABNote].frequency)
            tap::buzzer::playNote(&drivers->pwm, crabRaveNotes[currentCRABNote].frequency);
        lastCRABFreq = crabRaveNotes[currentCRABNote].frequency;
        currentCRABNote++;
        isSongDone = currentCRABNote == CRAB_NOTE_COUNT;
    }
}

}  // namespace utils::Music