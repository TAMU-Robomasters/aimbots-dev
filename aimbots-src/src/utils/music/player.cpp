#include "player.hpp"

#include <cstdint>

#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "drivers.hpp"

namespace utils::Music {

// built around A = 440 Hz
static constexpr uint32_t NOTE_C0 = 16;     // C0 (16.35 Hz)
static constexpr uint32_t NOTE_Db0 = 13;     // Db0 (13.75 Hz)
static constexpr uint32_t NOTE_D0 = 14;      // D0 (14.57 Hz)
static constexpr uint32_t NOTE_Eb0 = 15;     // Eb0 (15.75 Hz)
static constexpr uint32_t NOTE_E0 = 16;      // E0 (16.35 Hz)
static constexpr uint32_t NOTE_F0 = 17;      // F0 (17.32 Hz)
static constexpr uint32_t NOTE_Gb0 = 19;     // Gb0 (19.00 Hz)
static constexpr uint32_t NOTE_G0 = 20;      // G0 (20.60 Hz)
static constexpr uint32_t NOTE_Ab0 = 23;     // Ab0 (23.12 Hz)
static constexpr uint32_t NOTE_A0 = 24;      // A0 (24.50 Hz)
static constexpr uint32_t NOTE_Bb0 = 28;     // Bb0 (27.50 Hz)
static constexpr uint32_t NOTE_B0 = 30;      // B0 (30.87 Hz)
static constexpr uint32_t NOTE_C1 = 32;      // C1 (32.70 Hz)
static constexpr uint32_t NOTE_Db1 = 34;     // Db1 (34.65 Hz)
static constexpr uint32_t NOTE_D1 = 36;      // D1 (36.71 Hz)
static constexpr uint32_t NOTE_Eb1 = 38;     // Eb1 (38.89 Hz)
static constexpr uint32_t NOTE_E1 = 41;      // E1 (41.20 Hz)
static constexpr uint32_t NOTE_F1 = 43;      // F1 (43.65 Hz)
static constexpr uint32_t NOTE_Gb1 = 46;     // Gb1 (46.25 Hz)
static constexpr uint32_t NOTE_G1 = 49;      // G1 (49.00 Hz)
static constexpr uint32_t NOTE_Ab1 = 51;     // Ab1 (51.91 Hz)
static constexpr uint32_t NOTE_A1 = 55;      // A1 (55.00 Hz)
static constexpr uint32_t NOTE_Bb1 = 58;     // Bb1 (58.27 Hz)
static constexpr uint32_t NOTE_B1 = 61;      // B1 (61.74 Hz)
static constexpr uint32_t NOTE_C2 = 65;      // C2 (65.41 Hz)
static constexpr uint32_t NOTE_A3 = 220;     // A3 (220 Hz)
static constexpr uint32_t NOTE_B3 = 247;     // B3 (246.94 Hz)
static constexpr uint32_t NOTE_C3 = 131;     // C3 (130.81 Hz)
static constexpr uint32_t NOTE_D3 = 147;     // D3 (146.83 Hz)
static constexpr uint32_t NOTE_E3 = 165;     // E3 (164.81 Hz)
static constexpr uint32_t NOTE_F3 = 175;     // F3 (174.61 Hz)
static constexpr uint32_t NOTE_G3 = 196;     // G3 (196 Hz)
static constexpr uint32_t NOTE_C4 = 262;     // C4 (261.63 Hz)
static constexpr uint32_t NOTE_D4 = 294;     // D4 (293.66 Hz)
static constexpr uint32_t NOTE_E4 = 330;     // E4 (329.63 Hz)
static constexpr uint32_t NOTE_F4 = 340;     // F4 (349.23 Hz)
static constexpr uint32_t NOTE_Gb4 = 370;    // Gb4 (392.00 Hz)
static constexpr uint32_t NOTE_G4 = 392;     // G4 (392.00 Hz)
static constexpr uint32_t NOTE_Ab4 = 415;    // Ab4 (415.30 Hz)
static constexpr uint32_t NOTE_A4 = 440;     // A4 (440.00 Hz)
static constexpr uint32_t NOTE_Bb4 = 469;    // Bb4 (466.16 Hz)
static constexpr uint32_t NOTE_B4 = 494;     // B4 (493.88 Hz)
static constexpr uint32_t NOTE_C5 = 523;     // C5 (523.25 Hz)
static constexpr uint32_t NOTE_Db5 = 554;    // Db5 (554.37 Hz)
static constexpr uint32_t NOTE_D5 = 588;     // D5 (587.33 Hz)
static constexpr uint32_t NOTE_Eb5 = 622;    // Eb5 (622.25 Hz)
static constexpr uint32_t NOTE_E5 = 659;     // E5 (659.25 Hz)
static constexpr uint32_t NOTE_F5 = 699;     // F5 (698.46 Hz)
static constexpr uint32_t NOTE_Gb5 = 740;    // Gb5 (739.99 Hz)
static constexpr uint32_t NOTE_G5 = 784;     // G5 (783.99 Hz)
static constexpr uint32_t NOTE_Ab5 = 830;    // Ab5 (830.61 Hz)
static constexpr uint32_t NOTE_A5 = 880;     // A5 (880.00 Hz)
static constexpr uint32_t NOTE_Bb5 = 932;    // Bb5 (932.33 Hz)
static constexpr uint32_t NOTE_B5 = 988;     // B5 (987.77 Hz)
static constexpr uint32_t NOTE_C6 = 1047;    // C6 (1046.50 Hz)
static constexpr uint32_t NOTE_Db6 = 1108;   // Db6 (1108.73 Hz)
static constexpr uint32_t NOTE_D6 = 1174;    // D6 (1174.66 Hz)
static constexpr uint32_t NOTE_Eb6 = 1244;   // Eb6 (1244.51 Hz)
static constexpr uint32_t NOTE_E6 = 1318;    // E6 (1318.51 Hz)
static constexpr uint32_t NOTE_F6 = 1396;    // F6 (1396.91 Hz)
static constexpr uint32_t NOTE_Gb6 = 1492;    // F6 (1491.91 Hz)
static constexpr uint32_t NOTE_G6 = 1567;    // G6 (1567.98 Hz)
static constexpr uint32_t NOTE_Ab6 = 1661;   // Ab6 (1661.2 Hz)
static constexpr uint32_t NOTE_A6 = 1760;    // A6 (1760.00 Hz)
static constexpr uint32_t NOTE_Bb6 = 1864;    // Bb6 (1864.66 Hz)
static constexpr uint32_t NOTE_D7 = 2349;    // D7 (2349.32 Hz)


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