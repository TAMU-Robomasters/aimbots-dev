#pragma once

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "utils/tools/common_types.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace utils::Jukebox {

static constexpr float W_N = 1.0f;           // Whole Note
static constexpr float H_N = 1.0f / 2.0f;    // Half Note
static constexpr float QH_N = 3.0f / 8.0f;   // Quarter and a Half Note
static constexpr float Q_N = 1.0f / 4.0f;    // Quarter Note
static constexpr float TQ_N = 1.0f / 6.0f;   // Triplet Quarter Note
static constexpr float EH_N = 3.0f / 16.0f;  // Eighth and a Half Note
static constexpr float E_N = 1.0f / 8.0f;    // Eighth Note
static constexpr float TE_N = 1.0f / 12.0f;  // Triplet Eighth Note
static constexpr float S_N = 1.0f / 16.0f;   // Sixteenth Note

enum NoteFreq : uint32_t {
    REST = 0,
    END = UINT32_MAX,
    C0_N = 16,         // C0 (16.35 Hz)
    Db0_N = 13,         // Db0 (13.75 Hz)
    D0_N = 14,          // D0 (14.57 Hz)
    Eb0_N = 15,         // Eb0 (15.75 Hz)
    E0_N = 16,          // E0 (16.35 Hz)
    F0_N = 17,          // F0 (17.32 Hz)
    Gb0_N = 19,         // Gb0 (19.00 Hz)
    G0_N = 20,          // G0 (20.60 Hz)
    Ab0_N = 23,         // Ab0 (23.12 Hz)
    A0_N = 24,          // A0 (24.50 Hz)
    Bb0_N = 28,         // Bb0 (27.50 Hz)
    B0_N = 30,          // B0 (30.87 Hz)
    C1_N = 32,          // C1 (32.70 Hz)
    Db1_N = 34,         // Db1 (34.65 Hz)
    D1_N = 36,          // D1 (36.71 Hz)
    Eb1_N = 38,         // Eb1 (38.89 Hz)
    E1_N = 41,          // E1 (41.20 Hz)
    F1_N = 43,          // F1 (43.65 Hz)
    Gb1_N = 46,         // Gb1 (46.25 Hz)
    G1_N = 49,          // G1 (49.00 Hz)
    Ab1_N = 51,         // Ab1 (51.91 Hz)
    A1_N = 55,          // A1 (55.00 Hz)
    Bb1_N = 58,         // Bb1 (58.27 Hz)
    B1_N = 61,          // B1 (61.74 Hz)
    C2_N = 65,          // C2 (65.41 Hz)
    A3_N = 220,         // A3 (220 Hz)
    B3_N = 247,         // B3 (246.94 Hz)
    C3_N = 131,         // C3 (130.81 Hz)
    D3_N = 147,         // D3 (146.83 Hz)
    E3_N = 165,         // E3 (164.81 Hz)
    F3_N = 175,         // F3 (174.61 Hz)
    G3_N = 196,         // G3 (196 Hz)
    C4_N = 262,         // C4 (261.63 Hz)
    D4_N = 294,         // D4 (293.66 Hz)
    E4_N = 330,         // E4 (329.63 Hz)
    F4_N = 340,         // F4 (349.23 Hz)
    Gb4_N = 370,        // Gb4 (392.00 Hz)
    G4_N = 392,         // G4 (392.00 Hz)
    Ab4_N = 415,        // Ab4 (415.30 Hz)
    A4_N = 440,         // A4 (440.00 Hz)
    Bb4_N = 469,        // Bb4 (466.16 Hz)
    B4_N = 494,         // B4 (493.88 Hz)
    C5_N = 523,         // C5 (523.25 Hz)
    Db5_N = 554,        // Db5 (554.37 Hz)
    D5_N = 588,         // D5 (587.33 Hz)
    Eb5_N = 622,        // Eb5 (622.25 Hz)
    E5_N = 659,         // E5 (659.25 Hz)
    F5_N = 699,         // F5 (698.46 Hz)
    Gb5_N = 740,        // Gb5 (739.99 Hz)
    G5_N = 784,         // G5 (783.99 Hz)
    Ab5_N = 830,        // Ab5 (830.61 Hz)
    A5_N = 880,         // A5 (880.00 Hz)
    Bb5_N = 932,        // Bb5 (932.33 Hz)
    B5_N = 988,         // B5 (987.77 Hz)
    Cb6_N = 988,        // Cb6 (987.77 Hz)
    C6_N = 1047,        // C6 (1046.50 Hz)
    Db6_N = 1108,       // Db6 (1108.73 Hz)
    D6_N = 1174,        // D6 (1174.66 Hz)
    Eb6_N = 1244,       // Eb6 (1244.51 Hz)
    E6_N = 1318,        // E6 (1318.51 Hz)
    F6_N = 1396,        // F6 (1396.91 Hz)
    Gb6_N = 1492,       // Gb6 (1491.91 Hz)
    G6_N = 1567,        // G6 (1567.98 Hz)
    Ab6_N = 1661,       // Ab6 (1661.2 Hz)
    A6_N = 1760,        // A6 (1760.00 Hz)
    Bb6_N = 1864,       // Bb6 (1864.66 Hz)
    D7_N = 2349         // D7 (2349.32 Hz)

};

struct MusicNote {
    NoteFreq frequency;
    float noteTiming;
};

struct Song {
    uint32_t songBPM;
    float noteTypePerBeat;
    MusicNote songNotes[];
};

class JukeboxPlayer {
public:
    JukeboxPlayer(src::Drivers* drivers);
    JukeboxPlayer(src::Drivers* drivers, SongTitle initSong);
    ~JukeboxPlayer() = default;

    bool requestSong(SongTitle name);

    void playMusic();

    SongTitle getCurrentSongTitle() { return currentSongTitle; }

    bool isPaused() { return isCurrSongPaused; }
    void pause() { isCurrSongPaused = true; }
    void unpause() { isCurrSongPaused = false; }

    void stopCurrentSong();

    bool isCurrentSongDone() { return currentSongTitle == NONE; }

private:
    src::Drivers* drivers;

    uint32_t prevTime = 0;
    MusicNote prevNote = {REST, Q_N};
    uint32_t currNoteIndex = 0;
    uint32_t currentTime = 0;

    bool isCurrSongPaused = false;

    bool holdNote = false;

    SongTitle currentSongTitle;
};
}  // namespace utils::Jukebox