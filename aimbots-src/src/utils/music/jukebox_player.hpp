#pragma once

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "utils/common_types.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace utils::Jukebox {

enum NoteType : uint8_t {
    W_N = 1,   // Whole Note
    H_N = 2,   // Half Note
    Q_N = 4,   // Quarter Note
    TQ_N = 6,  // Triplet Quarter Note
    E_N = 8,   // Eighth Note
    TE_N = 12  // Triplet Eighth Note
};

enum NoteFreq : uint32_t {
    REST = 0,
    END = UINT32_MAX,
    E4 = 330,
    F4 = 340,
    Gb4 = 370,
    G4 = 392,
    Ab4 = 415,
    A4 = 440,
    Bb4 = 469,
    B4 = 494,
    C5 = 523,
    Db5 = 554,
    D5 = 588,
    Eb5 = 622,
    E5 = 659,
    F5 = 699,
    Gb5 = 740,
    G5 = 784,
    Ab5 = 830,
    A5 = 880,
    Bb5 = 932,
    B5 = 988,
    Cb6 = 987,
    C6 = 1047,
    Db6 = 1108,
    D6 = 1174,
    Eb6 = 1244,
    E6 = 1318,
    F6 = 1396,
    Gb6 = 1479,
    G6 = 1567,
    Ab6 = 1661,
    A6 = 1760,
    Bb6 = 1864,
    D7 = 2349,
};

struct MusicNote {
    NoteFreq frequency;
    NoteType type;
};

struct Song {
    uint32_t Song_BPM;
    uint8_t Beats_Per_Measure;
    uint8_t Notes_Per_Beat;
    MusicNote SongNotes[];
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

    SongTitle currentSongTitle;
};
}  // namespace utils::Jukebox