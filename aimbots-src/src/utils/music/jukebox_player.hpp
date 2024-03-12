#pragma once

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "utils/common_types.hpp"

#include "drivers.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace utils::Jukebox {

enum SongTitle { None = 0, PacMan, WeAreNumberOne, ChainSawMan, Mystery, CrabRave, Zelda };

enum MusicNote : uint32_t {
    PAUSE = 0,
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
    C6 = 1047,
    Db6 = 1108,
    D6 = 1174,
    Eb6 = 1244,
    E6 = 1318,
    F6 = 1396,
    G6 = 1567,
    A6 = 1760,
    Bb6 = 1864,
    D7 = 2349,
};

struct Song {
    uint32_t Song_BPM;
    uint32_t NoteCount;
    MusicNote SongNotes[];
};

class JukeboxPlayer {
public:
    JukeboxPlayer(src::Drivers* drivers);
    ~JukeboxPlayer() = default;

    bool requestSong(SongTitle name);

    void playMusic();

    SongTitle getcurrentSongTitle() { return currentSongTitle; }

    bool isPaused() { return isCurrSongPaused; }
    void pause() { isCurrSongPaused = true; }
    void unpause() { isCurrSongPaused = false; }

    void stopcurrentSong() { currentSongTitle = None; }

    bool iscurrentSongDone() { return currentSongTitle == None; }

private:
    src::Drivers* drivers;

    uint32_t prevTime = 0;
    MusicNote prevNote = PAUSE;
    uint32_t currNoteIndex = 0;
    uint32_t currentTime = 0;

    bool isCurrSongPaused = false;

    SongTitle currentSongTitle = None;
};
}  // namespace utils::Jukebox