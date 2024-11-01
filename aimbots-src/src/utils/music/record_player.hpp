#pragma once

#include <vector>

#include "tap/communication/sensors/buzzer/buzzer.hpp"

#include "utils/tools/common_types.hpp"

namespace src {
class Drivers;
}  // namespace src

namespace utils::Record {

struct MusicNote {
    float frequency;
    float duration;
};

struct Song {
    std::vector<MusicNote> songNotes;
};

class RecordPlayer {
public:
    RecordPlayer(src::Drivers* drivers);
    RecordPlayer(src::Drivers* drivers, RecordTitle initSong);
    ~RecordPlayer() = default;

    bool requestSong(RecordTitle namme);

    void playMusic();

    RecordTitle getCurrentRecordTitle() { return currentRecordTitle; }

    bool isPaused() { return isCurrSongPaused; }
    void pause() { isCurrSongPaused = true; }
    void unpause() { isCurrSongPaused = false; }

    void stopCurrentSong();

    bool isCurrentSongDone() { return currentRecordTitle == NA; }

private:
    src::Drivers* drivers;

    uint32_t prevTime = 0;
    MusicNote prevNote = {0, 0.1};
    uint32_t currNoteIndex = 0;
    uint32_t currentTime = 0;

    bool isCurrSongPaused = false;

    bool holdNote = false;

    RecordTitle currentRecordTitle;
};

}  // namespace utils::Record