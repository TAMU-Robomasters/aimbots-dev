#include "record_player.hpp"

#include "drivers.hpp"
#include "record_disc_constants.hpp"

namespace utils::Record {

static constexpr Song* songsList[] = {
    &nothingIsPlayingSong,
    &DomSongRecord};

RecordPlayer::RecordPlayer(src::Drivers* driver) : drivers(drivers), currentRecordTitle(NA) {}

RecordPlayer::RecordPlayer(src::Drivers* drivers, RecordTitle initSong) : drivers(drivers), currentRecordTitle(initSong) {}

/**
 * @brief Checks to see if another song is currently playing
 * If there is none, return true and update the internal value to start it
 */

bool RecordPlayer::requestSong(RecordTitle name) {
    if (isCurrentSongDone()) {
        unpause();
        currNoteIndex = 0;
        currentRecordTitle = name;
        return true;
    }
    return false;
}

uint32_t pauseAtIndexWatch = 1000;  // Change as a Watch Variable

void RecordPlayer::playMusic() {
    if (isCurrentSongDone() || isPaused() || currNoteIndex > pauseAtIndexWatch) return;

    currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - prevTime;

    Song* currentSong = songsList[currentRecordTitle];

    MusicNote currentNote = static_cast<MusicNote>(currentSong->songNotes[currNoteIndex]);

    // Need to Fix Probably
    uint32_t MS_HOLD_FOR_NOTE = 0.5;

    if (holdNote) {
        if (timeSinceLast >= MS_HOLD_FOR_NOTE) {
            currNoteIndex++;
            holdNote = false;
        }
    } else {
        // Done playing, don't continue any further
        if (currentNote.frequency == 69420) {
            stopCurrentSong();
            return;
        }

        prevTime = tap::arch::clock::getTimeMilliseconds();
        tap::buzzer::playNote(&drivers->pwm, currentNote.frequency);
        prevNote = currentNote;
        holdNote = true;
    }
}

void RecordPlayer::stopCurrentSong() {
    tap::buzzer::playNote(&drivers->pwm, 0);
    prevNote = {0, 0.1};
    holdNote = false;
    currentRecordTitle = NA;
}

}  // namespace utils::Record