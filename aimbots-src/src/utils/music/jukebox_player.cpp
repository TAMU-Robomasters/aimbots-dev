#include "jukebox_player.hpp"

#include "sheetmusic_constants.hpp"

namespace utils::Jukebox {

static constexpr Song* songsList[] = {&WeAreNumberOneSong};

JukeboxPlayer::JukeboxPlayer(src::Drivers* drivers) : drivers(drivers) {}

/**
 * @brief Checks to see if another song is currently playing
 * If there is none, return true and update the internal value to start it
 */
bool JukeboxPlayer::requestSong(SongTitle name) {
    if (iscurrentSongDone()) {
        currNoteIndex = 0;
        currentSongTitle = name;
        return true;
    }
    return false;
}

void JukeboxPlayer::playMusic() {
    if (iscurrentSongDone() || isPaused()) return;

    currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - prevTime;

    Song* currentSong = songsList[currentSongTitle];

    uint32_t songNoteCount = currentSong->NoteCount;

    uint32_t Song_MS_PER_16th = (uint32_t)(((1.0f / currentSong->Song_BPM) * 60.0f * 1000.0f) / 4.0f);

    if (timeSinceLast >= Song_MS_PER_16th) {
        // Done playing, don't continue any further
        if (currNoteIndex >= songNoteCount) {
            tap::buzzer::playNote(&drivers->pwm, PAUSE);
            currentSongTitle = None;
            return;
        }

        prevTime = tap::arch::clock::getTimeMilliseconds();
        if (prevNote != currentSong->SongNotes[currNoteIndex])
            tap::buzzer::playNote(&drivers->pwm, currentSong->SongNotes[currNoteIndex]);
        prevNote = currentSong->SongNotes[currNoteIndex];
        currNoteIndex++;
        // isSongDone = currentWeNum1Note == WeNum1_NOTE_COUNT;
    }
}

}  // namespace utils::Jukebox