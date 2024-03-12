#include "jukebox_player.hpp"

#include "sheetmusic_constants.hpp"

namespace utils::Jukebox {

static constexpr MusicNote songNotesList[] = {{0, &NothingNote}}

JukeboxPlayer::JukeboxPlayer(src::Drivers * drivers)
    : drivers(drivers) {}

/**
 * @brief Checks to see if another song is currently playing
 * If there is none, return true and update the internal value to start it
 */
bool JukeboxPlayer::requestSong(SongTitle name) {
    if (isCurrentSongDone()) {
        currNoteIndex = 0;
        currentSong = name;
        return true;
    }
    return false;
}

void JukeboxPlayer::playMusic() {
    if (isCurrentSongDone()) return;

    currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - prevTime;

    uint32_t Song_MS_PER_16th = (uint32_t)(((1.0f / 110) * 60.0f * 1000.0f) / 4.0f);

    if (timeSinceLast >= Song_MS_PER_16th) {
        prevTime = tap::arch::clock::getTimeMilliseconds();
        if (prevNote != WeAreNumberOneNotes[currNoteIndex])
            tap::buzzer::playNote(&drivers->pwm, WeAreNumberOneNotes[currNoteIndex]);
        prevNote = WeAreNumberOneNotes[currNoteIndex];
        currNoteIndex++;
        // isSongDone = currentWeNum1Note == WeNum1_NOTE_COUNT;
    }
}

}  // namespace utils::Jukebox