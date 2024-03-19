#include "jukebox_player.hpp"

#include "drivers.hpp"
#include "sheetmusic_constants.hpp"

namespace utils::Jukebox {

static constexpr Song* songsList[] = {
    &NothingIsPlayingSong,
    &PacManSong,
    &WeAreNumberOneSong,
    &ChainSawManSong,
    &MysterySong,
    &CrabRaveSong,
    &LegendOfZeldaSong,
    &LG_WashSong};

JukeboxPlayer::JukeboxPlayer(src::Drivers* drivers) : drivers(drivers), currentSongTitle(NONE) {}

JukeboxPlayer::JukeboxPlayer(src::Drivers* drivers, SongTitle initSong) : drivers(drivers), currentSongTitle(initSong) {}

/**
 * @brief Checks to see if another song is currently playing
 * If there is none, return true and update the internal value to start it
 */
bool JukeboxPlayer::requestSong(SongTitle name) {
    if (isCurrentSongDone()) {
        unpause();
        currNoteIndex = 0;
        currentSongTitle = name;
        return true;
    }
    return false;
}

uint32_t pauseAtIndexWatch = 1000;  // Change as a Watch Variable

void JukeboxPlayer::playMusic() {
    if (isCurrentSongDone() || isPaused() || currNoteIndex > pauseAtIndexWatch) return;

    currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - prevTime;

    Song* currentSong = songsList[currentSongTitle];

    MusicNote currentNote = static_cast<MusicNote>(currentSong->SongNotes[currNoteIndex]);

    uint32_t MS_HOLD_FOR_NOTE =
        (uint32_t)(((1.0f / currentSong->Song_BPM) * 60.0f * 1000.0f) * (currentNote.noteTiming / currentSong->NoteType_Per_Beat));

    if (holdNote) {
        if (timeSinceLast >= MS_HOLD_FOR_NOTE) {
            currNoteIndex++;
            holdNote = false;
        }
    } else {
        // Done playing, don't continue any further
        if (currentNote.frequency == NoteFreq::END) {
            stopCurrentSong();
            return;
        }

        prevTime = tap::arch::clock::getTimeMilliseconds();
        tap::buzzer::playNote(&drivers->pwm, currentNote.frequency);
        prevNote = currentNote;
        holdNote = true;
    }
}

void JukeboxPlayer::stopCurrentSong() {
    tap::buzzer::playNote(&drivers->pwm, REST);
    prevNote = {REST, Q_N};
    holdNote = false;
    currentSongTitle = NONE;
}

}  // namespace utils::Jukebox