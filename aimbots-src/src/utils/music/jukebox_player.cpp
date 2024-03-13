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
    &LegendOfZeldaSong};

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

void JukeboxPlayer::playMusic() {
    if (isCurrentSongDone() || isPaused()) return;

    currentTime = tap::arch::clock::getTimeMilliseconds();
    uint32_t timeSinceLast = currentTime - prevTime;

    Song* currentSong = songsList[currentSongTitle];

    uint32_t Song_MS_PER_16th = (uint32_t)(((1.0f / currentSong->Song_BPM) * 60.0f * 1000.0f) / 4.0f);

    MusicNote currentNote = static_cast<MusicNote>(currentSong->SongNotes[currNoteIndex]);

    if (timeSinceLast >= Song_MS_PER_16th) {
        // Done playing, don't continue any further
        if (currentNote == MusicNote::END) {
            tap::buzzer::playNote(&drivers->pwm, PAUSE);
            currentSongTitle = NONE;
            return;
        }

        prevTime = tap::arch::clock::getTimeMilliseconds();
        if (prevNote != currentNote) tap::buzzer::playNote(&drivers->pwm, currentNote);
        prevNote = currentNote;
        currNoteIndex++;
    }
}

void JukeboxPlayer::stopCurrentSong() {
    tap::buzzer::playNote(&drivers->pwm, PAUSE);
    currentSongTitle = NONE;
}

}  // namespace utils::Jukebox