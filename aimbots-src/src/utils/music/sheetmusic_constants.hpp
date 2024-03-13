#pragma once

#include "jukebox_player.hpp"

namespace utils::Jukebox {
// IMPORTANT: The SongTitle Enum must first have a new title added to it in common_types.hpp

// IMPORTANT: After creating a song here,
// add a pointer to it at the top of jukebox_player.cpp

// IMPORTANT: Every song must end with the END note,
// else undefined behavior will happen

// Default, don't do anything music
Song NothingIsPlayingSong = {120, {PAUSE, PAUSE}};

// PacMan Theme

static constexpr uint32_t PM_BPM = 120;
Song PacManSong = {PM_BPM, {B4,  B5,    Gb5, Eb5,

                            B5,  PAUSE, Eb5, PAUSE,

                            C5,  C6,    G5,  E5,

                            C6,  PAUSE, E5,  PAUSE,

                            B4,  B5,    G5,  E5,

                            B5,  PAUSE, Eb5, PAUSE,

                            Eb5, Eb5,   F5,  F5,

                            G5,  G5,    B5,  PAUSE,

                            END}};

// We Are Number One
// MusicNote WeAreNumberOneNotes[] = {F5,  F5,  F5,  F5,  F5,  F5,  C6,  C6,  B5,  C6,  B5,  C6,  B5,  B5,  C6, C6,  Ab5,
//                                    Ab5, Ab5, Ab5, F5,  F5,  F5,  F5,  F5,  F5,  F5,  F5,  Ab4, Ab4, C6,  C6, Db6, Db6,
//                                    Db6, Db6, Ab4, Ab4, Ab4, Ab4, Db6, Db6, Db6, Db6, Eb6, Eb6, Eb6, Eb6, C6, C6,  Db6,
//                                    Db6, C6,  C6,  Db6, Db6, C6,  C6,  C6,  C6,  E6,  E6,  E6,  E6,  END};

static constexpr uint32_t WeNum1_BPM = 168;
Song WeAreNumberOneSong = {WeNum1_BPM, {F5,  F5,  F5,  F5,  F5,  F5,  C6,  C6,

                                        B5,  C6,  B5,  C6,  B5,  B5,  C6,  C6,

                                        Ab5, Ab5, Ab5, Ab5, F5,  F5,  F5,  F5,

                                        F5,  F5,  F5,  F5,  Ab4, Ab4, C6,  C6,

                                        Db6, Db6, Db6, Db6, Ab4, Ab4, Ab4, Ab4,

                                        Db6, Db6, Db6, Db6, Eb6, Eb6, Eb6, Eb6,

                                        C6,  C6,  Db6, Db6, C6,  C6,  Db6, Db6,

                                        C6,  C6,  C6,  C6,  E6,  E6,  E6,  E6,

                                        END}};

}  // namespace utils::Jukebox