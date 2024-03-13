#pragma once

#include "jukebox_player.hpp"

namespace utils::Jukebox {
// IMPORTANT: The SongTitle Enum must first have a new title added to it in common_types.hpp

// IMPORTANT: After creating a song here,
// add a pointer to it at the top of jukebox_player.cpp

// IMPORTANT: Every song must end with the END note,
// else undefined behavior will happen

// Default, don't do anything music
Song NothingIsPlayingSong = {120, {PAUSE, PAUSE, END}};

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

//Chainsaw Man Theme

static constexpr uint32_t CHNSW_BPM = 350 / 2;  // I think this fixes the fact that we have a hardcoded time signature(?)
Song ChainSawManSong[] = {CHNSW_BPM, {Ab5,   Ab5,   Db5,   Db5, Eb5, Eb5, E5,  Gb5, Gb5, A4,

                                      A4,    E5,    E5,    E5,  A4,  A4,  A4,  A4,  Ab4, Ab4,

                                      Eb5,   Eb5,   Ab4,   Gb4, Gb4, Ab4, Ab4, Db5, Db5, Db5,

                                      PAUSE, PAUSE, Ab4,   Ab4, Gb4, E4,  Gb4, Gb4, Ab4, Ab4,

                                      Bb4,   B4,    B4,    A4,  Ab4, Gb4, Gb4, Db5, Db5, Db5,

                                      Db5,   B4,    A4,    Ab4, E5,  E5,  Eb5, Eb5, Eb5, Eb5,

                                      Eb5,   PAUSE, PAUSE, Ab5, Ab5, Db5, Db5, Db5, Db5,

                                      END}};

// Mystery Song (hmm...)

static constexpr uint32_t MYST_BPM = 114;
Song MysterySong[] = {MYST_BPM, {G4, A4, C5, A4, E5, E5, E5, E5, E6, E6, D5,

                                 D5, D5, D5, G4, A4, C5, A4, D6, D6, D6, D5,

                                 D5, D5, C5, C5, C5, B4, A4, A4, G4, A4, C5,

                                 A4, C5, C5, C5, C5, D5, D5, B4, B4, B4, A4,

                                 G4, G4, G4, G4, G5, G5, D5, D5, D5, D5, C5,

                                 C5, C5, C5, C5, C5, C5, C5,

                                 END}};

// Crab Rave

static constexpr uint32_t CRAB_BPM = 125;
Song CrabRaveSong[] = {CRAB_BPM, {D5, D5, Bb5, Bb5, G6, G6, G6,

                                  D5, D5, D6,  A5,  A5, F5, F5,

                                  F5, F5, D5,  D5,  D6, A5, A5,

                                  F5, F5, F5,  C5,  C5, C5, C5,

                                  E5, E5, E5,  F5,

                                  END}};

//Legend of Zelda Overworld Theme

static constexpr LOZ_BPM =
    130 / 2;  // Halving since it's a lot of eighth-notes and we're hard-coded to fourth notes currently
Song LegendOfZeldaSong = {LOZ_BPM, {B5, B5,    PAUSE, PAUSE, PAUSE, PAUSE, B5, B5, B5, B5,

                                    B5, PAUSE, Ab5,   B5,    PAUSE, PAUSE, B5, B5, B5, B5,

                                    A5, PAUSE, Bb5,   A5,    PAUSE, PAUSE, A5, A5, A5, A5,

                                    B5, F4,    F4,    F4,    F4,    F4,    F4, F4, F4, F4, F4,

                                    END}};

}  // namespace utils::Jukebox