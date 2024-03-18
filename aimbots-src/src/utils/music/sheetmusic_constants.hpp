#pragma once

#include "jukebox_player.hpp"

namespace utils::Jukebox {
// IMPORTANT: The SongTitle Enum must first have a new title added to it in common_types.hpp

// IMPORTANT: After creating a song here,
// add a pointer to it at the top of jukebox_player.cpp

// IMPORTANT: Every song must end with the END note,
// else undefined behavior will happen

// Default, don't do anything music
Song NothingIsPlayingSong = {120, 4, 4, {{REST, Q_N}, {REST, Q_N}, {END, Q_N}}};

// PacMan Theme

static constexpr uint32_t PM_BPM = 120;
Song PacManSong = {PM_BPM, 4, 4, {{B4, S_N},  {B5, S_N},   {Gb5, S_N}, {Eb5, S_N},

                                  {B5, S_N},  {REST, S_N}, {Eb5, S_N}, {REST, S_N},

                                  {C5, S_N},  {C6, S_N},   {G5, S_N},  {E5, S_N},

                                  {C6, S_N},  {REST, S_N}, {E5, S_N},  {REST, S_N},

                                  {B4, S_N},  {B5, S_N},   {G5, S_N},  {E5, S_N},

                                  {B5, S_N},  {REST, S_N}, {Eb5, S_N}, {REST, S_N},

                                  {Eb5, S_N}, {F5, E_N},   {G5, E_N},  {B5, E_N},

                                  {END, S_N}}};

// We Are Number One

// clang-format off
static constexpr uint32_t WeNum1_BPM = 168;
Song WeAreNumberOneSong = {WeNum1_BPM, 4, 4, {{F5, QH_N}, {C6, E_N}, {B5, S_N}, {C6, S_N}, {B5, S_N}, {C6, S_N}, {B5, E_N}, {C6, E_N},

                                              {Ab5, Q_N}, {F5, Q_N}, {F5, E_N}, {F5, E_N}, {Ab4, E_N}, {C6, E_N}, 
                                              
                                              {Db6, Q_N}, {Ab4, Q_N}, {Db6, Q_N}, {Eb6, Q_N},

                                              {C6, E_N}, {Db6, E_N}, {C6, E_N}, {Db6, E_N}, {C6, Q_N},  {E6, Q_N},

                                              {END, Q_N}}};
// clang-format on

// Chainsaw Man Theme
/*
static constexpr uint32_t CHNSW_BPM = 175;
Song ChainSawManSong = {CHNSW_BPM, 4, 4, {Ab5,   Ab5,   Db5,   Db5, Eb5, Eb5, E5,  Gb5, Gb5, A4,

                                          A4,    E5,    E5,    E5,  A4,  A4,  A4,  A4,  Ab4, Ab4,

                                          Eb5,   Eb5,   Ab4,   Gb4, Gb4, Ab4, Ab4, Db5, Db5, Db5,

                                          REST, REST, Ab4,   Ab4, Gb4, E4,  Gb4, Gb4, Ab4, Ab4,

                                          Bb4,   B4,    B4,    A4,  Ab4, Gb4, Gb4, Db5, Db5, Db5,

                                          Db5,   B4,    A4,    Ab4, E5,  E5,  Eb5, Eb5, Eb5, Eb5,

                                          Eb5,   REST, REST, Ab5, Ab5, Db5, Db5, Db5, Db5,

                                          END}};

// Mystery Song (hmm...)

static constexpr uint32_t MYST_BPM = 114;
Song MysterySong = {MYST_BPM, 4, 4, {G4, A4, C5, A4, E5, E5, E5, E5, E6, E6, D5,

                                     D5, D5, D5, G4, A4, C5, A4, D6, D6, D6, D5,

                                     D5, D5, C5, C5, C5, B4, A4, A4, G4, A4, C5,

                                     A4, C5, C5, C5, C5, D5, D5, B4, B4, B4, A4,

                                     G4, G4, G4, G4, G5, G5, D5, D5, D5, D5, C5,

                                     C5, C5, C5, C5, C5, C5, C5,

                                     END}};

// Crab Rave

static constexpr uint32_t CRAB_BPM = 125;
Song CrabRaveSong = {CRAB_BPM, 4, 4, {D5, D5, Bb5, Bb5, G6, G6, G6,

                                      D5, D5, D6,  A5,  A5, F5, F5,

                                      F5, F5, D5,  D5,  D6, A5, A5,

                                      F5, F5, F5,  C5,  C5, C5, C5,

                                      E5, E5, E5,  F5,

                                      END}};
*/
// Legend of Zelda Overworld Theme
// clang-format off
static constexpr uint32_t LOZ_BPM = 130;

Song LegendOfZeldaSong = {LOZ_BPM, 4, 4, 

{{A5, H_N}, {REST, TQ_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N},

{A5, TQ_N}, {G4, TE_N}, {A5, Q_N}, {REST, TQ_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N},

{A5, TQ_N}, {G4, TE_N}, {A5, Q_N}, {REST, TQ_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N}, {A5, TE_N},

{A5, E_N}, {E5, S_N}, {E5, S_N}, {E5, E_N}, {E5, S_N}, {E5, S_N}, {E5, E_N}, {E5, S_N}, {E5, S_N}, {E5, E_N}, {E5, E_N},

{END, Q_N}}};
// clang-format on

/*
// LG Washing Machine

// time signature 6/8
static constexpr uint32_t LG_BPM = 92;
Song LG_WashSong = {LG_BPM, 6, 8, {Db6, Db6, Db6, Gb6, F6,  Eb6,

                                   Db6, Db6, Db6, Bb5, Bb5, Bb5,

                                   Cb6, Db6, Eb6, Ab5, Bb5, Cb6,

                                   Bb5, Bb5, Bb5, Db6, Db6, Db6,

                                   Db6, Db6, Db6, Gb6, F6,  Eb6,

                                   Db6, Db6, Db6, Gb6, Gb6, Gb6,

                                   Gb6, Ab6, Gb6, F6,  Eb6, F6,

                                   Gb6, Gb6, Gb6, Gb6, Gb6, Gb6,

                                   END}};
*/
}  // namespace utils::Jukebox