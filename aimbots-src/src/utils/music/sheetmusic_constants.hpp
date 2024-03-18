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

                                  {Eb5, E_N}, {F5, E_N},   {G5, E_N},  {B5, S_N},

                                  {END, S_N}}};

// Based on:
// https://musescore.com/user/85429/scores/107109
// Measure 1-2

// We Are Number One

// clang-format off
static constexpr uint32_t WeNum1_BPM = 168;
Song WeAreNumberOneSong = {WeNum1_BPM, 4, 4, {{F5, QH_N}, {C6, E_N}, {B5, S_N}, {C6, S_N}, {B5, S_N}, {C6, S_N}, {B5, E_N}, {C6, E_N},

                                              {Ab5, Q_N}, {F5, Q_N}, {F5, E_N}, {F5, E_N}, {Ab4, E_N}, {C6, E_N}, 
                                              
                                              {Db6, Q_N}, {Ab4, Q_N}, {Db6, Q_N}, {Eb6, Q_N},

                                              {C6, E_N}, {Db6, E_N}, {C6, E_N}, {Db6, E_N}, {C6, Q_N},  {E6, Q_N},

                                              {END, Q_N}}};
// clang-format on

// Based on:
// https://musescore.com/user/14081071/scores/3082501
// Measures 1-4

// Chainsaw Man Theme

static constexpr uint32_t CHNSW_BPM = 175;
Song ChainSawManSong = {CHNSW_BPM, 4, 4, {{Ab5, Q_N}, {Db5, Q_N},  {Eb5, Q_N},  {E5, E_N},  {Gb5, Q_N}, {A4, E_N},

                                          {A4, E_N},  {E5, QH_N},  {A4, Q_N},   {A4, Q_N},

                                          {Ab4, Q_N}, {Eb5, Q_N},  {Ab4, E_N},  {Gb4, Q_N}, {Ab4, E_N},

                                          {Ab4, E_N}, {Db5, QH_N}, {REST, Q_N}, {Ab4, Q_N},

                                          {Gb4, E_N}, {E4, E_N},   {Gb4, Q_N},  {Ab4, E_N}, {Ab4, E_N},

                                          {Bb4, E_N}, {B4, Q_N},   {A4, E_N},   {Ab4, E_N}, {Gb4, Q_N}, {Db5, Q_N},

                                          {Db5, Q_N}, {B4, E_N},   {A4, E_N},   {Ab4, E_N}, {E5, Q_N},  {Eb5, E_N},

                                          {Eb5, H_N}, {REST, Q_N}, {Ab5, Q_N},  {Db5, H_N},

                                          {END, Q_N}}};

// Based on:
// https://musescore.com/user/48165141/scores/8812008
// Measures 42 - 49.5

// Mystery Song (hmm...)

static constexpr uint32_t MYST_BPM = 114;
Song MysterySong = {MYST_BPM, 4, 4, {{G4, S_N},  {A4, S_N}, {C5, S_N}, {A4, S_N},

                                     {E5, EH_N}, {E5, S_N}, {E5, E_N}, {D5, E_N}, {D5, Q_N}, {G4, S_N}, {A4, S_N},
                                     {C5, S_N},  {A4, S_N},

                                     {D5, EH_N}, {D5, S_N}, {D5, E_N}, {C5, E_N}, {C5, S_N}, {B4, S_N}, {A4, E_N},
                                     {G4, S_N},  {A4, S_N}, {C5, S_N}, {A4, S_N},

                                     {C5, Q_N},  {D5, E_N}, {B4, E_N}, {B4, S_N}, {A4, S_N}, {G4, Q_N}, {G4, E_N},

                                     {D5, Q_N},  {C5, H_N},

                                     {END, Q_N}}};

// Based on:
// https://musescore.com/punctuationless/never-gonna-give-you-up
// Measures 18.5 - 22.5

// Crab Rave

static constexpr uint32_t CRAB_BPM = 125;
Song CrabRaveSong = {CRAB_BPM, 4, 4, {D5, D5, Bb5, Bb5, G6, G6, G6,

                                      D5, D5, D6,  A5,  A5, F5, F5,

                                      F5, F5, D5,  D5,  D6, A5, A5,

                                      F5, F5, F5,  C5,  C5, C5, C5,

                                      E5, E5, E5,  F5,

                                      END}};

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

// Based on:
// https://musescore.com/user/20360426/scores/4880846
// Measures 1 - 4

// LG Washing Machine

// time signature 6/8
static constexpr uint32_t LG_BPM = 150;  // 92
Song LG_WashSong = {LG_BPM, 6, 8, {{Db6, QH_N}, Gb6, F6,  Eb6,

                                   Db6,         Db6, Db6, Bb5, Bb5, Bb5,

                                   Cb6,         Db6, Eb6, Ab5, Bb5, Cb6,

                                   Bb5,         Bb5, Bb5, Db6, Db6, Db6,

                                   Db6,         Db6, Db6, Gb6, F6,  Eb6,

                                   Db6,         Db6, Db6, Gb6, Gb6, Gb6,

                                   Gb6,         Ab6, Gb6, F6,  Eb6, F6,

                                   Gb6,         Gb6, Gb6, Gb6, Gb6, Gb6,

                                   END}};

// Based on:
// https://musescore.com/user/35424120/scores/6208111
// Measures 1-8

}  // namespace utils::Jukebox