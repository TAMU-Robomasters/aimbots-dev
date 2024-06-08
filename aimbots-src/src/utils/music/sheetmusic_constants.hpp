#pragma once

#include "jukebox_player.hpp"

namespace utils::Jukebox {
/* How to create a song:
 *
 * A song consists of a BPM, a NoteType for the BPM, and a list of MusicNotes to play in order
 * A MusicNote consists of two things:
 *  1) A NoteFreq, which is an enum defined in jukebox_player.hpp
 *     relating tones to a buzzer frequency (feel free to add new tones if you need them)
 *  2) A NoteType, which is a series of constexpr fractions also defined in jukebox_player.hpp,
 *     deciding how long the note should be held
 *
 * A Song is defined in terms of {BPM, NoteType, {List of MusicNotes, ...,}}
 * NoteType for the BPM can be the same constexpr fractions used to define MusicNotes
 * The List of MusicNotes must end with the END note frequency, else the song will crash (note type does not matter)
 * You can look at the examples of other songs here for more info
 *
 * For formatting reasons, please separate each measure by a completely empty line, to enhance readability
 * Addtionally, after the song, include a link to the sheet music referenced online,
 * and a mention of which measures are recorded in the code
 *
 * After defining the song, there are two more modifications needed to make it useable:
 * 1) Song Title: Go to common_types.hpp, and find the SongTitle enum.
 *    Add a new entry at the bottom of the list with a short title for your song.
 * 2) Song Pointer: Go to the top of jukebox_player.cpp, and add a reference to your song in the songsList array,
 *    This should match the order that your song title is in the enum as well. (ie &insertNewSongHere)
 */

// Default, don't do anything music
Song NothingIsPlayingSong = {120, Q_N, {{REST, Q_N}, {REST, Q_N}, {END, Q_N}}};

// PacMan Theme

static constexpr uint32_t PM_BPM = 120;
Song PacManSong = {PM_BPM, Q_N, {{B4, S_N},   {B5, S_N},   {Gb5, S_N}, {Eb5, S_N}, {B5, S_N}, {REST, S_N}, {Eb5, S_N},
                                 {REST, S_N}, {C5, S_N},   {C6, S_N},  {G5, S_N},  {E5, S_N}, {C6, S_N},   {REST, S_N},
                                 {E5, S_N},   {REST, S_N},

                                 {B4, S_N},   {B5, S_N},   {G5, S_N},  {E5, S_N},  {B5, S_N}, {REST, S_N}, {Eb5, S_N},
                                 {REST, S_N}, {Eb5, E_N},  {F5, E_N},  {G5, E_N},  {B5, S_N}, {END, S_N}}};

// Based on:
// https://musescore.com/user/85429/scores/107109
// Measure 1 - 2

// We Are Number One

static constexpr uint32_t WeNum1_BPM = 168;
Song WeAreNumberOneSong = {WeNum1_BPM, Q_N, {{F5, QH_N}, {C6, E_N},  {B5, S_N},  {C6, S_N},  {B5, S_N},  {C6, S_N},
                                             {B5, E_N},  {C6, E_N},

                                             {Ab5, Q_N}, {F5, Q_N},  {F5, E_N},  {F5, E_N},  {Ab4, E_N}, {C6, E_N},

                                             {Db6, Q_N}, {Ab4, Q_N}, {Db6, Q_N}, {Eb6, Q_N},

                                             {C6, E_N},  {Db6, E_N}, {C6, E_N},  {Db6, E_N}, {C6, Q_N},  {E6, Q_N},

                                             {END, Q_N}}};

// Based on:
// https://musescore.com/user/14081071/scores/3082501
// Measures 1 - 4

// Chainsaw Man Theme

static constexpr uint32_t CHNSW_BPM = 175;
Song ChainSawManSong = {CHNSW_BPM, Q_N, {{Ab5, Q_N}, {Db5, Q_N},  {Eb5, Q_N},  {E5, E_N},  {Gb5, Q_N}, {A4, E_N},

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
Song MysterySong = {MYST_BPM, Q_N, {{G4, S_N},  {A4, S_N}, {C5, S_N}, {A4, S_N},

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
Song CrabRaveSong = {CRAB_BPM, Q_N, {{D5, E_N}, {Bb5, E_N}, {G6, E_N}, {G6, S_N}, {D5, E_N},
                                     {D6, S_N}, {A5, E_N},  {F5, E_N}, {F5, E_N},

                                     {D5, E_N}, {D6, S_N},  {A5, E_N}, {F5, E_N}, {F5, S_N},
                                     {C5, E_N}, {C5, E_N},  {E5, E_N}, {E5, S_N}, {F5, S_N},

                                     {END, Q_N}}};

// Based on:
// https://musescore.com/user/8140911/scores/5345775
// Measures 19 - 20

// Legend of Zelda Overworld Theme

static constexpr uint32_t LOZ_BPM = 130;
Song LegendOfZeldaSong = {LOZ_BPM, Q_N, {{A5, H_N},  {REST, TQ_N}, {A5, TE_N}, {A5, TE_N},   {A5, TE_N}, {A5, TE_N},

                                         {A5, TQ_N}, {G4, TE_N},   {A5, Q_N},  {REST, TQ_N}, {A5, TE_N}, {A5, TE_N},
                                         {A5, TE_N}, {A5, TE_N},

                                         {A5, TQ_N}, {G4, TE_N},   {A5, Q_N},  {REST, TQ_N}, {A5, TE_N}, {A5, TE_N},
                                         {A5, TE_N}, {A5, TE_N},

                                         {A5, E_N},  {E5, S_N},    {E5, S_N},  {E5, E_N},    {E5, S_N},  {E5, S_N},
                                         {E5, E_N},  {E5, S_N},    {E5, S_N},  {E5, E_N},    {E5, E_N},

                                         {END, Q_N}}};

// Based on:
// https://musescore.com/user/20360426/scores/4880846
// Measures 1 - 4

// LG Washing Machine

static constexpr uint32_t LG_BPM = 160;
Song LG_WashSong = {LG_BPM, Q_N, {{Db6, QH_N}, {Gb6, E_N},  {F6, E_N},  {Eb6, E_N},

                                  {Db6, QH_N}, {Bb5, QH_N},

                                  {Cb6, E_N},  {Db6, E_N},  {Eb6, E_N}, {Ab5, E_N}, {Bb5, E_N}, {Cb6, E_N},

                                  {Bb5, QH_N}, {Db6, QH_N},

                                  {Db6, QH_N}, {Gb6, E_N},  {F6, E_N},  {Eb6, E_N},

                                  {Db6, QH_N}, {Gb6, QH_N},

                                  {Gb6, E_N},  {Ab6, E_N},  {Gb6, E_N}, {F6, E_N},  {Eb6, E_N}, {F6, E_N},

                                  {Gb6, H_N},

                                  {END, Q_N}}};

// Based on:
// https://musescore.com/user/35424120/scores/6208111
// Measures 1 - 8

}  // namespace utils::Jukebox