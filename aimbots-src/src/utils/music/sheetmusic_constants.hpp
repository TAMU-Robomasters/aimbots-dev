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
 * 1) Song Title: Go to tools/common_types.hpp, and find the SongTitle enum.
 *    Add a new entry at the bottom of the list with a short title for your song.
 * 2) Song Pointer: Go to the top of jukebox_player.cpp, and add a reference to your song in the songsList array,
 *    This should match the order that your song title is in the enum as well. (ie &insertNewSongHere)
 */

// Default, don't do anything music
Song nothingIsPlayingSong = {120, Q_N, {{REST, Q_N}, {REST, Q_N}, {END, Q_N}}};

// PacMan Theme

static constexpr uint32_t PM_BPM = 120;
Song pacManSong = {PM_BPM, Q_N, {{B4_N, S_N},  {B5_N, S_N}, {Gb5_N, S_N}, {Eb5_N, S_N}, {B5_N, S_N}, {REST, S_N},
                                 {Eb5_N, S_N}, {REST, S_N}, {C5_N, S_N},  {C6_N, S_N},  {G5_N, S_N}, {E5_N, S_N},
                                 {C6_N, S_N},  {REST, S_N}, {E5_N, S_N},  {REST, S_N},

                                 {B4_N, S_N},  {B5_N, S_N}, {G5_N, S_N},  {E5_N, S_N},  {B5_N, S_N}, {REST, S_N},
                                 {Eb5_N, S_N}, {REST, S_N}, {Eb5_N, E_N}, {F5_N, E_N},  {G5_N, E_N}, {B5_N, S_N},
                                 {END, S_N}}};

// Based on:
// https://musescore.com/user/85429/scores/107109
// Measure 1 - 2

// We Are Number One

static constexpr uint32_t WENUM1_BPM = 168;
Song weAreNumberOneSong = {
    WENUM1_BPM,
    Q_N,
    {{F5_N, QH_N}, {C6_N, E_N},  {B5_N, S_N},  {C6_N, S_N},  {B5_N, S_N},  {C6_N, S_N}, {B5_N, E_N}, {C6_N, E_N},

     {Ab5_N, Q_N}, {F5_N, Q_N},  {F5_N, E_N},  {F5_N, E_N},  {Ab4_N, E_N}, {C6_N, E_N},

     {Db6_N, Q_N}, {Ab4_N, Q_N}, {Db6_N, Q_N}, {Eb6_N, Q_N},

     {C6_N, E_N},  {Db6_N, E_N}, {C6_N, E_N},  {Db6_N, E_N}, {C6_N, Q_N},  {E6_N, Q_N},

     {END, Q_N}}};

// Based on:
// https://musescore.com/user/14081071/scores/3082501
// Measures 1 - 4

// Chainsaw Man Theme

static constexpr uint32_t CHNSW_BPM = 175;
Song chainSawManSong = {CHNSW_BPM, Q_N, {{Ab5_N, Q_N}, {Db5_N, Q_N},  {Eb5_N, Q_N}, {E5_N, E_N},  {Gb5_N, Q_N}, {A4_N, E_N},

                                         {A4_N, E_N},  {E5_N, QH_N},  {A4_N, Q_N},  {A4_N, Q_N},

                                         {Ab4_N, Q_N}, {Eb5_N, Q_N},  {Ab4_N, E_N}, {Gb4_N, Q_N}, {Ab4_N, E_N},

                                         {Ab4_N, E_N}, {Db5_N, QH_N}, {REST, Q_N},  {Ab4_N, Q_N},

                                         {Gb4_N, E_N}, {E4_N, E_N},   {Gb4_N, Q_N}, {Ab4_N, E_N}, {Ab4_N, E_N},

                                         {Bb4_N, E_N}, {B4_N, Q_N},   {A4_N, E_N},  {Ab4_N, E_N}, {Gb4_N, Q_N}, {Db5_N, Q_N},

                                         {Db5_N, Q_N}, {B4_N, E_N},   {A4_N, E_N},  {Ab4_N, E_N}, {E5_N, Q_N},  {Eb5_N, E_N},

                                         {Eb5_N, H_N}, {REST, Q_N},   {Ab5_N, Q_N}, {Db5_N, H_N},

                                         {END, Q_N}}};

// Based on:
// https://musescore.com/user/48165141/scores/8812008
// Measures 42 - 49.5

// Mystery Song (hmm...)

static constexpr uint32_t MYST_BPM = 114;
Song mysterySong = {MYST_BPM, Q_N, {{G4_N, S_N},  {A4_N, S_N}, {C5_N, S_N}, {A4_N, S_N},

                                    {E5_N, EH_N}, {E5_N, S_N}, {E5_N, E_N}, {D5_N, E_N}, {D5_N, Q_N}, {G4_N, S_N},
                                    {A4_N, S_N},  {C5_N, S_N}, {A4_N, S_N},

                                    {D5_N, EH_N}, {D5_N, S_N}, {D5_N, E_N}, {C5_N, E_N}, {C5_N, S_N}, {B4_N, S_N},
                                    {A4_N, E_N},  {G4_N, S_N}, {A4_N, S_N}, {C5_N, S_N}, {A4_N, S_N},

                                    {C5_N, Q_N},  {D5_N, E_N}, {B4_N, E_N}, {B4_N, S_N}, {A4_N, S_N}, {G4_N, Q_N},
                                    {G4_N, E_N},

                                    {D5_N, Q_N},  {C5_N, H_N},

                                    {END, Q_N}}};

// Based on:
// https://musescore.com/punctuationless/never-gonna-give-you-up
// Measures 18.5 - 22.5

// Crab Rave

static constexpr uint32_t CRAB_BPM = 125;
Song crabRaveSong = {CRAB_BPM, Q_N, {{D5_N, E_N}, {Bb5_N, E_N}, {G6_N, E_N}, {G6_N, S_N}, {D5_N, E_N},
                                     {D6_N, S_N}, {A5_N, E_N},  {F5_N, E_N}, {F5_N, E_N},

                                     {D5_N, E_N}, {D6_N, S_N},  {A5_N, E_N}, {F5_N, E_N}, {F5_N, S_N},
                                     {C5_N, E_N}, {C5_N, E_N},  {E5_N, E_N}, {E5_N, S_N}, {F5_N, S_N},

                                     {END, Q_N}}};

// Based on:
// https://musescore.com/user/8140911/scores/5345775
// Measures 19 - 20

// Legend of Zelda Overworld Theme

static constexpr uint32_t LOZ_BPM = 130;
Song legendOfZeldaSong = {LOZ_BPM, Q_N, {{A5_N, H_N},  {REST, TQ_N}, {A5_N, TE_N}, {A5_N, TE_N}, {A5_N, TE_N}, {A5_N, TE_N},

                                         {A5_N, TQ_N}, {G4_N, TE_N}, {A5_N, Q_N},  {REST, TQ_N}, {A5_N, TE_N}, {A5_N, TE_N},
                                         {A5_N, TE_N}, {A5_N, TE_N},

                                         {A5_N, TQ_N}, {G4_N, TE_N}, {A5_N, Q_N},  {REST, TQ_N}, {A5_N, TE_N}, {A5_N, TE_N},
                                         {A5_N, TE_N}, {A5_N, TE_N},

                                         {A5_N, E_N},  {E5_N, S_N},  {E5_N, S_N},  {E5_N, E_N},  {E5_N, S_N},  {E5_N, S_N},
                                         {E5_N, E_N},  {E5_N, S_N},  {E5_N, S_N},  {E5_N, E_N},  {E5_N, E_N},

                                         {END, Q_N}}};

// Based on:
// https://musescore.com/user/20360426/scores/4880846
// Measures 1 - 4

// LG Washing Machine

static constexpr uint32_t LG_BPM = 600;
Song LG_WashSong = {LG_BPM, Q_N, {{Db6_N, QH_N}, {Gb6_N, E_N},  {F6_N, E_N},  {Eb6_N, E_N},

                                  {Db6_N, QH_N}, {Bb5_N, QH_N},

                                  {Cb6_N, E_N},  {Db6_N, E_N},  {Eb6_N, E_N}, {Ab5_N, E_N}, {Bb5_N, E_N}, {Cb6_N, E_N},

                                  {Bb5_N, QH_N}, {Db6_N, QH_N},

                                  {Db6_N, QH_N}, {Gb6_N, E_N},  {F6_N, E_N},  {Eb6_N, E_N},

                                  {Db6_N, QH_N}, {Gb6_N, QH_N},

                                  {Gb6_N, E_N},  {Ab6_N, E_N},  {Gb6_N, E_N}, {F6_N, E_N},  {Eb6_N, E_N}, {F6_N, E_N},

                                  {Gb6_N, H_N},

                                  {END, Q_N}}};

// Based on:
// https://musescore.com/user/35424120/scores/6208111
// Measures 1 - 8

static constexpr uint32_t DomSongBPM = 600;
Song DomSong = {
    DomSongBPM,
    Q_N,
    {{REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {C3_N, QH_N}, {E4_N, QH_N}, {G4_N, QH_N}, {C3_N, QH_N}, {A3_N, QH_N},
     {G3_N, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {C3_N, QH_N}, {E4_N, QH_N}, {D5_N, QH_N}, {G4_N, QH_N}, {G3_N, QH_N}, {D5_N, QH_N}, {G4_N, QH_N},
     {D5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N}, {G4_N, QH_N},
     {D5_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {C5_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N},
     {C5_N, QH_N}, {C4_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {C5_N, QH_N}, {A3_N, QH_N}, {E3_N, QH_N},
     {G4_N, QH_N}, {B4_N, QH_N}, {E5_N, QH_N}, {D5_N, QH_N}, {E3_N, QH_N}, {D5_N, QH_N}, {E5_N, QH_N}, {D5_N, QH_N},
     {G4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {F4_N, QH_N}, {B4_N, QH_N}, {C5_N, QH_N},
     {G4_N, QH_N}, {B4_N, QH_N}, {F4_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {D5_N, QH_N}, {C5_N, QH_N},
     {G4_N, QH_N}, {C5_N, QH_N}, {A4_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {A5_N, QH_N}, {G3_N, QH_N}, {D4_N, QH_N},
     {F4_N, QH_N}, {A5_N, QH_N}, {A4_N, QH_N}, {D5_N, QH_N}, {D4_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {G3_N, QH_N},
     {D5_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N},
     {A4_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {C5_N, QH_N}, {D5_N, QH_N}, {G4_N, QH_N},
     {D5_N, QH_N}, {G3_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {A4_N, QH_N}, {C5_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N},
     {REST, QH_N}, {REST, QH_N}, {A3_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N}, {A3_N, QH_N},
     {D3_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {B4_N, QH_N}, {G4_N, QH_N}, {E3_N, QH_N}, {G4_N, QH_N},
     {F4_N, QH_N}, {B4_N, QH_N}, {F4_N, QH_N}, {B4_N, QH_N}, {A4_N, QH_N}, {B4_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N},
     {B4_N, QH_N}, {F4_N, QH_N}, {B4_N, QH_N}, {A4_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {D3_N, QH_N}, {D5_N, QH_N},
     {E3_N, QH_N}, {G4_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {A4_N, QH_N}, {D5_N, QH_N}, {D3_N, QH_N},
     {A4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {E3_N, QH_N},
     {D3_N, QH_N}, {E3_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N},
     {D3_N, QH_N}, {F4_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N}, {D3_N, QH_N}, {G4_N, QH_N}, {E3_N, QH_N}, {G5_N, QH_N},
     {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {A5_N, QH_N}, {A4_N, QH_N}, {A5_N, QH_N}, {B4_N, QH_N}, {A5_N, QH_N},
     {A4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {D6_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N}, {A3_N, QH_N}, {E3_N, QH_N},
     {D3_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {D3_N, QH_N}, {G4_N, QH_N}, {D3_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N},
     {F4_N, QH_N}, {C5_N, QH_N}, {C3_N, QH_N}, {A3_N, QH_N}, {C5_N, QH_N}, {C6_N, QH_N}, {F4_N, QH_N}, {D5_N, QH_N},
     {D3_N, QH_N}, {D5_N, QH_N}, {D3_N, QH_N}, {E3_N, QH_N}, {G4_N, QH_N}, {C3_N, QH_N}, {E3_N, QH_N}, {A3_N, QH_N},
     {A4_N, QH_N}, {D3_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N},
     {D5_N, QH_N}, {E3_N, QH_N}, {D3_N, QH_N}, {E3_N, QH_N}, {D3_N, QH_N}, {E3_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N},
     {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {F4_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N},
     {D3_N, QH_N}, {F4_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {G5_N, QH_N}, {D6_N, QH_N},
     {G5_N, QH_N}, {D6_N, QH_N}, {A5_N, QH_N}, {G4_N, QH_N}, {F4_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N}, {G4_N, QH_N},
     {E3_N, QH_N}, {B4_N, QH_N}, {A4_N, QH_N}, {B4_N, QH_N}, {C5_N, QH_N}, {D5_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N},
     {A5_N, QH_N}, {A4_N, QH_N}, {A5_N, QH_N}, {F4_N, QH_N}, {C3_N, QH_N}, {A4_N, QH_N}, {A3_N, QH_N}, {E4_N, QH_N},
     {D4_N, QH_N}, {B4_N, QH_N}, {G4_N, QH_N}, {E3_N, QH_N}, {B4_N, QH_N}, {F4_N, QH_N}, {B4_N, QH_N}, {D3_N, QH_N},
     {B4_N, QH_N}, {D3_N, QH_N}, {F3_N, QH_N}, {B4_N, QH_N}, {D3_N, QH_N}, {A4_N, QH_N}, {B4_N, QH_N}, {C3_N, QH_N},
     {B4_N, QH_N}, {A4_N, QH_N}, {D5_N, QH_N}, {D4_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {F4_N, QH_N},
     {G5_N, QH_N}, {G4_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {G5_N, QH_N}, {G4_N, QH_N}, {F4_N, QH_N},
     {G4_N, QH_N}, {D5_N, QH_N}, {D3_N, QH_N}, {F4_N, QH_N}, {G4_N, QH_N}, {F4_N, QH_N}, {G4_N, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {D5_N, QH_N}, {REST, QH_N},
     {D5_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {A5_N, QH_N},
     {A4_N, QH_N}, {F4_N, QH_N}, {G4_N, QH_N}, {REST, QH_N}, {F4_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N},
     {G5_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {C5_N, QH_N}, {A4_N, QH_N}, {C5_N, QH_N},
     {A4_N, QH_N}, {F4_N, QH_N}, {REST, QH_N}, {REST, QH_N}, {C5_N, QH_N}, {A4_N, QH_N}, {F4_N, QH_N}, {A4_N, QH_N},
     {REST, QH_N}, {REST, QH_N}, {G3_N, QH_N}, {G5_N, QH_N}, {B4_N, QH_N}, {G4_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N},
     {E3_N, QH_N}, {D5_N, QH_N}, {E5_N, QH_N}, {D5_N, QH_N}, {G4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N},
     {B4_N, QH_N}, {A4_N, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {B5_N, QH_N}, {E4_N, QH_N},
     {F4_N, QH_N}, {B4_N, QH_N}, {A4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {D5_N, QH_N}, {F4_N, QH_N}, {C5_N, QH_N},
     {G4_N, QH_N}, {C5_N, QH_N}, {B4_N, QH_N}, {D5_N, QH_N}, {A5_N, QH_N}, {G4_N, QH_N}, {A5_N, QH_N}, {G3_N, QH_N},
     {A3_N, QH_N}, {D4_N, QH_N}, {A4_N, QH_N}, {D4_N, QH_N}, {F4_N, QH_N}, {A5_N, QH_N}, {A4_N, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {F4_N, QH_N}, {D4_N, QH_N}, {G4_N, QH_N}, {REST, QH_N}, {D5_N, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N}, {REST, QH_N},
     {REST, QH_N}, {REST, QH_N}, {END, Q_N}}};

}  // namespace utils::Jukebox