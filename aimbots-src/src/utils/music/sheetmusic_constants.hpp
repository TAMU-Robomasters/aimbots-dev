#pragma once

#include "jukebox_player.hpp"

namespace utils::Jukebox {
// IMPORTANT: After creating a song here,
// add a pointer to it at the top of jukebox_player.cpp

// Default, don't do anything music
Song NothingIsPlayingSong = {120, 2, {PAUSE, PAUSE}};

// We Are Number One
static constexpr uint32_t WeNum1_BPM = 168;
static constexpr uint32_t WeNum1_NOTE_COUNT = 64;
Song WeAreNumberOneSong = {WeNum1_BPM, WeNum1_NOTE_COUNT, {F5,  F5,  F5,  F5,  F5,  F5,  C6,  C6,  B5,  C6,  B5,  C6,  B5,
                                                           B5,  C6,  C6,  Ab5, Ab5, Ab5, Ab5, F5,  F5,  F5,  F5,  F5,  F5,
                                                           F5,  F5,  Ab4, Ab4, C6,  C6,  Db6, Db6, Db6, Db6, Ab4, Ab4, Ab4,
                                                           Ab4, Db6, Db6, Db6, Db6, Eb6, Eb6, Eb6, Eb6, C6,  C6,  Db6, Db6,
                                                           C6,  C6,  Db6, Db6, C6,  C6,  C6,  C6,  E6,  E6,  E6,  E6}};

MusicNote WeAreNumberOneNotes[] = {F5,  F5,  F5,  F5,  F5,  F5,  C6,  C6,  B5,  C6,  B5,  C6,  B5,  B5,   C6, C6,  Ab5,
                                   Ab5, Ab5, Ab5, F5,  F5,  F5,  F5,  F5,  F5,  F5,  F5,  Ab4, Ab4, C6,   C6, Db6, Db6,
                                   Db6, Db6, Ab4, Ab4, Ab4, Ab4, Db6, Db6, Db6, Db6, Eb6, Eb6, Eb6, Eb6,  C6, C6,  Db6,
                                   Db6, C6,  C6,  Db6, Db6, C6,  C6,  C6,  C6,  E6,  E6,  E6,  E6,  PAUSE};

}  // namespace utils::Jukebox