#pragma once

#include "record_player.hpp"

namespace utils::Record {

// Default, don't do anything music
Song nothingIsPlayingSong = {{{0, 0.1}, {0, 0.1}, {69420, 0.1}}};

Song DomSongRecord = {{{250, 5}}};

}  // namespace utils::Record