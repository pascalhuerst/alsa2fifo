/*
 * Copyright (C) 2018 Pascal Huerst <pascal.huerst@gmail.com>
 * Inspired by work of Henry Hoegelow
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
#include <asoundlib.h>
#include <iostream>

using Sample = int16_t;
using SamplePosition = uint64_t;

struct SampleFrame
{
    Sample left;
    Sample right;
};

struct AlsaSpecs {
    unsigned int rate;
    unsigned int channels;
    snd_pcm_format_t format;
    std::string device;
};
