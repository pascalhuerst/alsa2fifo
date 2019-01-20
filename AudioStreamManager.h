/*
 * Copyright (C) 2018 Pascal Huerst <pascal.huerst@gmail.com>
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

#include <atomic>
#include <thread>
#include <memory>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "AlsaAudioInput.h"
#include "readerwriterqueue.h"

using namespace moodycamel;

class AudioStreamManager {

public:
    enum DetectorState {
        STATE_SILENT,
        STATE_SIGNAL
    };

    using Callback = std::function<void(DetectorState s)>;

    AudioStreamManager(const po::variables_map &vmCombined,
                       Callback onDetectorStateChangedCB);

    ~AudioStreamManager();

    void start();
    void stop();

private:
    std::atomic<bool> m_terminateRequest;

    po::variables_map m_vmCombined;

    std::unique_ptr<BlockingReaderWriterQueue<SampleFrame>> m_streamBuffer;
    std::unique_ptr<BlockingReaderWriterQueue<SampleFrame>> m_detectorBuffer;

    std::unique_ptr<AlsaAudioInput> m_alsaAudioInput;
    std::unique_ptr<std::thread> m_streamWorker;
    std::unique_ptr<std::thread> m_detectorWorker;

    size_t m_detectorBufferSize;
    unsigned int m_detectorSuccession;
    double m_detectorThreshold;

    std::string m_streamFifo;
    std::string m_streamPcmOutDir;
    std::string m_streamPcmOutPrefix;
    std::string m_streamPcmOutChunkSize;

    Callback m_detectorStateChangedCB;
};
