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

#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <limits>
#include <vector>
#include <algorithm>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <asoundlib.h>
#include <stdint.h>

#include "types.h"
#include "params.h"

#include "publishZeroConf/publishmDNS.h"
#include "AudioStreamManager.h"

void usage [[noreturn]] (const po::options_description &od)
{
    std::cout << "Triggered!" << std::endl;
    std::cout << od << std::endl;

    exit(EXIT_FAILURE);
}

int main(int argc, char **argv)
{
    // Is used in catch blocks
    po::options_description odCombined(argv[0]);

    try {
        // ########## General Options ##########
        po::options_description odGeneric("Generic");
        odGeneric.add_options()
                (strOptGenericHelp.c_str(), "Print help message")
                (strOptGenericDaemon.c_str(), "Daemonize after startup")
                (strOptGenericPidFile.c_str(), po::value<std::string>(), "Path to pid file");

        // ########## Audio Options ##########
        po::options_description odAudio("Audio");
        odAudio.add_options()
                (strOptAudioDevice.c_str(), po::value<std::string>()->default_value("pulse"), "Alsa device to read from")
                (strOptAudioRate.c_str(), po::value<unsigned int>()->default_value(48000), "Sample rate")
                (strOptAudioChannels.c_str(), po::value<unsigned int>()->default_value(2), "Channels")
                (strOptAudioLatency.c_str(), po::value<double>()->default_value(100.0), "Input latency in ms")
                (strOptAudioFormat.c_str(), po::value<std::string>()->default_value("S16_LE"), "Sample format");

        // ########## StreamManager Options ##########
        po::options_description odStreamManager("StreamManager");
        odStreamManager.add_options()
                (strOptStreamManagerFifo.c_str(), po::value<std::string>()->default_value("/tmp/stream_pipe"), "Named pipe to write raw samples to")
                (strOptStreamManagerStreamBufferSize.c_str(), po::value<unsigned int>()->default_value(4096), "Ring buffer size in frames for pipe stream")
                (strOptStreamManagerPcmOurDir.c_str(), po::value<std::string>(), "Directory to store raw pcm data chunks to")
                (strOptStreamManagerPcmOutChunkSize.c_str(), po::value<unsigned int>(), "Chunk size for raw pcm files");

        // ########## Detector Options ##########
        po::options_description odDetector("Detector");
        odDetector.add_options()
                (strOptDetectorTotalTime.c_str(), po::value<double>()->default_value(10.0), "Total time a signal needs to rest before a status change occurs")
                (strOptDetectorWindowTime.c_str(), po::value<double>()->default_value(1.0), "Time window used to analize input signal")
                (strOptDetectorThreshold.c_str(), po::value<double>()->default_value(1.0), "Threshold in percent of rms signal per window to detect silence");

        // ########## Combined ##########
        odCombined.add(odGeneric).add(odAudio).add(odStreamManager).add(odDetector);
        po::variables_map vmCombined;
        po::store(po::parse_command_line(argc, argv, odCombined), vmCombined);
        po::notify(vmCombined);

        if (vmCombined.count(strOptGenericHelp)) {
            usage(odCombined);
        }

        AudioStreamManager streamManager(vmCombined, [&](AudioStreamManager::DetectorState s) {

            if (s == AudioStreamManager::STATE_SIGNAL) {
                std::cout << "Signal detected!" << std::endl;
            } else {
                std::cout << "Silence detected!" << std::endl;
            }
        });

        int c =0;
        bool isRunning = false;

        std::cout << "To start the shit, press t: " << std::endl;
        while((c = getchar()) != 'q') {

            if (c == 't') {
                std::cout << "StreamManager is currently " << (isRunning ? "running" : "not running") << " toggle!" << std::endl;
                if (!isRunning) {
                    streamManager.start();
                    isRunning = true;
                } else {
                    streamManager.stop();
                    isRunning = false;
                }
            } else {
                std::cout << "Nice key: " << static_cast<char>(c) << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

    } catch (std::invalid_argument &e) {

        std::cerr << odCombined << "\n";
        std::cerr << e.what() << std::endl;
        return -EIO;

    } catch (std::runtime_error &e) {

        std::cerr << e.what() << std::endl;


    } catch (std::exception &e) {

        std::cerr << odCombined << "\n";
        std::cerr << e.what() << std::endl;
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}
