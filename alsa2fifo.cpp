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
#include <alsa/asoundlib.h>
#include <stdint.h>

#include "types.h"
#include "params.h"

#include "publishZeroConf/publishmDNS.h"
#include "AudioStreamManager.h"
#include "InputKey.h"
#include "Led.h"

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
                (strOptGenericKillDaemon.c_str(), "Kill a running daemon");

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
                (strOptStreamManagerLocalStoreOutputDir.c_str(), po::value<std::string>(), "Directory to store raw pcm data chunks to")
                (strOptStreamManagerPcmOutChunkSize.c_str(), po::value<unsigned long>(), "Chunk size for raw pcm files")
                (strOptStreamManagerLocalStorePrefix.c_str(), po::value<std::string>(), "Prefix for raw pcm files");

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

/*
        //std::string key = "tpacpi::kbd_backlight";
        std::string key = "input3::capslock";
        auto led = Led::create(key);

        for (auto &l : Led::available()) {
            std::cout << "led:    |" << l << std::endl;
        }

        InputKey keys(vmCombined, 3);
        keys.registerKey(30, [](){ // a
            std::cout << "Press on key=30" << std::endl;

        }, [](std::chrono::milliseconds t){
            std::cout << "Release on key=30 t=" << t.count() << "ms" << std::endl;
        });

        keys.registerKey(31, [](){ // s
            std::cout << "Press on key=31" << std::endl;

        }, [](std::chrono::milliseconds t){
            std::cout << "Release on key=31 t=" << t.count() << "ms" << std::endl;
        });
*/
        AudioStreamManager streamManager(vmCombined, [&](AudioStreamManager::DetectorState s) {

            if (s == AudioStreamManager::STATE_SIGNAL) {
                std::cout << "Signal detected!" << std::endl;
            } else {
                std::cout << "Silence detected!" << std::endl;
            }
        });

        int c =0;
        bool isRunning = false;
        bool ledOn = false;

        std::cout << "To start the shit, press t: " << std::endl;
        while((c = getchar()) != 'q') {

            std::cout << "key=" << c << std::endl;

            if (c == 't') {
                if (!isRunning) {
                    std::cout << "STARTING" << c << std::endl;
                    streamManager.start();
                    isRunning = true;
                } else {
                    std::cout << "STOPPING" << c << std::endl;
                    streamManager.stop();
                    isRunning = false;
                }
                std::cout << "StreamManager is currently " << (isRunning ? "running" : "not running") << " toggle!" << std::endl;

            } else {
                std::cout << "Nice key: " << static_cast<char>(c) << std::endl;
            }

#if 0
            // Toggle Led for fun
            ledOn = !ledOn;
            if (ledOn)
                led->on();
            else
                led->off();
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        streamManager.stop();

    } catch (std::invalid_argument &e) {

        std::cerr << odCombined << "\n";
        std::cerr << e.what() << std::endl;
        return -EINVAL;

    } catch (std::runtime_error &e) {

        std::cerr << e.what() << std::endl;
        return -EIO;


    } catch (std::exception &e) {
        std::cerr << e.what() << std::endl;
        return -EINVAL;
    }

    return EXIT_SUCCESS;
}
