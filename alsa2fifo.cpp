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

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <asoundlib.h>
#include <stdint.h>

#include "types.h"
#include "AlsaAudioInput.h"
#include "publishZeroConf/publishmDNS.h"
#include "readerwriterqueue.h"

using namespace moodycamel;
namespace po = boost::program_options;

static const std::string strOptDaemon = "daemon";
static const std::string strOptPidFile = "pidfile";
static const std::string strOptDevice = "device";
static const std::string strOptFifo = "fifo";
static const std::string strOptPcmDir = "pcmfile";
static const std::string strOptRate = "rate";
static const std::string strOptChannels = "channels";
static const std::string strOptFormat = "format";
static const std::string strOptDetectorTime = "detector-time";

static const size_t streamBufferSize = 4096 ;

int main(int argc, char **argv)
{
    po::options_description desc("Available options");

    std::string optPidFile = "";
    std::string optDevice = "";
    std::string optFifo = "";
    std::string optPcmDir = "";
    unsigned int optRate = 48000;
    unsigned int optChannels = 2;
    snd_pcm_format_t optFormat = SND_PCM_FORMAT_S16;
    double optDetectorTime = 1.0;

    try {
        desc.add_options()
                ("help,h", "produce help message")
                (strOptDaemon.c_str(), "Run in background as a daemon")
                (strOptPidFile.c_str(), po::value<std::string>(), "Path to pid file")

                (strOptDevice.c_str(), po::value<std::string>(), "Alsa device to read from")
                (strOptRate.c_str(), po::value<unsigned int>(), "Sample rate")
                (strOptChannels.c_str(), po::value<unsigned int>(), "Channels")
                (strOptFormat.c_str(), po::value<std::string>(), "Sample format")

                (strOptFifo.c_str(), po::value<std::string>(), "Fifo to write to")
                (strOptPcmDir.c_str(), po::value<std::string>(), "Directory to store raw pcm data to")
                (strOptDetectorTime.c_str(), po::value<double>(), "Time window to detect silence or signal");

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
            std::cout << desc << "\n";
            return EXIT_SUCCESS;
        }

        bool optIsDaemon = vm.count(strOptDaemon) == true;

        if (vm.count(strOptPidFile) && optIsDaemon) {
            optPidFile = vm[strOptPidFile].as<std::string>();
            std::ofstream f(optPidFile);
            if (!f.good())
                throw std::invalid_argument(strOptPidFile + " is not writable or does not exist.");
        }

        if (vm.count(strOptDevice)) {
            optDevice = vm[strOptDevice].as<std::string>();
        }

        if (vm.count(strOptRate)) {
            optRate = vm[strOptRate].as<unsigned int>();
        }

        if (vm.count(strOptChannels)) {
            optChannels = vm[strOptChannels].as<unsigned int>();
        }

        if (vm.count(strOptFormat)) {
            auto fstring = vm[strOptFormat].as<std::string>();

            if (fstring != "S16_LE")
                throw std::invalid_argument(strOptFormat + ": Only supported format right now is S16_LE.");

            optFormat = SND_PCM_FORMAT_S16;
        }

        if (vm.count(strOptFifo)) {
            optFifo = vm[strOptFifo].as<std::string>();

            struct stat st;
            int fd = open(optFifo.c_str(), O_RDWR);
            if (fd < 0)
                throw std::invalid_argument(strOptFifo + ": " + strerror(errno));

            if (fstat(fd, &st) < 0)
                throw std::invalid_argument(strOptFifo + ": " + strerror(errno));

            if (!S_ISFIFO(st.st_mode))
                throw std::invalid_argument(optFifo + " is not a named pipe.");
        }

        if (vm.count(optPcmDir)) {
            optPcmDir = vm[strOptPcmDir].as<std::string>();

            struct stat st;
            int fd = open(optPcmDir.c_str(), O_RDWR);
            if (fd < 0)
                throw std::invalid_argument(optPcmDir + " -> " + strerror(errno));

            if (fstat(fd, &st) < 0)
                throw std::invalid_argument(optPcmDir + " -> " + strerror(errno));

            if (!S_ISDIR(fd))
                throw std::invalid_argument(optPcmDir + " is not a directory.");
        }

        if (vm.count(strOptDetectorTime)) {
            optDetectorTime = vm[strOptDetectorTime].as<double>();
        }

        std::cout << "optPidFile      = " << optPidFile << "\n"
                  << "optDevice       = " << optDevice << "\n"
                  << "optFifo         = " << optFifo << "\n"
                  << "ptPcmDir        = " << optPcmDir << "\n"
                  << "optRate         = " << optRate << "\n"
                  << "optChannels     = " << optChannels << "\n"
                  << "optDetectorTime = " << optDetectorTime << std::endl << std::endl;

    } catch (std::invalid_argument &e) {

        std::cerr << e.what() << std::endl;
        return -EINVAL;

    } catch (std::exception &e) {

        std::cerr << desc << "\n";
        std::cerr << e.what() << std::endl;
        return -EINVAL;
    }

    BlockingReaderWriterQueue<SampleFrame> streamBuffer(streamBufferSize);

    const double analyzerWindowTime = 0.5; // seconds
    const size_t analyzerBufferSize = static_cast<size_t>(static_cast<double>(optRate) * analyzerWindowTime);
    //BlockingCircularBuffer<SampleFrame> analyzerBuffer(analyzerBufferSize+1);
    BlockingReaderWriterQueue<SampleFrame> analyzerBuffer(analyzerBufferSize);

    std::cout << "Analyzer Size = " << analyzerBufferSize << std::endl;

    // Get Set Size
    //fcntl() F_GETPIPE_SZ F_SETPIPE_SZ

    // unread bytes
    //ioctl(fd, FIONREAD, &nbytes);

    // Reads from analyzerBuffer and processes
    new std::thread([&] {

        PublishZeroConf publishZeroConfg("domestic-recorder");
        SampleFrame buffer[analyzerBufferSize];

        enum AnalyzerState {
            STATE_SILENT,
            STATE_SIGNAL
        };

        AnalyzerState currentState = STATE_SILENT;
        double silenceThresholdPercent = 5; //Detect silence below 5% rms
        int rmsCounter = 0;
        int rmsCounterThreshold = 10;

        while (true) {

            for (size_t i=0; i<analyzerBufferSize; ++i)
                analyzerBuffer.wait_dequeue(buffer[i]);

            long chunkSum = 0;
            for (unsigned int i=0; i<analyzerBufferSize; ++i) {

                int monoSum = buffer[i].left + buffer[i].right;
                chunkSum += (monoSum * monoSum);
            }

            double chunkSumMean = static_cast<double>(chunkSum) / static_cast<double>(analyzerBufferSize);
            double rms = sqrt(chunkSumMean);
            double rmsPercent = 100.0 * rms / static_cast<double>(std::numeric_limits<Sample>::max());

            std::cout << "rms: " << rmsPercent << "%" << std::endl;

            if (rmsPercent < silenceThresholdPercent) {
                if (rmsCounter > 0) {
                    rmsCounter--;
                    std::cout << "rmsCounter = " << rmsCounter << std::endl;
                    if (rmsCounter == 0 && currentState == STATE_SIGNAL) {
                        currentState = STATE_SILENT;
                        std::cout << "stateChanged: STATE_SILENT" << std::endl;
                    }
                }
            } else { // rmsPercent >= silenceThresholdPercent
                if (rmsCounter <= rmsCounterThreshold) {
                    rmsCounter++;
                    std::cout << "rmsCounter = " << rmsCounter << std::endl;
                    if (rmsCounter == rmsCounterThreshold && currentState == STATE_SILENT) {
                        currentState = STATE_SIGNAL;
                        std::cout << "stateChanged: STATE_SIGNAL" << std::endl;
                        //publishZeroConfg.publish({mDNSService("_domestic-recorder._tcp", 4482)});
                    }
                }
            }
        }
    });


    // Reads from StreamBuffer and Writes into FiFo
    new std::thread([&] {

        const size_t chunkSize = 1024;
        SampleFrame buffer[chunkSize];

        int fifoFd = ::open(optFifo.c_str(), O_WRONLY);
        if (fifoFd < 0) {
            std::cerr << "Can not open fifo: (" << optFifo << ") - " << strerror(errno) << std::endl;
            exit(EXIT_FAILURE);
        }

        while (true) {

            //streamBuffer.get(buffer, chunkSize);
            for (size_t i=0; i<chunkSize; ++i)
                streamBuffer.wait_dequeue(buffer[i]);

            size_t byteSize = chunkSize * sizeof(SampleFrame);
            size_t bytesWritten = 0;

            uint8_t *byteBuffer = reinterpret_cast<uint8_t*>(buffer);

            while (bytesWritten < byteSize) {

                ssize_t ret = write(fifoFd, &byteBuffer[bytesWritten], byteSize - bytesWritten);
                if (ret < 0) {
                    std::cerr << "Error writing to fifo: " << strerror(errno) << std::endl;
                    exit(EXIT_FAILURE);
                }

                bytesWritten += static_cast<size_t>(ret);
            }
        }
    });

    AlsaSpecs specs;
    specs.rate = optRate;
    specs.device = optDevice;
    specs.format = optFormat;
    specs.channels = optChannels;


    AlsaAudioInput *ai = new AlsaAudioInput(specs, [&streamBuffer, &analyzerBuffer](SampleFrame *frames, size_t numFrames) {
        for (size_t i=0; i<numFrames; ++i) {
            streamBuffer.enqueue(frames[i]);
            analyzerBuffer.enqueue(frames[i]);
        }
    });

    std::cout << "Starting capture..\n\n";

    while(1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        //std::cout << "\033[F";
        std::cout << ".";
    }


    return EXIT_SUCCESS;
}
