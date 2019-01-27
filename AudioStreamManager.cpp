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

#include "AudioStreamManager.h"
#include "params.h"
#include "types.h"

#include <sys/ioctl.h>
#include <signal.h>
#include <stdint.h>
#include <sys/time.h>
#include <poll.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/stat.h>


AudioStreamManager::AudioStreamManager(const po::variables_map &vmCombined,
                                       Callback onDetectorStateChangedCB) :
    m_terminateRequest(true),
    m_vmCombined(vmCombined),
    m_streamBuffer(nullptr),
    m_detectorBuffer(nullptr),
    m_alsaAudioInput(nullptr),
    m_streamWorker(nullptr),
    m_detectorWorker(nullptr),
    m_detectorBufferSize(0),
    m_detectorSuccession(0),
    m_detectorThreshold(0.0),
    m_streamFifo(""),
    m_streamPcmOutDir(""),
    m_streamPcmOutPrefix(""),
    m_streamPcmOutChunkSize(""),
    m_detectorStateChangedCB(onDetectorStateChangedCB)
{
}

AudioStreamManager::~AudioStreamManager()
{
}

void AudioStreamManager::start()
{
    if (m_terminateRequest) {

        m_terminateRequest = false;

        if (m_vmCombined.count(strOptStreamManagerFifo)) {
            m_streamFifo = m_vmCombined[strOptStreamManagerFifo].as<std::string>();

            if (!isValidPath(m_streamFifo)) {
                std::cout << "Fifo: " + m_streamFifo + " does not exist. Creating." << std::endl;
                createFifo(m_streamFifo);
                std::cout << "Pipe created at: " << m_streamFifo << " with size: " << static_cast<float>(fifoSize(File(m_streamFifo.c_str(), O_RDWR)) / 1024.0) << "kb" << std::endl;
            } else {
                if (!isFifo(File(m_streamFifo.c_str(), O_RDWR)))
                    throw std::invalid_argument(m_streamFifo + " is not a fifo.");
            }
        } else {
            throw std::invalid_argument(strOptStreamManagerFifo + " must be set!");
        }
#if 0
        if (m_vmCombined.count(strOptStreamManagerPcmOurDir)) {
            m_streamPcmOutDir = m_vmCombined[strOptStreamManagerPcmOurDir].as<std::string>();

            struct stat st;
            int fd = open(m_streamPcmOutDir.c_str(), O_DIRECTORY);
            if (fd < 0)
                throw std::invalid_argument(m_streamPcmOutDir + ": " + strerror(errno));

            if (fstat(fd, &st) < 0)
                throw std::invalid_argument(m_streamPcmOutDir + ": " + strerror(errno));

            if (!S_ISDIR(fd))
                throw std::invalid_argument(m_streamPcmOutDir + " is not a directory.");
        } else {
            throw std::invalid_argument(strOptStreamManagerPcmOurDir + " must be set!");
        }

        if (m_vmCombined.count(strOptStreamManagerPcmOutPrefix)) {
            m_streamPcmOutPrefix = m_vmCombined[strOptStreamManagerPcmOutPrefix].as<std::string>();
        } else {
            throw std::invalid_argument(strOptStreamManagerPcmOutPrefix + " must be set!");
        }
#endif

        if (m_vmCombined.count(strOptStreamManagerPcmOutChunkSize)) {
            m_streamPcmOutChunkSize = m_vmCombined[strOptStreamManagerPcmOutChunkSize].as<std::string>();
        }

        unsigned int streamBufferSize = 0;
        if (m_vmCombined.count(strOptStreamManagerStreamBufferSize)) {
            streamBufferSize = m_vmCombined[strOptStreamManagerStreamBufferSize].as<unsigned int>();
        } else {
            throw std::invalid_argument(strOptStreamManagerStreamBufferSize + "must be set!");
        }

        m_streamBuffer.reset(new BlockingReaderWriterQueue<SampleFrame>(streamBufferSize));

        double detectorTotalTime = 0.0;
        if (m_vmCombined.count(strOptDetectorTotalTime)) {
            detectorTotalTime = m_vmCombined[strOptDetectorTotalTime].as<double>();
        } else {
            throw std::invalid_argument(strOptDetectorTotalTime + "must be set!");
        }

        double detectorWindowTime = 0.0;
        if (m_vmCombined.count(strOptDetectorWindowTime)) {
            detectorWindowTime = m_vmCombined[strOptDetectorWindowTime].as<double>();
        } else {
            throw std::invalid_argument(strOptDetectorWindowTime + "must be set!");
        }

        if (m_vmCombined.count(strOptDetectorThreshold)) {
            m_detectorThreshold = m_vmCombined[strOptDetectorThreshold].as<double>();
        } else {
            throw std::invalid_argument(strOptDetectorThreshold + "must be set!");
        }

        unsigned int rate = 0;
        if (m_vmCombined.count(strOptAudioRate)) {
            rate = m_vmCombined[strOptAudioRate].as<unsigned int>();
        } else {
            throw std::invalid_argument(strOptAudioRate + "must be set!");
        }

        m_detectorBufferSize = static_cast<size_t>(static_cast<double>(rate) * detectorWindowTime);
        m_detectorSuccession = static_cast<unsigned int>(detectorTotalTime / detectorWindowTime + 0.5);
        m_detectorBuffer.reset(new BlockingReaderWriterQueue<SampleFrame>(m_detectorBufferSize));

        // Instatiation and lambda callback from alsa. Just fill out ringbuffers
        m_alsaAudioInput.reset(new AlsaAudioInput(m_vmCombined, [&] (SampleFrame *frames, size_t numFrames) {
                                   for (size_t i=0; i<numFrames; ++i) {
                                       m_streamBuffer->enqueue(frames[i]);
                                       m_detectorBuffer->enqueue(frames[i]);

                                   }
                               }));

        // Stream lamda in a thread
        m_streamWorker.reset(new std::thread([&] {

            try {
                // Ignore SIGPIPE. Otherwise we terminate if reader disappears. Instead, we want start from the beginning
                ::signal(SIGPIPE, SIG_IGN);

                const size_t chunkSize = 1024;
                SampleFrame buffer[chunkSize];

                int counter = 0;
                while (!fifoHasReader(m_streamFifo) && !m_terminateRequest) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    if ((++counter) % 10 == 0) {
                        std::cout << "Pipe has no reader. (" << (counter>>1) << "s)" << std::endl;
                    }
                }

                // Now we had a reader. Open blocking.
                File fifoFd = File(m_streamFifo, O_WRONLY);

                while (!m_terminateRequest) {

                    size_t i=0;
                    while (i<chunkSize && !m_terminateRequest) {
                        if(!m_streamBuffer->wait_dequeue_timed(buffer[i++], std::chrono::milliseconds(500)))
                            continue;
                    }

                    if (m_terminateRequest)
                        continue;

                    size_t byteSize = chunkSize * sizeof(SampleFrame);
                    size_t bytesWritten = 0;

                    uint8_t *byteBuffer = reinterpret_cast<uint8_t*>(buffer);

                    while (bytesWritten < byteSize && !m_terminateRequest) {

                        struct pollfd pfd = {0, 0, 0};
                        pfd.fd = fifoFd;
                        pfd.events = POLLOUT;

                        int ret = poll(&pfd, 1, 500);

                        if (ret == 0) { //Timeout
                            int availableBytes = fifoBytesAvailable(fifoFd);
                            size_t size = fifoSize(fifoFd);

                            std::cout << "Writing to Fifo timed out:\n";
                            std::cout << "available:  " << availableBytes << "\n";
                            std::cout << "size:       " << size << "\n";
                            std::cout << "Level:      " << (static_cast<float>(availableBytes) / static_cast<float>(availableBytes)) << "\n";

                            continue;

                        } else if (ret < 0) {
                            throw std::runtime_error(std::string("Error in poll(): ") + strerror(errno));

                        } // ret>0 -> Some data can be written

                        ssize_t written = 0;
                        while ((written = ::write(fifoFd, &byteBuffer[bytesWritten], byteSize - bytesWritten)) < 0 && errno == EPIPE && !m_terminateRequest) {
                            std::cout << "Fifo lost reader! write throws SIGPIPE. Waiting 500ms." << std::endl;
                            std::this_thread::sleep_for(std::chrono::milliseconds(500));
                        }

                        if (m_terminateRequest)
                            continue;

                        if (written < 0)
                            throw std::runtime_error(std::string("Error writing to fifo: (") + m_streamFifo + ") - " + strerror(errno));

                        bytesWritten += static_cast<size_t>(written);
                    }
                }
            } catch (std::exception &e) {
                m_terminateRequest = true;
                if (m_detectorWorker->joinable())
                    m_detectorWorker->join();

                throw e;
            }
        }));

        // Detector lambda in a thread
        m_detectorWorker.reset(new std::thread([&] {

            SampleFrame buffer[m_detectorBufferSize];

            DetectorState currentState = STATE_SILENT;
            unsigned int rmsCounter = 0;

            while (!m_terminateRequest) {

                size_t i=0;
                while (i<m_detectorBufferSize && !m_terminateRequest) {
                    auto ret = m_detectorBuffer->wait_dequeue_timed(buffer[i], std::chrono::milliseconds(500));
                    if (m_terminateRequest) return;
                    if (!ret)
                        continue;

                    i++;
                }

                long chunkSum = 0;
                for (unsigned int i=0; i<m_detectorBufferSize; ++i) {

                    int monoSum = buffer[i].left + buffer[i].right;
                    chunkSum += (monoSum * monoSum);
                }

                double chunkSumMean = static_cast<double>(chunkSum) / static_cast<double>(m_detectorBufferSize);
                double rms = sqrt(chunkSumMean);
                double rmsPercent = 100.0 * rms / static_cast<double>(std::numeric_limits<Sample>::max());

                //std::cout << "rms: " << rmsPercent << "%" << std::endl;

                if (rmsPercent < m_detectorThreshold) {
                    if (rmsCounter > 0) {
                        rmsCounter--;
                        if (rmsCounter == 0 && currentState == STATE_SIGNAL) {
                            currentState = STATE_SILENT;
                            if (m_detectorStateChangedCB)
                                m_detectorStateChangedCB(currentState);
                        }
                    }
                } else { // rmsPercent >= silenceThresholdPercent
                    if (rmsCounter <= m_detectorSuccession) {
                        rmsCounter++;
                        if (rmsCounter == m_detectorSuccession && currentState == STATE_SILENT) {
                            currentState = STATE_SIGNAL;
                            if (m_detectorStateChangedCB)
                                m_detectorStateChangedCB(currentState);
                        }
                    }
                }
            }
        }));
    }
}

void AudioStreamManager::stop()
{
    if (!m_terminateRequest) {

        m_terminateRequest = true;

        if (m_detectorWorker->joinable()) {
            m_detectorWorker->join();
            m_detectorWorker.reset();
        }

        if (m_streamWorker->joinable()) {
            m_streamWorker->join();
            m_detectorWorker.reset();
        }

        m_alsaAudioInput.reset();
        m_streamBuffer.reset();
        m_detectorBuffer.reset();
    }
}

size_t AudioStreamManager::fifoSize(int fd)
{
    int ret = fcntl(fd, F_GETPIPE_SZ);
    if (ret < 0)
        throw std::invalid_argument(std::string("Can not get fifo size. ") + m_streamFifo + ": " + strerror(errno));

    return static_cast<size_t>(ret);
}

int AudioStreamManager::fifoBytesAvailable(int fd)
{
    int nbytes = 0;
    int ret = ::ioctl(fd, FIONREAD, &nbytes);
    if (ret < 0)
        throw std::invalid_argument(std::string("Can not determin available bytes to read. ") + m_streamFifo + ": " + strerror(errno));

    return ret;
}

bool AudioStreamManager::isFifo(int fd)
{
    bool ret = true;
    struct stat st;

    if (fstat(fd, &st) < 0)
        throw std::invalid_argument(strOptStreamManagerFifo + ": " + strerror(errno));

    if (!S_ISFIFO(st.st_mode))
        ret = false;

    return ret;
}

bool AudioStreamManager::isValidPath(const std::string &path)
{
    bool ret = true;

    int fd = ::open(path.c_str(), O_RDWR);
    if (fd < 0)
        ret = false;
    else
        close(fd);

    return ret;
}

void AudioStreamManager::createFifo(const std::string &path)
{
    int ret = ::mkfifo(path.c_str(), 0777);
    if (ret < 0)
        throw std::invalid_argument("Can not create fifo " + path + ": " + strerror(errno));
}

size_t AudioStreamManager::setFifoSize(int fd, size_t s)
{
    int ret = fcntl(fd, F_SETPIPE_SZ, s);
    if (ret < 0)
        throw std::invalid_argument(std::string("Can not set fifo size. ") + m_streamFifo + ": " + strerror(errno));

    return fifoSize(fd);
}

bool AudioStreamManager::fifoHasReader(const std::string &path)
{
    int fd = ::open(path.c_str(), O_WRONLY | O_NONBLOCK);
    if (fd < 0)
        return false;

    close(fd);
    return true;
}
