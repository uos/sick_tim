/*
 * Copyright (C) 2013, Freiburg University
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 15.11.2013
 *
 *      Authors:
 *         Christian Dornhege <c.dornhege@googlemail.com>
 */

#include <sys/socket.h>
#include <strings.h>
#include <netdb.h>
#include <sick_tim3xx/sick_tim3xx_common_tcp.h>

namespace sick_tim3xx
{

SickTim3xxCommonTcp::SickTim3xxCommonTcp(const std::string & hostname, AbstractParser* parser) : SickTim3xxCommon(parser),
    socket_fd_(-1), hostname_(hostname)
{
}

SickTim3xxCommonTcp::~SickTim3xxCommonTcp()
{
  stop_scanner();
  close_device();
}

int SickTim3xxCommonTcp::init_device()
{
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_fd_ == -1) {
        ROS_FATAL("Could not open socket: %d", errno);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not open socket.");
        return EXIT_FAILURE;
    }

    struct addrinfo hints;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = 0;
    hints.ai_protocol = 0;

    struct addrinfo* result;
    int res = getaddrinfo(hostname_.c_str(), "2112", &hints, &result);
    if(res != 0) {
        ROS_FATAL("Could not resolve host: ... (%d)", res);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not resolve host.");
        return EXIT_FAILURE;
    }

    // try to connect to any of the returned host infos
    struct addrinfo* cur_addr;
    for(cur_addr = result; cur_addr != NULL; cur_addr = cur_addr->ai_next) {
        if(connect(socket_fd_, cur_addr->ai_addr, cur_addr->ai_addrlen) != -1)
            break;
    }

    freeaddrinfo(result);

    if(cur_addr == NULL) {
        ROS_FATAL("Could not connect to host %s", hostname_.c_str());
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not connect to host.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int SickTim3xxCommonTcp::close_device()
{
    if(socket_fd_ != -1)
        return close(socket_fd_);
    return 0;
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTim3xxCommonTcp::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply)
{
    if(socket_fd_ == -1) {
        ROS_ERROR("sendSOPASCommand: socket_fd_ not open");
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "sendSOPASCommand: socket_fd_ not open.");
        return EXIT_FAILURE;
    }

    /*
     * Write a SOPAS variable read request to the device.
     */
    ssize_t bytesWritten = write(socket_fd_, request, strlen(request));
    if(bytesWritten != (ssize_t)strlen(request)) {
        ROS_ERROR("write error for command: %s", request);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Write error for sendSOPASCommand.");
        return EXIT_FAILURE;
    }

    unsigned char receiveBuffer[65536];
    ssize_t bytesRead = read(socket_fd_, receiveBuffer, 65536 - 1);
    if(bytesRead < 0) {
        // FIXME might need a select
        ROS_ERROR("read error after command: %s (%d)", request, errno);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Read error after sendSOPASCommand.");
        return EXIT_FAILURE;
    }
    receiveBuffer[bytesRead] = 0;
    ROS_DEBUG("TCP - Read data...  %s", receiveBuffer);
    if(reply) {
        reply->clear();
        for(int i = 0; i < bytesRead; i++) {
            reply->push_back(receiveBuffer[i]);
        }
    }

    // FIXME: need to puzzle together messages from stream???
    // see below

    return EXIT_SUCCESS;
}

int SickTim3xxCommonTcp::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
{
    if(socket_fd_ == -1) {
        ROS_ERROR("get_datagram: socket_fd_ not open");
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "get_datagram: socket_fd_ not open.");
        return EXIT_FAILURE;
    }

    /*
     * Write a SOPAS variable read request to the device.
     */
    ssize_t bytesRead = read(socket_fd_, receiveBuffer, bufferSize - 1);
    if(bytesRead < 0) {
        // FIXME might need a select
        ROS_ERROR("get_datagram: Read error"); 
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "get_datagram: Read error.");
        return EXIT_FAILURE;
    }
    receiveBuffer[bytesRead] = 0;
    ROS_DEBUG("TCP - Read data...  %s", receiveBuffer);
    *actual_length = bytesRead + 1; // include \0

    // FIXME Ideally need to puzzle together messages from stream.
    // Currently working on LAN as the scanner sends 1 packet/scan.

    return EXIT_SUCCESS;
}

} /* namespace sick_tim3xx */
