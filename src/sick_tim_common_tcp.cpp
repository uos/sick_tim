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

#include <sick_tim/sick_tim_common_tcp.h>
#include <boost/asio.hpp>
#include <boost/lambda/lambda.hpp>
#include <algorithm>
#include <iterator>
#include <boost/lexical_cast.hpp>

namespace sick_tim
{

SickTimCommonTcp::SickTimCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, AbstractParser* parser)
:
    SickTimCommon(parser),
    socket_(io_service_),
    deadline_(io_service_),
    hostname_(hostname),
    port_(port),
    timelimit_(timelimit)
{
    // Set up the deadline actor to implement timeouts.
    // Based on blocking TCP example on:
    // http://www.boost.org/doc/libs/1_46_0/doc/html/boost_asio/example/timeouts/blocking_tcp_client.cpp

    deadline_.expires_at(boost::posix_time::pos_infin);
    checkDeadline();
}

SickTimCommonTcp::~SickTimCommonTcp()
{
  stop_scanner();
  close_device();
}

using boost::asio::ip::tcp;
using boost::lambda::var;
using boost::lambda::_1;

int SickTimCommonTcp::init_device()
{
    // Resolve the supplied hostname
    tcp::resolver::iterator iterator;
    try
    {
        tcp::resolver resolver(io_service_);
        tcp::resolver::query query(hostname_, port_);
        iterator = resolver.resolve(query);
    }
    catch (boost::system::system_error &e)
    {
        ROS_FATAL("Could not resolve host: ... (%d)(%s)", e.code().value(), e.code().message().c_str());
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not resolve host.");
        return EXIT_FAILURE;
    }

    // Try to connect to all possible endpoints
    boost::system::error_code ec;
    bool success = false;
    for ( ; iterator != tcp::resolver::iterator(); ++iterator)
    {
        std::string repr = boost::lexical_cast<std::string>(iterator->endpoint());
        socket_.close();

        // Set the time out length
        ROS_INFO("Waiting %i seconds for device to connect.", timelimit_);
        deadline_.expires_from_now(boost::posix_time::seconds(timelimit_));

        ec = boost::asio::error::would_block;
        ROS_DEBUG("Attempting to connect to %s", repr.c_str());
        socket_.async_connect(iterator->endpoint(), boost::lambda::var(ec) = _1);

        // Wait until timeout
        do io_service_.run_one(); while (ec == boost::asio::error::would_block);

        if (!ec && socket_.is_open())
        {
            success = true;
            ROS_INFO("Succesfully connected to %s", repr.c_str());
            break;
        }
        ROS_ERROR("Failed to connect with %s", repr.c_str());
    }

    // Check if connecting succeeded
    if (!success)
    {
        ROS_FATAL("Could not connect to host %s:%s", hostname_.c_str(), port_.c_str());
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Could not connect to host.");
        return EXIT_FAILURE;
    }

    input_buffer_.consume(input_buffer_.size());

    return EXIT_SUCCESS;
}

int SickTimCommonTcp::close_device()
{
    if (socket_.is_open())
    {
        try
        {
            socket_.close();
        }
        catch (boost::system::system_error &e)
        {
            ROS_ERROR("An error occured during closing of the connection: %d:%s", e.code().value(), e.code().message().c_str());
        }
    }
    return 0;
}

void SickTimCommonTcp::checkDeadline()
{
    if (deadline_.expires_at() <= boost::asio::deadline_timer::traits_type::now())
    {
        // The reason the function is called is that the deadline expired. Close
        // the socket to return all IO operations and reset the deadline
        socket_.close();
        deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Nothing bad happened, go back to sleep
    deadline_.async_wait(boost::bind(&SickTimCommonTcp::checkDeadline, this));
}

int SickTimCommonTcp::readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read, bool *exception_occured)
{
    // Set up the deadline to the proper timeout, error and delimiters
    deadline_.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
    const char end_delim = static_cast<char>(0x03);
    ec_ = boost::asio::error::would_block;
    bytes_transfered_ = 0;

    // Read until 0x03 ending indicator
    boost::asio::async_read_until(
        socket_, 
        input_buffer_,
        end_delim,
        boost::bind(
            &SickTimCommonTcp::handleRead,
            this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::bytes_transferred
        )
    );
    do io_service_.run_one(); while (ec_ == boost::asio::error::would_block);

    if (ec_)
    {
        // would_block means the connectio is ok, but nothing came in in time.
        // If any other error code is set, this means something bad happened.
        if (ec_ != boost::asio::error::would_block)
        {
            ROS_ERROR("sendSOPASCommand: failed attempt to read from socket: %d: %s", ec_.value(), ec_.message().c_str());
            diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "sendSOPASCommand: exception during read_until().");
            if (exception_occured != 0)
                *exception_occured = true;
        }

        // For would_block, just return and indicate nothing bad happend
        return EXIT_FAILURE;
    }
    
    // Avoid a buffer overflow by limiting the data we read
    size_t to_read = bytes_transfered_ > buffer_size - 1 ? buffer_size - 1 : bytes_transfered_;
    size_t i = 0;
    std::istream istr(&input_buffer_);
    if (buffer != 0)
    {
        istr.read(buffer, to_read);
        buffer[to_read] = 0;

        // Consume the rest of the message if necessary
        if (to_read < bytes_transfered_)
        {
            ROS_WARN("Dropping %zu bytes to avoid buffer overflow", bytes_transfered_ - to_read);
            input_buffer_.consume(bytes_transfered_ - to_read);
        }
    }
    else
        // No buffer was provided, just drop the data
        input_buffer_.consume(bytes_transfered_);
    
    // Set the return variable to the size of the read message
    if (bytes_read != 0)
        *bytes_read = to_read;

    return EXIT_SUCCESS;
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTimCommonTcp::sendSOPASCommand(const char* request, std::vector<unsigned char> * reply)
{
    if (!socket_.is_open()) {
        ROS_ERROR("sendSOPASCommand: socket not open");
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "sendSOPASCommand: socket not open.");
        return EXIT_FAILURE;
    }

    /*
     * Write a SOPAS variable read request to the device.
     */
    try
    {
        boost::asio::write(socket_, boost::asio::buffer(request, strlen(request)));
    }
    catch (boost::system::system_error &e)
    {
        ROS_ERROR("write error for command: %s", request);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Write error for sendSOPASCommand.");
        return EXIT_FAILURE;
    }

    // Set timeout in 5 seconds
    const int BUF_SIZE = 1000;
    char buffer[BUF_SIZE];
    int bytes_read;
    if (readWithTimeout(1000, buffer, BUF_SIZE, &bytes_read, 0) == EXIT_FAILURE)
    {
        ROS_ERROR_THROTTLE(1.0, "sendSOPASCommand: no full reply available for read after 1s");
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "sendSOPASCommand: no full reply available for read after 5 s.");
        return EXIT_FAILURE;
    }

    if (reply)
    {
        reply->resize(bytes_read);
        std::copy(buffer, buffer + bytes_read, &(*reply)[0]);
    }

    return EXIT_SUCCESS;
}

int SickTimCommonTcp::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
{
    if (!socket_.is_open()) {
        ROS_ERROR("get_datagram: socket not open");
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "get_datagram: socket not open.");
        return EXIT_FAILURE;
    }

    /*
     * Write a SOPAS variable read request to the device.
     */
    std::vector<unsigned char> reply;

    // Wait at most 1000ms for a new scan
    size_t timeout = 1000;
    bool exception_occured = false;

    char *buffer = reinterpret_cast<char *>(receiveBuffer);

    if (readWithTimeout(timeout, buffer, bufferSize, actual_length, &exception_occured) != EXIT_SUCCESS)
    {
        ROS_ERROR_THROTTLE(1.0, "get_datagram: no data available for read after %zu ms", timeout);
        diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "get_datagram: no data available for read after timeout.");

        // Attempt to reconnect when the connection was terminated
        if (!socket_.is_open())
        {
            sleep(1);
            ROS_INFO("Failure - attempting to reconnect");
            int ret = init_device();
            if (ret != EXIT_SUCCESS)
                return ret;

            return init_scanner();
        }

        return exception_occured ? EXIT_FAILURE : EXIT_SUCCESS;    // keep on trying
    }

    return EXIT_SUCCESS;
}

} /* namespace sick_tim */
