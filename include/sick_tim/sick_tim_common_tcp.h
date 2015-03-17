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

#ifndef SICK_TIM3XX_COMMON_TCP_H
#define SICK_TIM3XX_COMMON_TCP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sick_tim/sick_tim_common.h>
#include <boost/asio.hpp>

namespace sick_tim
{

class SickTimCommonTcp : public SickTimCommon
{
public:
  SickTimCommonTcp(const std::string &hostname, const std::string &port, int &timelimit, AbstractParser* parser);
  virtual ~SickTimCommonTcp();

protected:
  virtual int init_device();
  virtual int close_device();

  /// Send a SOPAS command to the device and print out the response to the console.
  virtual int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply);

  /// Read a datagram from the device.
  /**
   * \param [in] receiveBuffer data buffer to fill
   * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
   * \param [out] actual_length the actual amount of data written
   */
  virtual int get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length);
 
  // Helpers for boost asio
  int readWithTimeout(size_t timeout_ms, char *buffer, int buffer_size, int *bytes_read = 0, bool *exception_occured = 0);
  void handleRead(boost::system::error_code error, size_t bytes_transfered);
  void checkDeadline();

private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::deadline_timer deadline_;
  boost::asio::streambuf input_buffer_;
  boost::system::error_code ec_;
  size_t bytes_transfered_;

  std::string hostname_;
  std::string port_;
  int timelimit_;
};

inline void SickTimCommonTcp::handleRead(boost::system::error_code error, size_t bytes_transfered)
{
    ec_ = error;
    bytes_transfered_ += bytes_transfered;
}

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_COMMON_TCP_H */

