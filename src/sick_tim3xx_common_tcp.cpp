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

#include <sick_tim3xx/sick_tim3xx_common_tcp.h>

namespace sick_tim3xx
{

SickTim3xxCommonTcp::SickTim3xxCommonTcp(AbstractParser* parser) : SickTim3xxCommon(parser)
{
}

SickTim3xxCommonTcp::~SickTim3xxCommonTcp()
{
  stop_scanner();
  close_device();
}

int SickTim3xxCommonTcp::close_device()
{
  int result = 0;
  return result;
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTim3xxCommonTcp::sendSOPASCommand(const char* request)
{
  int result = 0;
  unsigned char receiveBuffer[65536];

  /*
   * Write a SOPAS variable read request to the device.
   */

  return result;
}

int SickTim3xxCommonTcp::init_device()
{

  return EXIT_SUCCESS;
}

int SickTim3xxCommonTcp::get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length)
{

  return EXIT_SUCCESS;
}

} /* namespace sick_tim3xx */
