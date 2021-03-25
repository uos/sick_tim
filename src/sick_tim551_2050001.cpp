/*
 * Copyright (C) 2013, Osnabrück University
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
 *  Created on: 14.11.2013
 *
 *      Author:
 *         Martin Günther <mguenthe@uos.de>
 *
 */

#include <sick_tim/sick_tim_common_usb.h>
#include <sick_tim/sick_tim_common_tcp.h>
#include <sick_tim/sick_tim_common_mockup.h>
#include <sick_tim/sick_tim551_2050001_parser.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sick_driver");

  // check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::string hostname;
  if(node->get_parameter("hostname", hostname))
  {
      useTCP = true;
  }
  std::string port;
  port = node->declare_parameter("port", "2112");

  int timelimit;
  timelimit = node->declare_parameter("timelimit", 5);

  bool subscribe_datagram;
  int device_number;
  timelimit = node->declare_parameter("subscribe_datagram", false);
  device_number = node->declare_parameter("device_number", 0);

  sick_tim::SickTim5512050001Parser* parser = new sick_tim::SickTim5512050001Parser();

  double param;
  if (node->get_parameter("range_min", param))
  {
    parser->set_range_min(param);
  }
  if (node->get_parameter("range_max", param))
  {
    parser->set_range_max(param);
  }
  if (node->get_parameter("time_increment", param))
  {
    parser->set_time_increment(param);
  }

  sick_tim::SickTimCommon* s = NULL;

  int result = sick_tim::ExitError;
  while (rclcpp::ok())
  {
    // Atempt to connect/reconnect
    if (subscribe_datagram)
      s = new sick_tim::SickTimCommonMockup(parser, node);
    else if (useTCP)
      s = new sick_tim::SickTimCommonTcp(hostname, port, timelimit, parser, node);
    else
      s = new sick_tim::SickTimCommonUsb(parser, device_number, node);
    result = s->init();

    while(rclcpp::ok() && (result == sick_tim::ExitSuccess)){
      rclcpp::spin_some(node);
      result = s->loopOnce();
    }

    delete s;

    if (result == sick_tim::ExitFatal)
      return result;

    if (rclcpp::ok() && !subscribe_datagram && !useTCP)
      rclcpp::sleep_for(std::chrono::seconds(1)); // Only attempt USB connections once per second
  }

  delete parser;
  return result;
}
