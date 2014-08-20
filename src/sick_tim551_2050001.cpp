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

#include <sick_tim3xx/sick_tim3xx_common_usb.h>
#include <sick_tim3xx/sick_tim3xx_common_tcp.h>
#include <sick_tim3xx/sick_tim551_2050001_parser.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim551_2050001");

  // check for TCP - use if ~hostname is set.
  ros::NodeHandle nhPriv("~");
  bool useTCP = false;
  std::string hostname;
  std::string ros_topic;
  if(nhPriv.getParam("hostname", hostname)) {
      useTCP = true;
  }
  nhPriv.param<std::string>("topic", ros_topic, "scan");

  sick_tim3xx::SickTim5512050001Parser* parser = new sick_tim3xx::SickTim5512050001Parser();
  sick_tim3xx::SickTim3xxCommon* s = NULL;
  if(useTCP)
      s = new sick_tim3xx::SickTim3xxCommonTcp(ros_topic, hostname, parser);
  else
      s = new sick_tim3xx::SickTim3xxCommonUsb(ros_topic, parser);

  int result = s->init();
  while (ros::ok() && (result == EXIT_SUCCESS))
  {
    ros::spinOnce();
    result = s->loopOnce();
  }

  delete s;
  return result;
}
