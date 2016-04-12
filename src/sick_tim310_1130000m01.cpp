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
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *         Martin Günther <mguenthe@uos.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#include <sick_tim/sick_tim_common_usb.h>
#include <sick_tim/sick_tim_common_mockup.h>
#include <sick_tim/sick_tim310_1130000m01_parser.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim310_1130000m01");
  ros::NodeHandle nhPriv("~");

  bool subscribe_datagram;
  nhPriv.param("subscribe_datagram", subscribe_datagram, false);

  sick_tim::SickTim3101130000M01Parser* parser = new sick_tim::SickTim3101130000M01Parser();

  sick_tim::SickTimCommon* s = NULL;

  int result = EXIT_FAILURE;
  while (ros::ok())
  {
    // Atempt to connect/reconnect
    delete s;
    if (subscribe_datagram)
      s = new sick_tim::SickTimCommonMockup(parser);
    else
      s = new sick_tim::SickTimCommonUsb(parser);
    result = s->init();

    while(ros::ok() && (result == EXIT_SUCCESS)){
      ros::spinOnce();
      result = s->loopOnce();
    }

    if (result == EXIT_FATAL)
      return result;

    if (ros::ok() && !subscribe_datagram)
      ros::Duration(1.0).sleep(); // Only attempt USB connections once per second
  }

  delete s;
  delete parser;
  return result;
}
