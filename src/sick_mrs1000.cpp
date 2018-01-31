/*
 * Copyright (C) 2017, Osnabrück University
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
 *      Author:
 *         Sebastian Pütz <spuetz@uos.de>
 *
 */

#include <sick_tim/sick_mrs1000_communication.h>
#include <sick_tim/sick_mrs1000_parser.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_mrs_1000");
  ros::NodeHandle nhPriv("~");

  // check for TCP - use if ~hostname is set.
  bool useTCP = false;
  std::string hostname;
  if(!nhPriv.getParam("hostname", hostname))
  {
    ROS_FATAL_STREAM("No hostname given!");
    return sick_tim::ExitError;
  }
  std::string port;
  nhPriv.param<std::string>("port", port, "2112");

  sick_tim::SickMRS1000Parser* parser = new sick_tim::SickMRS1000Parser();

  double param;
  if (nhPriv.getParam("range_min", param))
  {
    parser->set_range_min(param);
  }
  if (nhPriv.getParam("range_max", param))
  {
    parser->set_range_max(param);
  }
  if (nhPriv.getParam("time_increment", param))
  {
    parser->set_time_increment(param);
  }

  int timelimit;
  nhPriv.param("timelimit", timelimit, 5);

  sick_tim::SickTimCommon* s = NULL;

  int result = sick_tim::ExitError;
  while (ros::ok())
  {
    // Atempt to connect/reconnect
    s = new sick_tim::SickMrs1000Communication(hostname, port, timelimit, parser);
    result = s->init();

    while(ros::ok() && (result == sick_tim::ExitSuccess)){
      ros::spinOnce();
      result = s->loopOnce();
    }

    delete s;

    if (result == sick_tim::ExitFatal)
      return result;
  }

  delete parser;
  return result;
}
