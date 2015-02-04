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
 *  Created on: 21.08.2013
 *
 *      Author: Martin Günther <mguenthe@uos.de>
 *
 */

#include <sick_tim/sick_tim_datagram_test.h>

#include <sick_tim/sick_tim310s01_parser.h>

namespace sick_tim
{

SickTimDatagramTest::SickTimDatagramTest(AbstractParser* parser) :
    parser_(parser)
{
  //dynamic_reconfigure_server_.getConfigDefault(config_);
  dynamic_reconfigure::Server<sick_tim::SickTimConfig>::CallbackType f;
  f = boost::bind(&sick_tim::SickTimDatagramTest::update_config, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_from_datagram", 1000);
  sub_ = nh_.subscribe("datagram", 1, &SickTimDatagramTest::datagramCB, this);
}

SickTimDatagramTest::~SickTimDatagramTest()
{
  delete parser_;
}

void SickTimDatagramTest::datagramCB(const std_msgs::String::ConstPtr &datagram_msg)
{
  sensor_msgs::LaserScan scan_msg;

  std::vector<char> str(datagram_msg->data.begin(), datagram_msg->data.end());
  str.push_back('\0');

  int success = parser_->parse_datagram(&str[0], datagram_msg->data.length(), config_, scan_msg);
  if (success == EXIT_SUCCESS)
    pub_.publish(scan_msg);
  else
    ROS_ERROR("parse_datagram returned %d!", success);
}

void SickTimDatagramTest::check_angle_range(SickTimConfig &conf)
{
  if (conf.min_ang > conf.max_ang)
  {
    ROS_WARN("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
    conf.min_ang = conf.max_ang;
  }
}

void SickTimDatagramTest::update_config(sick_tim::SickTimConfig &new_config, uint32_t level)
{
  check_angle_range(new_config);
  config_ = new_config;
}

} /* namespace sick_tim */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim_datagram_test");

  sick_tim::SickTim310S01Parser* parser = new sick_tim::SickTim310S01Parser();
  sick_tim::SickTimDatagramTest s(parser);

  ros::spin();

  return 0;
}
