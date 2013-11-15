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

#include <sick_tim3xx/sick_tim3xx_common.h>

namespace sick_tim3xx
{

SickTim3xxCommon::SickTim3xxCommon(AbstractParser* parser) :
    parser_(parser)
{
  dynamic_reconfigure::Server<sick_tim3xx::SickTim3xxConfig>::CallbackType f;
  f = boost::bind(&sick_tim3xx::SickTim3xxCommon::update_config, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(f);

  // datagram publisher (only for debug)
  ros::NodeHandle pn("~");
  pn.param<bool>("publish_datagram", publish_datagram_, false);
  if (publish_datagram_)
    datagram_pub_ = nh_.advertise<std_msgs::String>("datagram", 1000);

  // scan publisher
  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);
}

int SickTim3xxCommon::stop_scanner()
{
  /*
   * Stop streaming measurements
   */
  const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
  int result = sendSOPASCommand(requestScanData0);
  if (result != 0)
    // use printf because we cannot use ROS_ERROR from the destructor
    printf("\nSOPAS - Error stopping streaming scan data!\n");
  else
    printf("\nSOPAS - Stopped streaming scan data.\n");

  return result;
}

SickTim3xxCommon::~SickTim3xxCommon()
{
  delete parser_;

  printf("sick_tim3xx driver exiting.\n");
}


int SickTim3xxCommon::init()
{
  int result = init_device();
  if(result != 0) {
      ROS_FATAL("Failed to init device: %d", result);
      return result;
  }
  result = init_scanner();
  if(result != 0) {
      ROS_FATAL("Failed to init scanner: %d", result);
  }
  return result;
}

int SickTim3xxCommon::init_scanner()
{
  /*
   * Read the SOPAS variable 'DeviceIdent' by index.
   */
  const char requestDeviceIdent[] = "\x02sRI0\x03\0";
  int result = sendSOPASCommand(requestDeviceIdent);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'DeviceIdent'.");
  }

  /*
   * Read the SOPAS variable 'SerialNumber' by name.
   */
  const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
  result = sendSOPASCommand(requestSerialNumber);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'SerialNumber'.");
  }

  /*
   * Read the SOPAS variable 'FirmwareVersion' by name.
   */
  const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
  result = sendSOPASCommand(requestFirmwareVersion);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'FirmwareVersion'.");
  }

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(requestScanData);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error starting to stream 'LMDscandata'.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int SickTim3xxCommon::loopOnce()
{
  unsigned char receiveBuffer[65536];
  int actual_length = 0;
  static unsigned int iteration_count = 0;

  int result = get_datagram(receiveBuffer, 65536, &actual_length);
  if (result != 0)
  {
      ROS_ERROR("Read Error when getting datagram: %i.", result);
      return EXIT_FAILURE; // return failure to exit node
  }
  if(actual_length <= 0)
      return EXIT_SUCCESS; // return success to continue looping

  // ----- if requested, skip frames
  if (iteration_count++ % (config_.skip + 1) != 0)
    return EXIT_SUCCESS;

  if (publish_datagram_)
  {
    std_msgs::String datagram_msg;
    datagram_msg.data = std::string(reinterpret_cast<char*>(receiveBuffer));
    datagram_pub_.publish(datagram_msg);
  }

  sensor_msgs::LaserScan msg;
  int success = parser_->parse_datagram((char*)receiveBuffer, (size_t)actual_length, config_, msg);
  if (success == EXIT_SUCCESS)
    pub_.publish(msg);

  return EXIT_SUCCESS; // return success to continue looping
}

void SickTim3xxCommon::check_angle_range(SickTim3xxConfig &conf)
{
  if (conf.min_ang > conf.max_ang)
  {
    ROS_WARN("Minimum angle must be greater than maximum angle. Adjusting min_ang.");
    conf.min_ang = conf.max_ang;
  }
}

void SickTim3xxCommon::update_config(sick_tim3xx::SickTim3xxConfig &new_config, uint32_t level)
{
  check_angle_range(new_config);
  config_ = new_config;
}

} /* namespace sick_tim3xx */
