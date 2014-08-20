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

#ifndef SICK_TIM3XX_COMMON_H_
#define SICK_TIM3XX_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <sick_tim/SickTimConfig.h>
#include <sick_tim/abstract_parser.h>

namespace sick_tim
{

class SickTimCommon
{
public:
  SickTimCommon(AbstractParser* parser);
  virtual ~SickTimCommon();
  virtual int init();
  int loopOnce();
  void check_angle_range(SickTimConfig &conf);
  void update_config(sick_tim::SickTimConfig &new_config, uint32_t level = 0);

  double get_expected_frequency() const { return expectedFrequency_; }

protected:
  virtual int init_device() = 0;
  virtual int init_scanner();
  virtual int stop_scanner();
  virtual int close_device() = 0;

  /// Send a SOPAS command to the device and print out the response to the console.
  /**
   * \param [in] request the command to send.
   * \param [out] reply if not NULL, will be filled with the reply package to the command.
   */
  virtual int sendSOPASCommand(const char* request, std::vector<unsigned char> * reply) = 0;

  /// Read a datagram from the device.
  /**
   * \param [in] receiveBuffer data buffer to fill
   * \param [in] bufferSize max data size to write to buffer (result should be 0 terminated)
   * \param [out] actual_length the actual amount of data written
   */
  virtual int get_datagram(unsigned char* receiveBuffer, int bufferSize, int* actual_length) = 0;

protected:
  diagnostic_updater::Updater diagnostics_;

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher datagram_pub_;
  bool publish_datagram_;

  // Diagnostics
  diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan>* diagnosticPub_;
  double expectedFrequency_;

  // Dynamic Reconfigure
  SickTimConfig config_;
  dynamic_reconfigure::Server<sick_tim::SickTimConfig> dynamic_reconfigure_server_;

  // Parser
  AbstractParser* parser_;
};

} /* namespace sick_tim */
#endif /* SICK_TIM3XX_COMMON_H_ */
