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

#include "sick_tim/sick_mrs1000_communication.h"

namespace sick_tim
{

SickMrs1000Communication::SickMrs1000Communication(const std::string &hostname,
                                                   const std::string &port,
                                                   int &timelimit,
                                                   ScanAndCloudParser* parser)
: SickTimCommonTcp(hostname, port, timelimit, parser), scan_and_cloud_parser_(parser)
{
  ros::NodeHandle private_nh("~");
  cloud_pub_ = private_nh.advertise<sensor_msgs::PointCloud2>("cloud", 300);
}

SickMrs1000Communication::~SickMrs1000Communication()
{


}

int SickMrs1000Communication::loopOnce()
{
  diagnostics_.update();

  unsigned char receiveBuffer[65536];
  int actual_length = 0;
  static unsigned int iteration_count = 0;

  int result = get_datagram(receiveBuffer, 65536, &actual_length);
  if (result != 0)
  {
    ROS_ERROR("Read Error when getting datagram: %i.", result);
    diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "Read Error when getting datagram.");
    return ExitError; // return failure to exit node
  }
  if(actual_length <= 0)
    return ExitSuccess; // return success to continue looping

  // ----- if requested, skip frames
  if (iteration_count++ % (config_.skip + 1) != 0)
    return ExitSuccess;

  if (publish_datagram_)
  {
    std_msgs::String datagram_msg;
    datagram_msg.data = std::string(reinterpret_cast<char*>(receiveBuffer));
    datagram_pub_.publish(datagram_msg);
  }

  sensor_msgs::LaserScan scan;

  /*
   * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
   */
  char* buffer_pos = (char*)receiveBuffer;
  char *dstart, *dend;
  while( (dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)) )
  {
    size_t dlength = dend - dstart;
    *dend = '\0';
    dstart++;
    sensor_msgs::PointCloud2 cloud;
    int success = scan_and_cloud_parser_->parse_datagram(dstart, dlength, config_, scan, cloud);
    if(cloud.header.frame_id != ""){
      ROS_DEBUG_STREAM("Publish cloud with " << cloud.height * cloud.width
                                            << " points in the frame \"" << cloud.header.frame_id << "\".");
      cloud_pub_.publish(cloud);
    }else{
      ROS_DEBUG_STREAM("Not publishing...");
    }
    if (success == ExitSuccess)
    {
      if(scan.header.frame_id == "laser")
        diagnosticPub_->publish(scan);

    }
    buffer_pos = dend + 1;
  }

  return ExitSuccess; // return success to continue looping
}

int SickMrs1000Communication::init_scanner()
{
  int init_base = SickTimCommonTcp::init_scanner();
  if(init_base != ExitSuccess)
  {
    return init_base;
  }

  /*
   * Set Maintenance access mode to allow mode change to be sent
   */
  std::vector<unsigned char> access_reply;
  int result = sendSOPASCommand("\x02sMN SetAccessMode 03 F4724744\x03\0", &access_reply);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error setting access mode");
    diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SOPAS - Error setting access mode.");
    return ExitError;
  }
  std::string access_reply_str = replyToString(access_reply);
  if (access_reply_str != "sAN SetAccessMode 1")
  {
    ROS_ERROR_STREAM("SOPAS - Error setting access mode, unexpected response : " << access_reply_str);
    diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SOPAS - Error setting access mode.");
    return ExitError;
  }

  /*
   * Application selection and switching 'sWN SetActiveApplication'
   */
  const char setActiveApplication[] = {"\x02sWN SetActiveApplications 1 RANG 1\x03\0"};
  std::vector<unsigned char> activeApplicationReply;
  result = sendSOPASCommand(setActiveApplication, &activeApplicationReply);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - SetActiveApplications failed 'sWN SetActiveApplications'.");
    diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SOPAS - 'sWN SetActiveApplications 1 RANG 1'.");
    return ExitError;
  }
  std::string activeApplicationReplyStr = replyToString(activeApplicationReply);
  ROS_INFO_STREAM("active application reply: " << activeApplicationReplyStr);

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(requestScanData, NULL);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error starting to stream 'LMDscandata'.");
    diagnostics_.broadcast(diagnostic_msgs::DiagnosticStatus::ERROR, "SOPAS - Error starting to stream 'LMDscandata'.");
    return ExitError;
  }

  return ExitSuccess;

}

} /* namespace sick_tim */
