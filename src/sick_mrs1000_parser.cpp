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
 *      Author: Sebastian Pütz <spuetz@uos.de>
 *
 */

#include <sick_tim/sick_mrs1000_parser.h>

#include <ros/ros.h>

namespace sick_tim
{

SickMRS1000Parser::SickMRS1000Parser() :
    override_range_min_(0.2),
    override_range_max_(64.0),
    override_time_increment_(-1.0f),
    modifier_(cloud_),
    x_iter((modifier_.setPointCloud2FieldsByString(1, "xyz"), sensor_msgs::PointCloud2Iterator<float>(cloud_, "x"))),
    y_iter(cloud_, "y"), z_iter(cloud_, "z"), layer_count_(0)

{
}

SickMRS1000Parser::~SickMRS1000Parser()
{
}

int SickMRS1000Parser::parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                                      sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud2& cloud)
{
  // Only allow config changes for a whole new cloud / scan and not for partial fragments of a cloud.
  if(layer_count_ == 0)
  {
    current_config_ = config;
  }

  static const size_t HEADER_FIELDS = 32;
  char* cur_field;
  size_t count;

  // Reserve sufficient space
  std::vector<char *> fields;
  fields.reserve(datagram_length / 2);

  // ----- only for debug output
  char datagram_copy[datagram_length + 1];
  strncpy(datagram_copy, datagram, datagram_length); // datagram will be changed by strtok
  datagram_copy[datagram_length] = 0;

  // ----- tokenize
  cur_field = strtok(datagram, " ");

  while (cur_field != NULL)
  {
    fields.push_back(cur_field);
    cur_field = strtok(NULL, " ");
  }

  count = fields.size();

  // Validate header. Total number of tokens is highly unreliable as this may
  // change when you change the scanning range or the device name using SOPAS ET
  // tool. The header remains stable, however.
  if (count < HEADER_FIELDS)
  {
    ROS_WARN(
        "received less fields than minimum fields (actual: %zu, minimum: %zu), ignoring scan", count, HEADER_FIELDS);
    ROS_WARN("are you using the correct node? (124 --> sick_tim310_1130000m01, > 33 --> sick_tim551_2050001, 580 --> sick_tim310s01, 592 --> sick_tim310)");
    // ROS_DEBUG("received message was: %s", datagram_copy);
    return ExitError;
  }
  if (strcmp(fields[20], "DIST1"))
  {
    ROS_WARN("Field 20 of received data is not equal to DIST1 (%s). Unexpected data, ignoring scan", fields[20]);
    return ExitError;
  }

  // More in depth checks: check data length and RSSI availability
  // 25: Number of data (<= 10F)
  unsigned short int number_of_data = 0;
  sscanf(fields[25], "%hx", &number_of_data);

  if (number_of_data < 1 || number_of_data > 1101)
  {
    ROS_WARN("Data length is outside acceptable range 1-1101 (%d). Ignoring scan", number_of_data);
    return ExitError;
  }
  if (count < HEADER_FIELDS + number_of_data)
  {
    ROS_WARN("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
    return ExitError;
  }
  ROS_DEBUG("Number of data: %d", number_of_data);

  // Calculate offset of field that contains indicator of whether or not RSSI data is included
  size_t rssi_idx = 26 + number_of_data;
  int tmp;
  sscanf(fields[rssi_idx], "%d", &tmp);
  bool rssi = tmp > 0;
  unsigned short int number_of_rssi_data = 0;
  if (rssi)
  {
    sscanf(fields[rssi_idx + 6], "%hx", &number_of_rssi_data);

    // Number of RSSI data should be equal to number of data
    if (number_of_rssi_data != number_of_data)
    {
      ROS_WARN("Number of RSSI data is lower than number of range data (%d vs %d", number_of_data, number_of_rssi_data);
      return ExitError;
    }

    // Check if the total length is still appropriate.
    // RSSI data size = number of RSSI readings + 6 fields describing the data
    if (count < HEADER_FIELDS + number_of_data + number_of_rssi_data + 6)
    {
      ROS_WARN("Less fields than expected for %d data points (%zu). Ignoring scan", number_of_data, count);
      return ExitError;
    }

    if (strcmp(fields[rssi_idx + 1], "RSSI1"))
    {
      ROS_WARN("Field %zu of received data is not equal to RSSI1 (%s). Unexpected data, ignoring scan", rssi_idx + 1, fields[rssi_idx + 1]);
    }
  }

  short layer = -1;
  sscanf(fields[15], "%hx", &layer);
  scan.header.seq = layer;

  // Only set the frame id for the layer 0, because only the points of that layer 0 lie in a plane.
  // If the frame_id is not set the caller will not and should not publish the scan.
  scan.header.frame_id = layer == 0 ? current_config_.frame_id.c_str() : "";

  // ----- read fields into scan
  ROS_DEBUG_STREAM("publishing with frame_id " << scan.header.frame_id);

  ros::Time start_time = ros::Time::now(); // will be adjusted in the end

  // 16: Scanning Frequency (5DC)
  unsigned short scanning_freq = -1;
  sscanf(fields[16], "%hx", &scanning_freq);
  scan.scan_time = 1.0 / (scanning_freq / 100.0);
  // ROS_DEBUG("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, scan.scan_time);

  // 17: Measurement Frequency (36)
  unsigned short measurement_freq = -1;
  sscanf(fields[17], "%hx", &measurement_freq);
  // Measurement Frequency = Inverse of the time between two measurement shots -> 275 * 4 / 20ms = 55kHz
  scan.time_increment = 1.0 / (4 * measurement_freq * 100.0);
  if (override_time_increment_ > 0.0)
  {
    // Some lasers may report incorrect measurement frequency
    scan.time_increment = override_time_increment_;
  }

  // 23: Starting angle (FFF92230)
  int starting_angle = -1;
  sscanf(fields[23], "%x", &starting_angle);
  scan.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
  // ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, scan.angle_min);

  // 24: Angular step width (2710)
  unsigned short angular_step_width = -1;
  sscanf(fields[24], "%hx", &angular_step_width);
  scan.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
  scan.angle_max = scan.angle_min + (number_of_data - 1) * scan.angle_increment;

  // 25: Number of data (<= 10F)
  // This is already determined above in number_of_data

  // adjust angle_min to min_ang config param
  int index_min = 0;
  while (scan.angle_min + scan.angle_increment < current_config_.min_ang)
  {
    scan.angle_min += scan.angle_increment;
    index_min++;
  }

  // adjust angle_max to max_ang config param
  int index_max = number_of_data - 1;
  while (scan.angle_max - scan.angle_increment > current_config_.max_ang)
  {
    scan.angle_max -= scan.angle_increment;
    index_max--;
  }

  ROS_DEBUG("index_min: %d, index_max: %d", index_min, index_max);
  // ROS_DEBUG("angular_step_width: %d, angle_increment: %f, angle_max: %f", angular_step_width, scan.angle_increment, scan.angle_max);

  double phi = scan.angle_min;
  // -250 -> 2.5 => x -> -x/100
  // radiant(-x/100) -> (-x / 100) * (pi / 180) -> -x * pi / 18000
  double alpha = -layer * M_PI / 18000;

  // order of layers: 2, 3, 1, 4
  // layer 2: +0.0 degree, layer ==  0
  // layer 3: +2.5 degree, layer == -250
  // layer 1: -2.5 degree, layer ==  250
  // layer 4: +5.0 degree, layer == -500

  // first layer
  if(layer == 0){
    modifier_.resize(4 * (index_max - index_min + 1));
    x_iter = sensor_msgs::PointCloud2Iterator<float>(cloud_, "x");
    y_iter = sensor_msgs::PointCloud2Iterator<float>(cloud_, "y");
    z_iter = sensor_msgs::PointCloud2Iterator<float>(cloud_, "z");
    // 26..26 + n - 1: Data_1 .. Data_n
    scan.ranges.resize(index_max - index_min + 1);
    // set time when first row is received.
    cloud.header.stamp = start_time + ros::Duration().fromSec(current_config_.time_offset);
  }

  // check for space, space was allocated if the layer was layer == 0 (Layer2) sometime before.
  if(modifier_.size() > 0){
    layer_count_++;
    for (int j = index_min; j <= index_max; ++j)
    {

      unsigned short range;
      sscanf(fields[j + 26], "%hx", &range);
      float range_meter = range / 1000.0;

      // only copy data to laser scan for layer 2 (layer == 0 degree)
      if(layer == 0)
      {
        if (range == 0)
          scan.ranges[j - index_min] = std::numeric_limits<float>::infinity();
        else
          scan.ranges[j - index_min] = range_meter;
      }

      /*
       * Transform point from spherical coordinates to Cartesian coordinates.
       * Alpha is measured from the xy-plane and not from the
       * upper z axis ---> use "pi/2 - alpha" for transformation
       * Simplified sin(pi/2 - alpha) to cos(alpha).
       * Simplified cos(pi/2 - alpha) to sin(alpha).
       */
      *x_iter = range_meter * cos(alpha) * cos(phi);
      *y_iter = range_meter * cos(alpha) * sin(phi);
      *z_iter = range_meter * sin(alpha);

      ++x_iter;
      ++y_iter;
      ++z_iter;

      phi += scan.angle_increment;
    }

    // last layer in the ordered list: 0, -250, 250, -500
    if(layer == -500){
      ROS_ASSERT_MSG(layer_count_ == 4, "Expected four layers and layer == -500 to be the last layer! Package loss in communication!");
      layer_count_ = 0;
      cloud = cloud_;
      cloud.header.frame_id = "laser";
    }
  }

  if (current_config_.intensity) {
    if (rssi)
    {
      // 26 + n: RSSI data included

      //   26 + n + 1 = RSSI Measured Data Contents (RSSI1)
      //   26 + n + 2 = RSSI scaling factor (3F80000)
      //   26 + n + 3 = RSSI Scaling offset (0000000)
      //   26 + n + 4 = RSSI starting angle (equal to Range starting angle)
      //   26 + n + 5 = RSSI angular step width (equal to Range angular step width)
      //   26 + n + 6 = RSSI number of data (equal to Range number of data)
      //   26 + n + 7 .. 26 + n + 7 + n - 1: RSSI_Data_1 .. RSSI_Data_n
      //   26 + n + 7 + n .. 26 + n + 7 + n + 2 = unknown (but seems to be [0, 1, B] always)
      //   26 + n + 7 + n + 2 .. count - 4 = device label
      //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
      //   <ETX> (\x03)
      scan.intensities.resize(index_max - index_min + 1);
      size_t offset = 26 + number_of_data + 7;
      for (int j = index_min; j <= index_max; ++j)
      {
        unsigned short intensity;
        sscanf(fields[j + offset], "%hx", &intensity);
        scan.intensities[j - index_min] = intensity;
      }
    } else {
      ROS_WARN_ONCE("Intensity parameter is enabled, but the scanner is not configured to send RSSI values! "
                        "Please read the section 'Enabling intensity (RSSI) output' here: http://wiki.ros.org/sick_tim.");
    }
  }

  // 26 + n: RSSI data included
  // IF RSSI not included:
  //   26 + n + 1 .. 26 + n + 3 = unknown (but seems to be [0, 1, B] always)
  //   26 + n + 4 .. count - 4 = device label
  //   count - 3 .. count - 1 = unknown (but seems to be 0 always)
  //   <ETX> (\x03)

  scan.range_min = override_range_min_; // TODO
  scan.range_max = override_range_max_;

  // ----- adjust start time
  // - last scan point = now  ==>  first scan point = now - number_of_data * time increment
  scan.header.stamp = start_time - ros::Duration().fromSec(number_of_data * scan.time_increment);

  // - shift forward to time of first published scan point
  scan.header.stamp += ros::Duration().fromSec((double)index_min * scan.time_increment);

  // - add time offset (to account for USB latency etc.)
  scan.header.stamp += ros::Duration().fromSec(current_config_.time_offset);

  // ----- consistency check
  float expected_time_increment = scan.scan_time * scan.angle_increment / (2.0 * M_PI);
  if (fabs(expected_time_increment - scan.time_increment) > 0.00001)
  {
    ROS_WARN_THROTTLE(60, "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
        "Expected time_increment: %.9f, reported time_increment: %.9f. "
        "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                      expected_time_increment, scan.time_increment);
  }

  return ExitSuccess;
}

void SickMRS1000Parser::set_range_min(float min)
{
  override_range_min_ = min;
}

void SickMRS1000Parser::set_range_max(float max)
{
  override_range_max_ = max;
}

void SickMRS1000Parser::set_time_increment(float time)
{
  override_time_increment_ = time;
}

} /* namespace sick_tim */
