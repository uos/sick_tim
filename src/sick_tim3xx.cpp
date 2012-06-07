/*
 * Copyright (C) 2012, University of Osnabrück
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
 *     * Neither the name of the University of Osnabrück nor the names of its
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class SickTim3xx
{
public:
  SickTim3xx();
  virtual ~SickTim3xx();
  int init_usb();
  int loopOnce();

private:
  static const unsigned int USB_TIMEOUT = 500; // milliseconds

  ssize_t getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID, libusb_device ***list);
  void freeSOPASDeviceList(libusb_device **list);

  void printUSBDeviceDetails(struct libusb_device_descriptor desc);
  void printUSBInterfaceDetails(libusb_device* device);
  void printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices);

  int sendSOPASCommand(libusb_device_handle* device_handle, const char* request, unsigned int timeout);

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // Parameters
  std::string frame_id_;

  // libusb
  libusb_context *ctx_;
  ssize_t numberOfDevices_;
  libusb_device **devices_;
  libusb_device_handle *device_handle_;
};

SickTim3xx::SickTim3xx() :
    ctx_(NULL), numberOfDevices_(0), devices_(NULL), device_handle_(NULL)
{
  ros::NodeHandle pn("~");
  pn.param(std::string("frame"), frame_id_, std::string("/laser_link"));

  pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);
}

SickTim3xx::~SickTim3xx()
{
  if (device_handle_ != NULL)
  {
    /*
     * Stop streaming measurements
     */
    const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
    int result = sendSOPASCommand(device_handle_, requestScanData0, USB_TIMEOUT);
    if (result != 0)
      // use printf because we cannot use ROS_ERROR in the destructor
      printf("\nSOPAS - Error stopping streaming scan data!\n");
    else
      printf("\nSOPAS - Stopped streaming scan data.\n");

    /*
     * Release the interface
     */
    result = libusb_release_interface(device_handle_, 0);
    if (result != 0)
      printf("LIBUSB - Cannot Release Interface!\n");
    else
      printf("LIBUSB - Released Interface.\n");

    /*
     * Close the device handle.
     */
    libusb_close(device_handle_);
  }

  /*
   * Free the list of the USB devices.
   */
  freeSOPASDeviceList(devices_);

  /*
   * Close the LIBUSB session.
   */
  libusb_exit(ctx_);

  printf("sick_tim3xx driver exiting.\n");
}

/**
 * Returns a list of USB devices currently attached to the system and matching the given vendorID and productID.
 */
ssize_t SickTim3xx::getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID,
                                       libusb_device ***list)
{
  libusb_device **resultDevices = NULL;
  ssize_t numberOfResultDevices = 0;
  libusb_device **devices;

  /*
   * Get a list of all USB devices connected.
   */
  ssize_t numberOfDevices = libusb_get_device_list(ctx, &devices);

  /*
   * Iterate through the list of the connected USB devices and search for devices with the given vendorID and productID.
   */
  for (ssize_t i = 0; i < numberOfDevices; i++)
  {
    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      ROS_ERROR("LIBUSB - Failed to get device descriptor");
      continue;
    }

    if (desc.idVendor == vendorID && desc.idProduct == 0x5001)
    {
      /*
       * Add the matching device to the function result list and increase the device reference count.
       */
      resultDevices = (libusb_device **)realloc(resultDevices, sizeof(libusb_device *) * (numberOfResultDevices + 2));
      if (resultDevices == NULL)
      {
        ROS_ERROR("LIBUSB - Failed to allocate memory for the device result list.");
      }
      else
      {
        resultDevices[numberOfResultDevices] = devices[i];
        resultDevices[numberOfResultDevices + 1] = NULL;
        libusb_ref_device(devices[i]);
        numberOfResultDevices++;
      }
    }
  }

  /*
   * Free the list of the connected USB devices and decrease the device reference count.
   */
  libusb_free_device_list(devices, 1);

  /*
   * Prepare the return values of the function.
   */
  *list = resultDevices;
  return numberOfResultDevices;
}

/*
 * Free the list of devices obtained from the function 'getSOPASDeviceList'.
 */
void SickTim3xx::freeSOPASDeviceList(libusb_device **list)
{
  if (!list)
    return;

  int i = 0;
  struct libusb_device *dev;
  while ((dev = list[i++]) != NULL)
    libusb_unref_device(dev);

  free(list);
}

/*
 * Print the device details such as USB device class, vendor id and product id to the console.
 */
void SickTim3xx::printUSBDeviceDetails(struct libusb_device_descriptor desc)
{
  ROS_INFO("Device Class: 0x%x", desc.bDeviceClass);
  ROS_INFO("VendorID:     0x%x", desc.idVendor);
  ROS_INFO("ProductID:    0x%x", desc.idProduct);
}

/*
 * Iterate through the the interfaces of the USB device and print out the interface details to the console.
 */
void SickTim3xx::printUSBInterfaceDetails(libusb_device* device)
{
  struct libusb_config_descriptor *config;

  /*
   * Get a USB configuration descriptor based on its index.
   */
  libusb_get_config_descriptor(device, 0, &config);

  ROS_INFO("Interfaces: %i", (int)config->bNumInterfaces);
  ROS_INFO("----------------------------------------");

  const struct libusb_interface *interface;
  const struct libusb_interface_descriptor *interface_descriptor;
  const struct libusb_endpoint_descriptor *endpoint_descriptor;

  int i, j, k;
  for (i = 0; i < config->bNumInterfaces; i++)
  {
    interface = &config->interface[i];
    ROS_INFO("Number of alternate settings: %i", interface->num_altsetting);

    for (j = 0; j < interface->num_altsetting; j++)
    {
      interface_descriptor = &interface->altsetting[j];

      ROS_INFO("Interface number: %i", (int)interface_descriptor->bInterfaceNumber);
      ROS_INFO("Number of endpoints: %i", (int)interface_descriptor->bNumEndpoints);

      for (k = 0; k < interface_descriptor->bNumEndpoints; k++)
      {
        endpoint_descriptor = &interface_descriptor->endpoint[k];
        ROS_INFO("Descriptor Type: %i", endpoint_descriptor->bDescriptorType);
        ROS_INFO("EP Address: %i", endpoint_descriptor->bEndpointAddress);
      }
    }

    if (i < config->bNumInterfaces - 1)
    {
      ROS_INFO("----------------------------------------");
    }
  }

  /*
   * Free the configuration descriptor obtained from 'libusb_get_config_descriptor'
   */
  libusb_free_config_descriptor(config);
}

/**
 * Print the USB device information of the connected TIM3xx devices to the console.
 */
void SickTim3xx::printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices)
{
  ssize_t i;
  for (i = 0; i < numberOfDevices; i++)
  {
    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      ROS_ERROR("LIBUSB - Failed to get device descriptor");
      continue;
    }
    if (result == 0)
    {
      ROS_INFO("SICK AG - TIM3XX - [%zu]", (i + 1));
      ROS_INFO("----------------------------------------");
      printUSBDeviceDetails(desc);
      ROS_INFO("----------------------------------------");
      printUSBInterfaceDetails(devices[i]);
      ROS_INFO("----------------------------------------");
    }
  }

  if (numberOfDevices == 0)
  {
    ROS_INFO("LIBUSB - No SICK TIM3xx device connected.");
  }
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int SickTim3xx::sendSOPASCommand(libusb_device_handle* device_handle, const char* request, unsigned int timeout)
{
  int result = 0;
  unsigned char receiveBuffer[65536];

  /*
   * Write a SOPAS variable read request to the device.
   */
  ROS_DEBUG("LIBUSB - Write data... %s", request);

  int actual_length = 0;
  int requestLength = strlen(request);
  result = libusb_bulk_transfer(device_handle, (2 | LIBUSB_ENDPOINT_OUT), (unsigned char*)request, requestLength,
                                &actual_length, 0);
  if (result != 0 || actual_length != requestLength)
  {
    ROS_ERROR("LIBUSB - Write Error: %i.", result);
    return result;
  }

  /*
   * Read the SOPAS device response with the given timeout.
   */
  result = libusb_bulk_transfer(device_handle, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual_length, timeout);
  if (result != 0)
  {
    ROS_ERROR("LIBUSB - Read Error: %i.", result);
    return result;
  }

  receiveBuffer[actual_length] = 0;
  ROS_DEBUG("LIBUSB - Read data...  %s", receiveBuffer);

  return result;
}

/*
 * provided as a separate method (not inside constructor) so we can return error codes
 */
int SickTim3xx::init_usb()
{
  /*
   * Create and initialize a new LIBUSB session.
   */
  int result = libusb_init(&ctx_);
  if (result != 0)
  {
    ROS_ERROR("LIBUSB - Initialization failed with the following error code: %i.", result);
    return EXIT_FAILURE;
  }

  /*
   * Set the verbosity level to 3 as suggested in the documentation.
   */
  libusb_set_debug(ctx_, 3);

  /*
   * Get a list of all SICK TIM3xx devices connected to the USB bus.
   *
   * As a shortcut, you can also use the LIBUSB function:
   * libusb_open_device_with_vid_pid(ctx, 0x19A2, 0x5001).
   */
  int vendorID = 0x19A2; // SICK AG
  int deviceID = 0x5001; // TIM3XX
  numberOfDevices_ = getSOPASDeviceList(ctx_, vendorID, deviceID, &devices_);

  /*
   * If available, open the first SICK TIM3xx device.
   */
  if (numberOfDevices_ == 0)
  {
    ROS_ERROR("No SICK TiM3xx devices connected!");
    return EXIT_FAILURE;
  }
  else if (numberOfDevices_ > 1)
  {
    ROS_WARN("%zu TiM3xx scanners connected, using the first one", numberOfDevices_);
  }

  /*
   * Print out the SOPAS device information to the console.
   */
  printSOPASDeviceInformation(numberOfDevices_, devices_);

  /*
   * Open the device handle and detach all kernel drivers.
   */
  libusb_open(devices_[0], &device_handle_);
  if (device_handle_ == NULL)
  {
    ROS_ERROR("LIBUSB - Cannot open device; please read sick_tim3xx/udev/README");
    return EXIT_FAILURE;
  }
  else
  {
    ROS_DEBUG("LIBUSB - Device opened");
  }

  if (libusb_kernel_driver_active(device_handle_, 0) == 1)
  {
    ROS_DEBUG("LIBUSB - Kernel driver active");
    if (libusb_detach_kernel_driver(device_handle_, 0) == 0)
    {
      ROS_DEBUG("LIBUSB - Kernel driver detached!");
    }
  }

  /*
   * Claim the interface 0
   */
  result = libusb_claim_interface(device_handle_, 0);
  if (result < 0)
  {
    ROS_ERROR("LIBUSB - Cannot claim interface");
    return EXIT_FAILURE;
  }
  else
  {
    ROS_INFO("LIBUSB - Claimed interface");
  }

  /*
   * Read the SOPAS variable 'DeviceIdent' by index.
   */
  const char requestDeviceIdent[] = "\x02sRI0\x03\0";
  result = sendSOPASCommand(device_handle_, requestDeviceIdent, USB_TIMEOUT);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'DeviceIdent'.");
  }

  /*
   * Read the SOPAS variable 'SerialNumber' by name.
   */
  const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
  result = sendSOPASCommand(device_handle_, requestSerialNumber, USB_TIMEOUT);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'SerialNumber'.");
  }

  /*
   * Read the SOPAS variable 'FirmwareVersion' by name.
   */
  const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
  result = sendSOPASCommand(device_handle_, requestFirmwareVersion, USB_TIMEOUT);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error reading variable 'FirmwareVersion'.");
  }

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(device_handle_, requestScanData, USB_TIMEOUT);
  if (result != 0)
  {
    ROS_ERROR("SOPAS - Error starting to stream 'LMDscandata'.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

int SickTim3xx::loopOnce()
{
  int result = 0;
  unsigned char receiveBuffer[65536];
  unsigned char receiveBufferCopy[65536]; // only for debugging
  int actual_length = 0;
  static const size_t NUM_FIELDS = 580;
  char* fields[NUM_FIELDS];
  size_t count;

  result = libusb_bulk_transfer(device_handle_, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual_length, USB_TIMEOUT);
  if (result != 0)
  {
    if (result == LIBUSB_ERROR_TIMEOUT)
    {
      ROS_WARN("LIBUSB - Read Error: LIBUSB_ERROR_TIMEOUT.");
      return EXIT_SUCCESS; // return success to continue looping
    }
    else
    {
      ROS_ERROR("LIBUSB - Read Error: %i.", result);
      return EXIT_FAILURE;
    }
  }

  receiveBuffer[actual_length] = 0;
  // ROS_DEBUG("LIBUSB - Read data...  %s", receiveBuffer);

  // ----- tokenize
  strncpy((char*)receiveBufferCopy, (char*)receiveBuffer, 65535); // receiveBuffer will be changed by strtok
  receiveBufferCopy[65535] = 0;

  count = 0;
  fields[count] = strtok((char *)receiveBuffer, " ");
  // ROS_DEBUG("%d: %s ", count, fields[count]);

  while (fields[count] != NULL)
  {
    count++;
    if (count > NUM_FIELDS)
      break;

    fields[count] = strtok(NULL, " ");
    // ROS_DEBUG("%d: %s ", count, fields[count]);
  }

  if (count < NUM_FIELDS)
  {
    ROS_WARN(
        "received less fields than expected fields (actual: %zu, expected: %zu), ignoring scan", count, NUM_FIELDS);
    ROS_DEBUG("received message was: %s", receiveBufferCopy);
    return EXIT_SUCCESS; // return success to continue looping
  }
  else if (count > NUM_FIELDS)
  {
    ROS_WARN("received more fields than expected (expected: %zu), ignoring scan", NUM_FIELDS);
    ROS_DEBUG("received message was: %s", receiveBufferCopy);
    return EXIT_SUCCESS; // return success to continue looping
  }

  // ----- read fields into msg
  sensor_msgs::LaserScan msg;

  msg.header.frame_id = frame_id_;
  ros::Time start_time = ros::Time::now();

  // <STX> (\x02)
  // 0: Type of command (SN)
  // 1: Command (LMDscandata)
  // 2: Firmware version number (1)
  // 3: Device number (1)
  // 4: Serial number (B96518)
  // 5 + 6: Device Status (0 0 = ok, 0 1 = error)
  // 7: Telegram counter (99)
  // 8: Scan counter (9A)
  // 9: Time since startup (13C8E59)
  // 10: Time of transmission (13C9CBE)
  // 11 + 12: Input status (0 0)
  // 13 + 14: Output status (8 0)
  // 15: Reserved Byte A (0)

  // 16: Scanning Frequency (5DC)
  unsigned short scanning_freq = -1;
  sscanf(fields[16], "%hx", &scanning_freq);
  msg.scan_time = 1.0 / (scanning_freq / 100.0);
  // ROS_DEBUG("hex: %s, scanning_freq: %d, scan_time: %f", fields[16], scanning_freq, msg.scan_time);

  // 17: Measurement Frequency (36)
  unsigned short measurement_freq = -1;
  sscanf(fields[17], "%hx", &measurement_freq);
  msg.time_increment = 1.0 / (measurement_freq * 100.0);
  // ROS_DEBUG("measurement_freq: %d, time_increment: %f", measurement_freq, msg.time_increment);

  // adjust start time:
  // - last scan point = now  ==>  first scan point = now - 271 * time increment
  // - also just assume 0.001 s USB latency between scanner and PC for now
  msg.header.stamp = start_time - ros::Duration().fromSec(271 * msg.time_increment - 0.001);

  // 18: Number of encoders (0)
  // 19: Number of 16 bit channels (1)
  // 20: Measured data contents (DIST1)

  // 21: Scaling factor (3F800000)
  // ignored for now (is always 1.0):
//      unsigned int scaling_factor_int = -1;
//      sscanf(fields[21], "%x", &scaling_factor_int);
//
//      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
//      // ROS_DEBUG("hex: %s, scaling_factor_int: %d, scaling_factor: %f", fields[21], scaling_factor_int, scaling_factor);

  // 22: Scaling offset (00000000) -- always 0
  // 23: Starting angle (FFF92230)
  int starting_angle = -1;
  sscanf(fields[23], "%x", &starting_angle);
  msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI/2;
  // ROS_DEBUG("starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

  // 24: Angular step width (2710)
  unsigned short angular_step_width = -1;
  sscanf(fields[24], "%hx", &angular_step_width);
  msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
  msg.angle_max = msg.angle_min + 270.0 * msg.angle_increment - M_PI/2;
  // ROS_DEBUG("angular_step_width: %d, angle_increment: %f, angle_max: %f", angular_step_width, msg.angle_increment, msg.angle_max);

  // 25: Number of data (10F)

  // 26..296: Data_1 .. Data_n
  msg.ranges.resize(271);
  for (int j = 0; j < 271; ++j)
  {
    unsigned short range;
    sscanf(fields[j + 26], "%hx", &range);
    msg.ranges[j] = range / 1000.0;
  }

  // 297: Number of 8 bit channels (1)
  // 298: Measured data contents (RSSI1)
  // 299: Scaling factor (3F800000)
  // 300: Scaling offset (00000000)
  // 301: Starting angle (FFF92230)
  // 302: Angular step width (2710)
  // 303: Number of data (10F)
  // 304..574: Data_1 .. Data_n

  msg.intensities.resize(271);
  for (int j = 0; j < 271; ++j)
  {
    unsigned short intensity;
    sscanf(fields[j + 304], "%hx", &intensity);
    msg.intensities[j] = intensity;
  }

  // 575: Position (0)
  // 576: Name (0)
  // 577: Comment (0)
  // 578: Time information (0)
  // 579: Event information (0)
  // <ETX> (\x03)

  msg.range_min = 0.05;
  msg.range_max = 4.0;

  pub_.publish(msg);
  return EXIT_SUCCESS;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim3xx");

  SickTim3xx s;
  s.init_usb();

  int result = EXIT_SUCCESS;
  while (ros::ok() && (result == EXIT_SUCCESS))
  {
    result = s.loopOnce();
  }

  return result;
}
