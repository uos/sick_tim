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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libusb-1.0/libusb.h>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

/**
 * Function prototypes.
 */
ssize_t getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID, libusb_device ***list);
void freeSOPASDeviceList(libusb_device **list);
void printUSBDeviceDetails(struct libusb_device_descriptor desc);
void printUSBInterfaceDetails(libusb_device* device);

void printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices);
int sendSOPASCommand(libusb_device_handle* device_handle, const char* request, unsigned int timeout);


/**
 * Returns a list of USB devices currently attached to the system and matching the given vendorID and productID.
 */
ssize_t getSOPASDeviceList(libusb_context *ctx, uint16_t vendorID, uint16_t productID, libusb_device ***list)
{

  libusb_device **resultDevices = NULL;
  ssize_t numberOfResultDevices = 0;

  libusb_device **devices;
  ssize_t numberOfDevices;

  /*
   * Get a list of all USB devices connected.
   */
  numberOfDevices = libusb_get_device_list(ctx, &devices);

  /*
   * Iterate through the list of the connected USB devices and search for devices with the given vendorID and prodcutID.
   */
  ssize_t i;
  for (i = 0; i < numberOfDevices; i++)
  {

    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      fprintf(stderr, "LIBUSB - Failed to get device descriptor");
    }

    if (desc.idVendor == vendorID && desc.idProduct == 0x5001)
    {

      /*
       * Add the matching device to the function result list and increase the device reference count.
       */
      resultDevices = (libusb_device **)realloc(resultDevices, sizeof(libusb_device *) + (numberOfResultDevices + 1));
      if (resultDevices == NULL)
      {
        fprintf(stderr, "LIBUSB - Failed to allocate memory for the device result list.");
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
void freeSOPASDeviceList(libusb_device **list)
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
void printUSBDeviceDetails(struct libusb_device_descriptor desc)
{

  printf("Device Class: 0x%x\n", desc.bDeviceClass);
  printf("VendorID:     0x%x\n", desc.idVendor);
  printf("ProductID:    0x%x\n", desc.idProduct);
}

/*
 * Iterate through the the interfaces of the USB device and print out the interface details to the console.
 */
void printUSBInterfaceDetails(libusb_device* device)
{
  struct libusb_config_descriptor *config;

  /*
   * Get a USB configuration descriptor based on its index.
   */
  libusb_get_config_descriptor(device, 0, &config);

  printf("Interfaces: %i\n", (int)config->bNumInterfaces);
  printf("----------------------------------------\n");

  const struct libusb_interface *interface;
  const struct libusb_interface_descriptor *interface_descriptor;
  const struct libusb_endpoint_descriptor *endpoint_descriptor;

  int i, j, k;
  for (i = 0; i < config->bNumInterfaces; i++)
  {

    interface = &config->interface[i];
    printf("Number of alternate settings: %i\n", interface->num_altsetting);

    for (j = 0; j < interface->num_altsetting; j++)
    {

      interface_descriptor = &interface->altsetting[j];

      printf("Interface number: %i\n", (int)interface_descriptor->bInterfaceNumber);
      printf("Number of endpoints: %i\n", (int)interface_descriptor->bNumEndpoints);

      for (k = 0; k < interface_descriptor->bNumEndpoints; k++)
      {

        endpoint_descriptor = &interface_descriptor->endpoint[k];
        printf("Descriptor Type: %i\n", endpoint_descriptor->bDescriptorType);
        printf("EP Address: %i\n", endpoint_descriptor->bEndpointAddress);
      }
    }

    if (i < config->bNumInterfaces - 1)
    {

      printf("----------------------------------------\n");
    }
  }

  /*
   * Free a configuration descriptor obtained from 'libusb_get_config_descriptor'
   */
  libusb_free_config_descriptor(config);
}

/**
 * Print the USB device information of the connected TIM3xx devices to the console.
 */
void printSOPASDeviceInformation(ssize_t numberOfDevices, libusb_device** devices)
{
  ssize_t i;
  for (i = 0; i < numberOfDevices; i++)
  {
    struct libusb_device_descriptor desc;
    int result = libusb_get_device_descriptor(devices[i], &desc);
    if (result < 0)
    {
      fprintf(stderr, "LIBUSB - Failed to get device descriptor");
    }
    if (result == 0)
    {
      printf("SICK AG - TIM3XX - [%i]\n", (i + 1));
      printf("----------------------------------------\n");
      printUSBDeviceDetails(desc);
      printf("----------------------------------------\n");
      printUSBInterfaceDetails(devices[i]);
      printf("----------------------------------------\n");
    }
  }

  if (numberOfDevices == 0)
  {
    printf("LIBUSB - No SICK TIM3xx device connected.\n");
  }
}

/**
 * Send a SOPAS command to the device and print out the response to the console.
 */
int sendSOPASCommand(libusb_device_handle* device_handle, const char* request, unsigned int timeout)
{

  int result = 0;

  unsigned char receiveBuffer[65536];

  /*
   * Write a SOPAS variable read request to the device.
   */
  printf("\nLIBUSB - Write data... %s\n", request);

  int actual = 0;
  int requestLength = strlen(request);
  result = libusb_bulk_transfer(device_handle, (2 | LIBUSB_ENDPOINT_OUT), (unsigned char*)request, requestLength,
                                &actual, 0);
  if (result != 0 || actual != requestLength)
  {
    fprintf(stderr, "LIBUSB - Write Error: %i.\n", result);
  }

  /*
   * Read the SOPAS device response with the given timeout.
   */
  result = libusb_bulk_transfer(device_handle, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual, timeout);
  if (result != 0)
  {
    fprintf(stderr, "LIBUSB - Read Error: %i.\n", result);
  }

  receiveBuffer[actual] = 0;
  printf("LIBUSB - Read data...  %s\n\n", receiveBuffer);

  return result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sick_tim3xx");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  std::string frame_id;
  pn.param(std::string("frame"), frame_id, std::string("laser_link"));

  ros::Publisher pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

   /*
   * Create and initialize a new LIBUSB session.
   */
  libusb_context *ctx = NULL;
  int result = libusb_init(&ctx);
  if (result != 0)
  {
    printf("LIBUSB - Initialization failed with the following error code: %i.\n", result);
  }

  /*
   * Set the verbosity level to 3 as suggested in the documentation.
   */
  libusb_set_debug(ctx, 3);

  /*
   * Get a list of all SICK TIM3xx devices connected to the USB bus.
   *
   * As a shortcut, you can also use the LIBUSB function:
   * libusb_open_device_with_vid_pid(ctx, 0x19A2, 0x5001).
   */
  libusb_device **devices;
  int vendorID = 0x19A2; // SICK AG
  int deviceID = 0x5001; // TIM3XX
  ssize_t numberOfDevices = getSOPASDeviceList(ctx, vendorID, deviceID, &devices);

  /*
   * If available, open the first SICK TIM3xx device.
   */
  if (numberOfDevices > 0)
  {
    /*
     * Print out the SOPAS device information to the console.
     */
    printSOPASDeviceInformation(numberOfDevices, devices);

    /*
     * Open the device handle and detach all kernel drivers.
     */
    libusb_device_handle *device_handle;
    libusb_open(devices[0], &device_handle);
    if (device_handle == NULL)
    {
      fprintf(stderr, "\nLIBUSB - Cannot open device\n");
    }
    else
    {
      printf("\nLIBUSB - Device opened\n");
    }

    if (libusb_kernel_driver_active(device_handle, 0) == 1)
    {
      printf("LIBUSB - Kernel driver active\n");
      if (libusb_detach_kernel_driver(device_handle, 0) == 0)
      {
        printf("LIBUSB - Kernel driver detached!\n");
      }
    }

    /*
     * Claim the interface 0
     */
    result = libusb_claim_interface(device_handle, 0);
    if (result < 0)
    {
      fprintf(stderr, "LIBUSB - Cannot claim interface\n");
    }
    else
    {
      printf("LIBUSB - Claimed interface\n");
    }

    /*
     * Read the SOPAS variable 'DeviceIdent' by index.
     */
    const char requestDeviceIdent[] = "\x02sRI0\x03\0";
    result = sendSOPASCommand(device_handle, requestDeviceIdent, 500);
    if (result != 0)
    {
      fprintf(stderr, "SOPAS - Error reading variable 'DeviceIdent'.\n");
    }

    /*
     * Read the SOPAS variable 'SerialNumber' by name.
     */
    const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
    result = sendSOPASCommand(device_handle, requestSerialNumber, 500);
    if (result != 0)
    {
      fprintf(stderr, "SOPAS - Error reading variable 'SerialNumber'.\n");
    }

    /*
     * Read the SOPAS variable 'FirmwareVersion' by name.
     */
    const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
    result = sendSOPASCommand(device_handle, requestFirmwareVersion, 500);
    if (result != 0)
    {
      fprintf(stderr, "SOPAS - Error reading variable 'FirmwareVersion'.\n");
    }

    /*
     * Read the SOPAS variable 'LMDscandata' by name.
     */
    const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
    result = sendSOPASCommand(device_handle, requestScanData, 500);
    if (result != 0)
    {
      fprintf(stderr, "SOPAS - Error reading variable 'LMDscandata'.\n");
    }

    int result = 0;
    unsigned char receiveBuffer[65536];
    int actual = 0;
    int i;
    static size_t NUM_FIELDS = 580;
    char* fields[NUM_FIELDS];
    unsigned int count;

    // TODO: while (ros::ok())
    for (i = 0; i < 1; i++)
    {
      result = libusb_bulk_transfer(device_handle, (1 | LIBUSB_ENDPOINT_IN), receiveBuffer, 65535, &actual, 500);
      if (result != 0)
      {
        fprintf(stderr, "LIBUSB - Read Error: %i.\n", result);
      }

      receiveBuffer[actual] = 0;
      printf("LIBUSB - Read data...  %s\n\n", receiveBuffer);

      // ----- tokenize
      count = 0;
      fields[count] = strtok((char *)receiveBuffer, " ");
      printf("%d: %s ", count, fields[count]);

      while (fields[count] != NULL)
      {
        count++;
        if  (count > NUM_FIELDS)
          break;

        fields[count] = strtok(NULL, " ");
        printf("%d: %s ", count, fields[count]);
      }
      printf("\n");

      if (count != NUM_FIELDS)
        printf("Error: received %d fields (expected: %d)", count, NUM_FIELDS);

      // ----- read fields into msg

      sensor_msgs::LaserScan msg;

      msg.header.frame_id = frame_id;
      msg.header.stamp = ros::Time::now();


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
      printf("hex: %s, scanning_freq: %d, scan_time: %f\n", fields[16], scanning_freq, msg.scan_time);


      // 17: Measurement Frequency (36)
      unsigned short measurement_freq = -1;
      sscanf(fields[17], "%hx", &measurement_freq);
      msg.time_increment = 1.0 / (measurement_freq * 100.0);
      printf("measurement_freq: %d, time_increment: %f\n", measurement_freq, msg.time_increment);

      // 18: Number of encoders (0)
      // 19: Number of 16 bit channels (1)
      // 20: Measured data contents (DIST1)

      // 21: Scaling factor (3F800000)
      unsigned int scaling_factor_int = -1;
      sscanf(fields[21], "%x", &scaling_factor_int);

      float scaling_factor = reinterpret_cast<float&>(scaling_factor_int);
      printf("hex: %s, scaling_factor_int: %d, scaling_factor: %f\n", fields[21], scaling_factor_int, scaling_factor);

      // 22: Scaling offset (00000000) -- always 0
      // 23: Starting angle (FFF92230)
      int starting_angle = -1;
      sscanf(fields[23], "%x", &starting_angle);
      msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI;
      printf("starting_angle: %d, angle_min: %f\n", starting_angle, msg.angle_min);

      // 24: Angular step width (2710)
      unsigned short angular_step_width = -1;
      sscanf(fields[24], "%hx", &angular_step_width);
      msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;
      msg.angle_max = msg.angle_min + 270.0 * msg.angle_increment;
      printf("angular_step_width: %d, angle_increment: %f, angle_max: %f\n", angular_step_width, msg.angle_increment, msg.angle_max);


      // 25: Number of data (10F)
      // 26..296: Data_1 .. Data_n

      // TODO: msg.ranges

      // 297: Number of 8 bit channels (1)
      // 298: Measured data contents (RSSI1)
      // 299: Scaling factor (3F800000)
      // 300: Scaling offset (00000000)
      // 301: Starting angle (FFF92230)
      // 302: Angular step width (2710)
      // 303: Number of data (10F)
      // 304..574: Data_1 .. Data_n

      // TODO: msg.intensities

      // 575: Position (0)
      // 576: Name (0)
      // 577: Comment (0)
      // 578: Time information (0)
      // 579: Event information (0)
      // <ETX> (\x03)

      // TODO: read data, fill msg

      pub.publish(msg);

      ros::spinOnce(); // do we need this?


    }

    /*
     * Stop streaming measurements
     */
    const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
    result = sendSOPASCommand(device_handle, requestScanData0, 500);
    if (result != 0)
    {
      fprintf(stderr, "SOPAS - Error stopping streaming.\n");
    }

    /*
     * Release the interface
     */
    result = libusb_release_interface(device_handle, 0);
    if (result != 0)
    {
      fprintf(stderr, "LIBUSB - Cannot Release Interface\n");
    }
    printf("LIBUSB - Released Interface\n");

    /*
     * Close the device handle.
     */
    libusb_close(device_handle);
  }

  /*
   * Free the list of the USB devices.
   */
  freeSOPASDeviceList(devices);

  /*
   * Close the LIBUSB session.
   */
  libusb_exit(ctx);

  printf("\nSICK AG - TIM3xx - Communication example finished.");

  return EXIT_SUCCESS;
}
