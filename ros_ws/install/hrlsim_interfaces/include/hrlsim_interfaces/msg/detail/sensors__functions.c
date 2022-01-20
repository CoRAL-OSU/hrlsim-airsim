// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice
#include "hrlsim_interfaces/msg/detail/sensors__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `imu`
#include "sensor_msgs/msg/detail/imu__functions.h"
// Member `gps`
#include "sensor_msgs/msg/detail/nav_sat_fix__functions.h"
// Member `altimeter`
#include "airsim_interfaces/msg/detail/altimeter__functions.h"
// Member `magnetometer`
#include "sensor_msgs/msg/detail/magnetic_field__functions.h"

bool
hrlsim_interfaces__msg__Sensors__init(hrlsim_interfaces__msg__Sensors * msg)
{
  if (!msg) {
    return false;
  }
  // imu
  if (!sensor_msgs__msg__Imu__init(&msg->imu)) {
    hrlsim_interfaces__msg__Sensors__fini(msg);
    return false;
  }
  // gps
  if (!sensor_msgs__msg__NavSatFix__init(&msg->gps)) {
    hrlsim_interfaces__msg__Sensors__fini(msg);
    return false;
  }
  // altimeter
  if (!airsim_interfaces__msg__Altimeter__init(&msg->altimeter)) {
    hrlsim_interfaces__msg__Sensors__fini(msg);
    return false;
  }
  // magnetometer
  if (!sensor_msgs__msg__MagneticField__init(&msg->magnetometer)) {
    hrlsim_interfaces__msg__Sensors__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__msg__Sensors__fini(hrlsim_interfaces__msg__Sensors * msg)
{
  if (!msg) {
    return;
  }
  // imu
  sensor_msgs__msg__Imu__fini(&msg->imu);
  // gps
  sensor_msgs__msg__NavSatFix__fini(&msg->gps);
  // altimeter
  airsim_interfaces__msg__Altimeter__fini(&msg->altimeter);
  // magnetometer
  sensor_msgs__msg__MagneticField__fini(&msg->magnetometer);
}

hrlsim_interfaces__msg__Sensors *
hrlsim_interfaces__msg__Sensors__create()
{
  hrlsim_interfaces__msg__Sensors * msg = (hrlsim_interfaces__msg__Sensors *)malloc(sizeof(hrlsim_interfaces__msg__Sensors));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__msg__Sensors));
  bool success = hrlsim_interfaces__msg__Sensors__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__msg__Sensors__destroy(hrlsim_interfaces__msg__Sensors * msg)
{
  if (msg) {
    hrlsim_interfaces__msg__Sensors__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__msg__Sensors__Sequence__init(hrlsim_interfaces__msg__Sensors__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__msg__Sensors * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__msg__Sensors *)calloc(size, sizeof(hrlsim_interfaces__msg__Sensors));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__msg__Sensors__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__msg__Sensors__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hrlsim_interfaces__msg__Sensors__Sequence__fini(hrlsim_interfaces__msg__Sensors__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__msg__Sensors__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hrlsim_interfaces__msg__Sensors__Sequence *
hrlsim_interfaces__msg__Sensors__Sequence__create(size_t size)
{
  hrlsim_interfaces__msg__Sensors__Sequence * array = (hrlsim_interfaces__msg__Sensors__Sequence *)malloc(sizeof(hrlsim_interfaces__msg__Sensors__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__msg__Sensors__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__msg__Sensors__Sequence__destroy(hrlsim_interfaces__msg__Sensors__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__msg__Sensors__Sequence__fini(array);
  }
  free(array);
}
