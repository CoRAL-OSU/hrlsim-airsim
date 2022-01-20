// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_H_
#define HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'imu'
#include "sensor_msgs/msg/detail/imu__struct.h"
// Member 'gps'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.h"
// Member 'altimeter'
#include "airsim_interfaces/msg/detail/altimeter__struct.h"
// Member 'magnetometer'
#include "sensor_msgs/msg/detail/magnetic_field__struct.h"

// Struct defined in msg/Sensors in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__msg__Sensors
{
  sensor_msgs__msg__Imu imu;
  sensor_msgs__msg__NavSatFix gps;
  airsim_interfaces__msg__Altimeter altimeter;
  sensor_msgs__msg__MagneticField magnetometer;
} hrlsim_interfaces__msg__Sensors;

// Struct for a sequence of hrlsim_interfaces__msg__Sensors.
typedef struct hrlsim_interfaces__msg__Sensors__Sequence
{
  hrlsim_interfaces__msg__Sensors * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__msg__Sensors__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_H_
