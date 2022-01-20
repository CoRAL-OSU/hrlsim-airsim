// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hrlsim_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_
#define HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__struct.h"

// Struct defined in msg/State in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__msg__State
{
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Twist vel;
  geometry_msgs__msg__Accel acc;
} hrlsim_interfaces__msg__State;

// Struct for a sequence of hrlsim_interfaces__msg__State.
typedef struct hrlsim_interfaces__msg__State__Sequence
{
  hrlsim_interfaces__msg__State * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__msg__State__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_H_
