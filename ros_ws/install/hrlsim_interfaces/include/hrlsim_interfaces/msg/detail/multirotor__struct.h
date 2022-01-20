// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_H_
#define HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'state'
#include "hrlsim_interfaces/msg/detail/state__struct.h"
// Member 'odom'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'looptime'
#include "std_msgs/msg/detail/float32__struct.h"

// Struct defined in msg/Multirotor in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__msg__Multirotor
{
  std_msgs__msg__Header header;
  hrlsim_interfaces__msg__State state;
  geometry_msgs__msg__Pose odom;
  std_msgs__msg__Float32 looptime;
} hrlsim_interfaces__msg__Multirotor;

// Struct for a sequence of hrlsim_interfaces__msg__Multirotor.
typedef struct hrlsim_interfaces__msg__Multirotor__Sequence
{
  hrlsim_interfaces__msg__Multirotor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__msg__Multirotor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_H_
