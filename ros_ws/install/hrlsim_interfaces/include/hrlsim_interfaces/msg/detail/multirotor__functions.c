// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice
#include "hrlsim_interfaces/msg/detail/multirotor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `state`
#include "hrlsim_interfaces/msg/detail/state__functions.h"
// Member `odom`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `looptime`
#include "std_msgs/msg/detail/float32__functions.h"

bool
hrlsim_interfaces__msg__Multirotor__init(hrlsim_interfaces__msg__Multirotor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hrlsim_interfaces__msg__Multirotor__fini(msg);
    return false;
  }
  // state
  if (!hrlsim_interfaces__msg__State__init(&msg->state)) {
    hrlsim_interfaces__msg__Multirotor__fini(msg);
    return false;
  }
  // odom
  if (!geometry_msgs__msg__Pose__init(&msg->odom)) {
    hrlsim_interfaces__msg__Multirotor__fini(msg);
    return false;
  }
  // looptime
  if (!std_msgs__msg__Float32__init(&msg->looptime)) {
    hrlsim_interfaces__msg__Multirotor__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__msg__Multirotor__fini(hrlsim_interfaces__msg__Multirotor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // state
  hrlsim_interfaces__msg__State__fini(&msg->state);
  // odom
  geometry_msgs__msg__Pose__fini(&msg->odom);
  // looptime
  std_msgs__msg__Float32__fini(&msg->looptime);
}

hrlsim_interfaces__msg__Multirotor *
hrlsim_interfaces__msg__Multirotor__create()
{
  hrlsim_interfaces__msg__Multirotor * msg = (hrlsim_interfaces__msg__Multirotor *)malloc(sizeof(hrlsim_interfaces__msg__Multirotor));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__msg__Multirotor));
  bool success = hrlsim_interfaces__msg__Multirotor__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__msg__Multirotor__destroy(hrlsim_interfaces__msg__Multirotor * msg)
{
  if (msg) {
    hrlsim_interfaces__msg__Multirotor__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__msg__Multirotor__Sequence__init(hrlsim_interfaces__msg__Multirotor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__msg__Multirotor * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__msg__Multirotor *)calloc(size, sizeof(hrlsim_interfaces__msg__Multirotor));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__msg__Multirotor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__msg__Multirotor__fini(&data[i - 1]);
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
hrlsim_interfaces__msg__Multirotor__Sequence__fini(hrlsim_interfaces__msg__Multirotor__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__msg__Multirotor__fini(&array->data[i]);
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

hrlsim_interfaces__msg__Multirotor__Sequence *
hrlsim_interfaces__msg__Multirotor__Sequence__create(size_t size)
{
  hrlsim_interfaces__msg__Multirotor__Sequence * array = (hrlsim_interfaces__msg__Multirotor__Sequence *)malloc(sizeof(hrlsim_interfaces__msg__Multirotor__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__msg__Multirotor__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__msg__Multirotor__Sequence__destroy(hrlsim_interfaces__msg__Multirotor__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__msg__Multirotor__Sequence__fini(array);
  }
  free(array);
}
