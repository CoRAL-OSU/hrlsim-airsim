// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hrlsim_interfaces:msg/State.idl
// generated code does not contain a copyright notice
#include "hrlsim_interfaces/msg/detail/state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `vel`
#include "geometry_msgs/msg/detail/twist__functions.h"
// Member `acc`
#include "geometry_msgs/msg/detail/accel__functions.h"

bool
hrlsim_interfaces__msg__State__init(hrlsim_interfaces__msg__State * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    hrlsim_interfaces__msg__State__fini(msg);
    return false;
  }
  // vel
  if (!geometry_msgs__msg__Twist__init(&msg->vel)) {
    hrlsim_interfaces__msg__State__fini(msg);
    return false;
  }
  // acc
  if (!geometry_msgs__msg__Accel__init(&msg->acc)) {
    hrlsim_interfaces__msg__State__fini(msg);
    return false;
  }
  return true;
}

void
hrlsim_interfaces__msg__State__fini(hrlsim_interfaces__msg__State * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // vel
  geometry_msgs__msg__Twist__fini(&msg->vel);
  // acc
  geometry_msgs__msg__Accel__fini(&msg->acc);
}

hrlsim_interfaces__msg__State *
hrlsim_interfaces__msg__State__create()
{
  hrlsim_interfaces__msg__State * msg = (hrlsim_interfaces__msg__State *)malloc(sizeof(hrlsim_interfaces__msg__State));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hrlsim_interfaces__msg__State));
  bool success = hrlsim_interfaces__msg__State__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
hrlsim_interfaces__msg__State__destroy(hrlsim_interfaces__msg__State * msg)
{
  if (msg) {
    hrlsim_interfaces__msg__State__fini(msg);
  }
  free(msg);
}


bool
hrlsim_interfaces__msg__State__Sequence__init(hrlsim_interfaces__msg__State__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  hrlsim_interfaces__msg__State * data = NULL;
  if (size) {
    data = (hrlsim_interfaces__msg__State *)calloc(size, sizeof(hrlsim_interfaces__msg__State));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hrlsim_interfaces__msg__State__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hrlsim_interfaces__msg__State__fini(&data[i - 1]);
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
hrlsim_interfaces__msg__State__Sequence__fini(hrlsim_interfaces__msg__State__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hrlsim_interfaces__msg__State__fini(&array->data[i]);
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

hrlsim_interfaces__msg__State__Sequence *
hrlsim_interfaces__msg__State__Sequence__create(size_t size)
{
  hrlsim_interfaces__msg__State__Sequence * array = (hrlsim_interfaces__msg__State__Sequence *)malloc(sizeof(hrlsim_interfaces__msg__State__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = hrlsim_interfaces__msg__State__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
hrlsim_interfaces__msg__State__Sequence__destroy(hrlsim_interfaces__msg__State__Sequence * array)
{
  if (array) {
    hrlsim_interfaces__msg__State__Sequence__fini(array);
  }
  free(array);
}
