// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hrlsim_interfaces/msg/detail/sensors__rosidl_typesupport_introspection_c.h"
#include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hrlsim_interfaces/msg/detail/sensors__functions.h"
#include "hrlsim_interfaces/msg/detail/sensors__struct.h"


// Include directives for member types
// Member `imu`
#include "sensor_msgs/msg/imu.h"
// Member `imu`
#include "sensor_msgs/msg/detail/imu__rosidl_typesupport_introspection_c.h"
// Member `gps`
#include "sensor_msgs/msg/nav_sat_fix.h"
// Member `gps`
#include "sensor_msgs/msg/detail/nav_sat_fix__rosidl_typesupport_introspection_c.h"
// Member `altimeter`
#include "airsim_interfaces/msg/altimeter.h"
// Member `altimeter`
#include "airsim_interfaces/msg/detail/altimeter__rosidl_typesupport_introspection_c.h"
// Member `magnetometer`
#include "sensor_msgs/msg/magnetic_field.h"
// Member `magnetometer`
#include "sensor_msgs/msg/detail/magnetic_field__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Sensors__rosidl_typesupport_introspection_c__Sensors_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__msg__Sensors__init(message_memory);
}

void Sensors__rosidl_typesupport_introspection_c__Sensors_fini_function(void * message_memory)
{
  hrlsim_interfaces__msg__Sensors__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array[4] = {
  {
    "imu",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Sensors, imu),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Sensors, gps),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altimeter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Sensors, altimeter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "magnetometer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__msg__Sensors, magnetometer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Sensors__rosidl_typesupport_introspection_c__Sensors_message_members = {
  "hrlsim_interfaces__msg",  // message namespace
  "Sensors",  // message name
  4,  // number of fields
  sizeof(hrlsim_interfaces__msg__Sensors),
  Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array,  // message members
  Sensors__rosidl_typesupport_introspection_c__Sensors_init_function,  // function to initialize message memory (memory has to be allocated)
  Sensors__rosidl_typesupport_introspection_c__Sensors_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Sensors__rosidl_typesupport_introspection_c__Sensors_message_type_support_handle = {
  0,
  &Sensors__rosidl_typesupport_introspection_c__Sensors_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, msg, Sensors)() {
  Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Imu)();
  Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, NavSatFix)();
  Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, airsim_interfaces, msg, Altimeter)();
  Sensors__rosidl_typesupport_introspection_c__Sensors_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, MagneticField)();
  if (!Sensors__rosidl_typesupport_introspection_c__Sensors_message_type_support_handle.typesupport_identifier) {
    Sensors__rosidl_typesupport_introspection_c__Sensors_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Sensors__rosidl_typesupport_introspection_c__Sensors_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
