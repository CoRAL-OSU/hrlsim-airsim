// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
#include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
#include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_Goal__init(message_memory);
}

void MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_member_array[10] = {
  {
    "target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "timeout",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tolerance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, tolerance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, position_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fvel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, fvel),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "facc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, facc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fjrk",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, fjrk),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, yaw_frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Goal, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_Goal",  // message name
  10,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_Goal),
  MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_member_array,  // message members
  MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_type_support_handle = {
  0,
  &MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Goal)() {
  if (!MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_Goal__rosidl_typesupport_introspection_c__MoveToLocation_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_Result__init(message_memory);
}

void MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_member_array[3] = {
  {
    "location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Result, location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Result, error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Result, time_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_Result",  // message name
  3,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_Result),
  MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_member_array,  // message members
  MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_type_support_handle = {
  0,
  &MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Result)() {
  if (!MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_Result__rosidl_typesupport_introspection_c__MoveToLocation_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_Feedback__init(message_memory);
}

void MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_member_array[3] = {
  {
    "location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Feedback, location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Feedback, error),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_Feedback, time_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_Feedback",  // message name
  3,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_Feedback),
  MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_member_array,  // message members
  MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_type_support_handle = {
  0,
  &MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Feedback)() {
  if (!MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_Feedback__rosidl_typesupport_introspection_c__MoveToLocation_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "hrlsim_interfaces/action/move_to_location.h"
// Member `goal`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__init(message_memory);
}

void MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Request),
  MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_member_array,  // message members
  MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_type_support_handle = {
  0,
  &MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Request)() {
  MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Goal)();
  if (!MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_SendGoal_Request__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__init(message_memory);
}

void MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response, accepted),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_SendGoal_Response),
  MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_member_array,  // message members
  MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_type_support_handle = {
  0,
  &MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Response)() {
  MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_SendGoal_Response__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_members = {
  "hrlsim_interfaces__action",  // service namespace
  "MoveToLocation_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_type_support_handle = {
  0,
  &hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal)() {
  if (!hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_type_support_handle.typesupport_identifier) {
    hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_SendGoal_Response)()->data;
  }

  return &hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request__init(message_memory);
}

void MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Request),
  MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_member_array,  // message members
  MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_type_support_handle = {
  0,
  &MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Request)() {
  MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_GetResult_Request__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "hrlsim_interfaces/action/move_to_location.h"
// Member `result`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response__init(message_memory);
}

void MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_GetResult_Response),
  MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_member_array,  // message members
  MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_type_support_handle = {
  0,
  &MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Response)() {
  MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Result)();
  if (!MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_GetResult_Response__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_members = {
  "hrlsim_interfaces__action",  // service namespace
  "MoveToLocation_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_type_support_handle = {
  0,
  &hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult)() {
  if (!hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_type_support_handle.typesupport_identifier) {
    hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_GetResult_Response)()->data;
  }

  return &hrlsim_interfaces__action__detail__move_to_location__rosidl_typesupport_introspection_c__MoveToLocation_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "hrlsim_interfaces/action/move_to_location.h"
// Member `feedback`
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__init(message_memory);
}

void MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "feedback",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "MoveToLocation_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__MoveToLocation_FeedbackMessage),
  MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_member_array,  // message members
  MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_type_support_handle = {
  0,
  &MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_FeedbackMessage)() {
  MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, MoveToLocation_Feedback)();
  if (!MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MoveToLocation_FeedbackMessage__rosidl_typesupport_introspection_c__MoveToLocation_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
