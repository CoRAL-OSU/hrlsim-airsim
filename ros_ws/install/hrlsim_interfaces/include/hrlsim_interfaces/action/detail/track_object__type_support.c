// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
#include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hrlsim_interfaces/action/detail/track_object__functions.h"
#include "hrlsim_interfaces/action/detail/track_object__struct.h"


// Include directives for member types
// Member `object_name`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_Goal__init(message_memory);
}

void TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_Goal__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_member_array[3] = {
  {
    "object_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Goal, object_name),  // bytes offset in struct
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
    offsetof(hrlsim_interfaces__action__TrackObject_Goal, timeout),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Goal, offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_Goal",  // message name
  3,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_Goal),
  TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_member_array,  // message members
  TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_type_support_handle = {
  0,
  &TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Goal)() {
  if (!TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_type_support_handle.typesupport_identifier) {
    TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_Goal__rosidl_typesupport_introspection_c__TrackObject_Goal_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_Result__init(message_memory);
}

void TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_Result__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_member_array[2] = {
  {
    "dist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Result, dist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist_mag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Result, dist_mag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_Result",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_Result),
  TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_member_array,  // message members
  TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_type_support_handle = {
  0,
  &TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Result)() {
  if (!TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_type_support_handle.typesupport_identifier) {
    TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_Result__rosidl_typesupport_introspection_c__TrackObject_Result_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_Feedback__init(message_memory);
}

void TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_Feedback__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_member_array[2] = {
  {
    "dist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Feedback, dist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist_mag",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_Feedback, dist_mag),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_Feedback",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_Feedback),
  TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_member_array,  // message members
  TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_type_support_handle = {
  0,
  &TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Feedback)() {
  if (!TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_type_support_handle.typesupport_identifier) {
    TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_Feedback__rosidl_typesupport_introspection_c__TrackObject_Feedback_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


// Include directives for member types
// Member `goal_id`
#include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
#include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `goal`
#include "hrlsim_interfaces/action/track_object.h"
// Member `goal`
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_SendGoal_Request__init(message_memory);
}

void TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_SendGoal_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_SendGoal_Request, goal_id),  // bytes offset in struct
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
    offsetof(hrlsim_interfaces__action__TrackObject_SendGoal_Request, goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_SendGoal_Request),
  TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_member_array,  // message members
  TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_type_support_handle = {
  0,
  &TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Request)() {
  TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Goal)();
  if (!TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_type_support_handle.typesupport_identifier) {
    TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_SendGoal_Request__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/time.h"
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_SendGoal_Response__init(message_memory);
}

void TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_SendGoal_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_SendGoal_Response, accepted),  // bytes offset in struct
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
    offsetof(hrlsim_interfaces__action__TrackObject_SendGoal_Response, stamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_SendGoal_Response),
  TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_member_array,  // message members
  TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_type_support_handle = {
  0,
  &TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Response)() {
  TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Time)();
  if (!TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_type_support_handle.typesupport_identifier) {
    TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_SendGoal_Response__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_members = {
  "hrlsim_interfaces__action",  // service namespace
  "TrackObject_SendGoal",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Request_message_type_support_handle,
  NULL  // response message
  // hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_Response_message_type_support_handle
};

static rosidl_service_type_support_t hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_type_support_handle = {
  0,
  &hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal)() {
  if (!hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_type_support_handle.typesupport_identifier) {
    hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_SendGoal_Response)()->data;
  }

  return &hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_SendGoal_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


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

void TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_GetResult_Request__init(message_memory);
}

void TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_GetResult_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_GetResult_Request, goal_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_GetResult_Request),
  TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_member_array,  // message members
  TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_type_support_handle = {
  0,
  &TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Request)() {
  TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  if (!TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_type_support_handle.typesupport_identifier) {
    TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_GetResult_Request__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


// Include directives for member types
// Member `result`
// already included above
// #include "hrlsim_interfaces/action/track_object.h"
// Member `result`
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_GetResult_Response__init(message_memory);
}

void TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_GetResult_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_GetResult_Response, status),  // bytes offset in struct
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
    offsetof(hrlsim_interfaces__action__TrackObject_GetResult_Response, result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_GetResult_Response),
  TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_member_array,  // message members
  TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_type_support_handle = {
  0,
  &TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Response)() {
  TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Result)();
  if (!TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_type_support_handle.typesupport_identifier) {
    TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_GetResult_Response__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_members = {
  "hrlsim_interfaces__action",  // service namespace
  "TrackObject_GetResult",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_Request_message_type_support_handle,
  NULL  // response message
  // hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_Response_message_type_support_handle
};

static rosidl_service_type_support_t hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_type_support_handle = {
  0,
  &hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult)() {
  if (!hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_type_support_handle.typesupport_identifier) {
    hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_GetResult_Response)()->data;
  }

  return &hrlsim_interfaces__action__detail__track_object__rosidl_typesupport_introspection_c__TrackObject_GetResult_service_type_support_handle;
}

// already included above
// #include <stddef.h>
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"
// already included above
// #include "hrlsim_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__functions.h"
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"


// Include directives for member types
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/uuid.h"
// Member `goal_id`
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__rosidl_typesupport_introspection_c.h"
// Member `feedback`
// already included above
// #include "hrlsim_interfaces/action/track_object.h"
// Member `feedback`
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hrlsim_interfaces__action__TrackObject_FeedbackMessage__init(message_memory);
}

void TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_fini_function(void * message_memory)
{
  hrlsim_interfaces__action__TrackObject_FeedbackMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces__action__TrackObject_FeedbackMessage, goal_id),  // bytes offset in struct
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
    offsetof(hrlsim_interfaces__action__TrackObject_FeedbackMessage, feedback),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_members = {
  "hrlsim_interfaces__action",  // message namespace
  "TrackObject_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces__action__TrackObject_FeedbackMessage),
  TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_member_array,  // message members
  TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_type_support_handle = {
  0,
  &TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hrlsim_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_FeedbackMessage)() {
  TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, unique_identifier_msgs, msg, UUID)();
  TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hrlsim_interfaces, action, TrackObject_Feedback)();
  if (!TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_type_support_handle.typesupport_identifier) {
    TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TrackObject_FeedbackMessage__rosidl_typesupport_introspection_c__TrackObject_FeedbackMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
