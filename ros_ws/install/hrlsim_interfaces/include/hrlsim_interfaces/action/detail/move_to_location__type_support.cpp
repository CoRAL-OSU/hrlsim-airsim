// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_Goal_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_Goal(_init);
}

void MoveToLocation_Goal_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_Goal *>(message_memory);
  typed_message->~MoveToLocation_Goal();
}

size_t size_function__MoveToLocation_Goal__target(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Goal__target(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Goal__target(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__MoveToLocation_Goal__fvel(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Goal__fvel(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Goal__fvel(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__MoveToLocation_Goal__facc(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Goal__facc(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Goal__facc(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__MoveToLocation_Goal__fjrk(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Goal__fjrk(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Goal__fjrk(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_Goal_message_member_array[10] = {
  {
    "target",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, target),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Goal__target,  // size() function pointer
    get_const_function__MoveToLocation_Goal__target,  // get_const(index) function pointer
    get_function__MoveToLocation_Goal__target,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "timeout",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, timeout),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tolerance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, tolerance),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position_frame",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, position_frame),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fvel",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, fvel),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Goal__fvel,  // size() function pointer
    get_const_function__MoveToLocation_Goal__fvel,  // get_const(index) function pointer
    get_function__MoveToLocation_Goal__fvel,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "facc",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, facc),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Goal__facc,  // size() function pointer
    get_const_function__MoveToLocation_Goal__facc,  // get_const(index) function pointer
    get_function__MoveToLocation_Goal__facc,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "fjrk",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, fjrk),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Goal__fjrk,  // size() function pointer
    get_const_function__MoveToLocation_Goal__fjrk,  // get_const(index) function pointer
    get_function__MoveToLocation_Goal__fjrk,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw_frame",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, yaw_frame),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "yaw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Goal, yaw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_Goal_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_Goal",  // message name
  10,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_Goal),
  MoveToLocation_Goal_message_member_array,  // message members
  MoveToLocation_Goal_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Goal_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_Goal_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_Goal_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Goal>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Goal_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_Goal)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Goal_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_Result_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_Result(_init);
}

void MoveToLocation_Result_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_Result *>(message_memory);
  typed_message->~MoveToLocation_Result();
}

size_t size_function__MoveToLocation_Result__location(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Result__location(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Result__location(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_Result_message_member_array[3] = {
  {
    "location",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Result, location),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Result__location,  // size() function pointer
    get_const_function__MoveToLocation_Result__location,  // get_const(index) function pointer
    get_function__MoveToLocation_Result__location,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Result, error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Result, time_left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_Result_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_Result",  // message name
  3,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_Result),
  MoveToLocation_Result_message_member_array,  // message members
  MoveToLocation_Result_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Result_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_Result_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_Result_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Result>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Result_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_Result)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Result_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_Feedback_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_Feedback(_init);
}

void MoveToLocation_Feedback_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_Feedback *>(message_memory);
  typed_message->~MoveToLocation_Feedback();
}

size_t size_function__MoveToLocation_Feedback__location(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__MoveToLocation_Feedback__location(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__MoveToLocation_Feedback__location(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 3> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_Feedback_message_member_array[3] = {
  {
    "location",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Feedback, location),  // bytes offset in struct
    nullptr,  // default value
    size_function__MoveToLocation_Feedback__location,  // size() function pointer
    get_const_function__MoveToLocation_Feedback__location,  // get_const(index) function pointer
    get_function__MoveToLocation_Feedback__location,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "error",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Feedback, error),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "time_left",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_Feedback, time_left),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_Feedback_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_Feedback",  // message name
  3,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_Feedback),
  MoveToLocation_Feedback_message_member_array,  // message members
  MoveToLocation_Feedback_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_Feedback_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_Feedback_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Feedback>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Feedback_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_Feedback)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_SendGoal_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_SendGoal_Request(_init);
}

void MoveToLocation_SendGoal_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request *>(message_memory);
  typed_message->~MoveToLocation_SendGoal_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_SendGoal_Request_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Goal>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Request, goal),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_SendGoal_Request_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_SendGoal_Request",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Request),
  MoveToLocation_SendGoal_Request_message_member_array,  // message members
  MoveToLocation_SendGoal_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_SendGoal_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_SendGoal_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_SendGoal_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_SendGoal_Request)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_SendGoal_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_SendGoal_Response(_init);
}

void MoveToLocation_SendGoal_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response *>(message_memory);
  typed_message->~MoveToLocation_SendGoal_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_SendGoal_Response_message_member_array[2] = {
  {
    "accepted",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Response, accepted),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "stamp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<builtin_interfaces::msg::Time>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Response, stamp),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_SendGoal_Response_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_SendGoal_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_SendGoal_Response),
  MoveToLocation_SendGoal_Response_message_member_array,  // message members
  MoveToLocation_SendGoal_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_SendGoal_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_SendGoal_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_SendGoal_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_SendGoal_Response)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MoveToLocation_SendGoal_service_members = {
  "hrlsim_interfaces::action",  // service namespace
  "MoveToLocation_SendGoal",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_SendGoal>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MoveToLocation_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_SendGoal_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_SendGoal>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_SendGoal_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_SendGoal)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_SendGoal>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_GetResult_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_GetResult_Request(_init);
}

void MoveToLocation_GetResult_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_GetResult_Request *>(message_memory);
  typed_message->~MoveToLocation_GetResult_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_GetResult_Request_message_member_array[1] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_GetResult_Request, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_GetResult_Request_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_GetResult_Request",  // message name
  1,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_GetResult_Request),
  MoveToLocation_GetResult_Request_message_member_array,  // message members
  MoveToLocation_GetResult_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_GetResult_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_GetResult_Request_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_GetResult_Request>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_GetResult_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_GetResult_Request)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_GetResult_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_GetResult_Response(_init);
}

void MoveToLocation_GetResult_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_GetResult_Response *>(message_memory);
  typed_message->~MoveToLocation_GetResult_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_GetResult_Response_message_member_array[2] = {
  {
    "status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_GetResult_Response, status),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "result",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Result>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_GetResult_Response, result),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_GetResult_Response_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_GetResult_Response",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_GetResult_Response),
  MoveToLocation_GetResult_Response_message_member_array,  // message members
  MoveToLocation_GetResult_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_GetResult_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_GetResult_Response_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_GetResult_Response>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_GetResult_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_GetResult_Response)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers MoveToLocation_GetResult_service_members = {
  "hrlsim_interfaces::action",  // service namespace
  "MoveToLocation_GetResult",  // service name
  // these two fields are initialized below on the first access
  // see get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_GetResult>()
  nullptr,  // request message
  nullptr  // response message
};

static const rosidl_service_type_support_t MoveToLocation_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_GetResult_service_members,
  get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_GetResult>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_GetResult_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure that both the request_members_ and the response_members_ are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hrlsim_interfaces::action::MoveToLocation_GetResult_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::hrlsim_interfaces::action::MoveToLocation_GetResult_Response
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_GetResult)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<hrlsim_interfaces::action::MoveToLocation_GetResult>();
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hrlsim_interfaces
{

namespace action
{

namespace rosidl_typesupport_introspection_cpp
{

void MoveToLocation_FeedbackMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hrlsim_interfaces::action::MoveToLocation_FeedbackMessage(_init);
}

void MoveToLocation_FeedbackMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage *>(message_memory);
  typed_message->~MoveToLocation_FeedbackMessage();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MoveToLocation_FeedbackMessage_message_member_array[2] = {
  {
    "goal_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<unique_identifier_msgs::msg::UUID>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_FeedbackMessage, goal_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "feedback",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_Feedback>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hrlsim_interfaces::action::MoveToLocation_FeedbackMessage, feedback),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MoveToLocation_FeedbackMessage_message_members = {
  "hrlsim_interfaces::action",  // message namespace
  "MoveToLocation_FeedbackMessage",  // message name
  2,  // number of fields
  sizeof(hrlsim_interfaces::action::MoveToLocation_FeedbackMessage),
  MoveToLocation_FeedbackMessage_message_member_array,  // message members
  MoveToLocation_FeedbackMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  MoveToLocation_FeedbackMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MoveToLocation_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MoveToLocation_FeedbackMessage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace action

}  // namespace hrlsim_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage>()
{
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_FeedbackMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hrlsim_interfaces, action, MoveToLocation_FeedbackMessage)() {
  return &::hrlsim_interfaces::action::rosidl_typesupport_introspection_cpp::MoveToLocation_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
