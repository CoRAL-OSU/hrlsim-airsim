// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hrlsim_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_

#include "hrlsim_interfaces/msg/detail/state__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hrlsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_State_acc
{
public:
  explicit Init_State_acc(::hrlsim_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::msg::State acc(::hrlsim_interfaces::msg::State::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::msg::State msg_;
};

class Init_State_vel
{
public:
  explicit Init_State_vel(::hrlsim_interfaces::msg::State & msg)
  : msg_(msg)
  {}
  Init_State_acc vel(::hrlsim_interfaces::msg::State::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_State_acc(msg_);
  }

private:
  ::hrlsim_interfaces::msg::State msg_;
};

class Init_State_pose
{
public:
  Init_State_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_State_vel pose(::hrlsim_interfaces::msg::State::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_State_vel(msg_);
  }

private:
  ::hrlsim_interfaces::msg::State msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::msg::State>()
{
  return hrlsim_interfaces::msg::builder::Init_State_pose();
}

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__STATE__BUILDER_HPP_
