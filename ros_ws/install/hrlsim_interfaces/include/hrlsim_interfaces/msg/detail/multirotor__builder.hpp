// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__BUILDER_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__BUILDER_HPP_

#include "hrlsim_interfaces/msg/detail/multirotor__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hrlsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Multirotor_looptime
{
public:
  explicit Init_Multirotor_looptime(::hrlsim_interfaces::msg::Multirotor & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::msg::Multirotor looptime(::hrlsim_interfaces::msg::Multirotor::_looptime_type arg)
  {
    msg_.looptime = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Multirotor msg_;
};

class Init_Multirotor_odom
{
public:
  explicit Init_Multirotor_odom(::hrlsim_interfaces::msg::Multirotor & msg)
  : msg_(msg)
  {}
  Init_Multirotor_looptime odom(::hrlsim_interfaces::msg::Multirotor::_odom_type arg)
  {
    msg_.odom = std::move(arg);
    return Init_Multirotor_looptime(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Multirotor msg_;
};

class Init_Multirotor_state
{
public:
  explicit Init_Multirotor_state(::hrlsim_interfaces::msg::Multirotor & msg)
  : msg_(msg)
  {}
  Init_Multirotor_odom state(::hrlsim_interfaces::msg::Multirotor::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_Multirotor_odom(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Multirotor msg_;
};

class Init_Multirotor_header
{
public:
  Init_Multirotor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Multirotor_state header(::hrlsim_interfaces::msg::Multirotor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Multirotor_state(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Multirotor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::msg::Multirotor>()
{
  return hrlsim_interfaces::msg::builder::Init_Multirotor_header();
}

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__BUILDER_HPP_
