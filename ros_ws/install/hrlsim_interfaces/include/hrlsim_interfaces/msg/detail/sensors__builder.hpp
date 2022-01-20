// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__BUILDER_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__BUILDER_HPP_

#include "hrlsim_interfaces/msg/detail/sensors__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hrlsim_interfaces
{

namespace msg
{

namespace builder
{

class Init_Sensors_magnetometer
{
public:
  explicit Init_Sensors_magnetometer(::hrlsim_interfaces::msg::Sensors & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::msg::Sensors magnetometer(::hrlsim_interfaces::msg::Sensors::_magnetometer_type arg)
  {
    msg_.magnetometer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Sensors msg_;
};

class Init_Sensors_altimeter
{
public:
  explicit Init_Sensors_altimeter(::hrlsim_interfaces::msg::Sensors & msg)
  : msg_(msg)
  {}
  Init_Sensors_magnetometer altimeter(::hrlsim_interfaces::msg::Sensors::_altimeter_type arg)
  {
    msg_.altimeter = std::move(arg);
    return Init_Sensors_magnetometer(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Sensors msg_;
};

class Init_Sensors_gps
{
public:
  explicit Init_Sensors_gps(::hrlsim_interfaces::msg::Sensors & msg)
  : msg_(msg)
  {}
  Init_Sensors_altimeter gps(::hrlsim_interfaces::msg::Sensors::_gps_type arg)
  {
    msg_.gps = std::move(arg);
    return Init_Sensors_altimeter(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Sensors msg_;
};

class Init_Sensors_imu
{
public:
  Init_Sensors_imu()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sensors_gps imu(::hrlsim_interfaces::msg::Sensors::_imu_type arg)
  {
    msg_.imu = std::move(arg);
    return Init_Sensors_gps(msg_);
  }

private:
  ::hrlsim_interfaces::msg::Sensors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::msg::Sensors>()
{
  return hrlsim_interfaces::msg::builder::Init_Sensors_imu();
}

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__BUILDER_HPP_
