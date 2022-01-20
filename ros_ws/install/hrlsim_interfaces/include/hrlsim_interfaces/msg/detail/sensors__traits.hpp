// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__TRAITS_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__TRAITS_HPP_

#include "hrlsim_interfaces/msg/detail/sensors__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'imu'
#include "sensor_msgs/msg/detail/imu__traits.hpp"
// Member 'gps'
#include "sensor_msgs/msg/detail/nav_sat_fix__traits.hpp"
// Member 'altimeter'
#include "airsim_interfaces/msg/detail/altimeter__traits.hpp"
// Member 'magnetometer'
#include "sensor_msgs/msg/detail/magnetic_field__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::msg::Sensors>()
{
  return "hrlsim_interfaces::msg::Sensors";
}

template<>
inline const char * name<hrlsim_interfaces::msg::Sensors>()
{
  return "hrlsim_interfaces/msg/Sensors";
}

template<>
struct has_fixed_size<hrlsim_interfaces::msg::Sensors>
  : std::integral_constant<bool, has_fixed_size<airsim_interfaces::msg::Altimeter>::value && has_fixed_size<sensor_msgs::msg::Imu>::value && has_fixed_size<sensor_msgs::msg::MagneticField>::value && has_fixed_size<sensor_msgs::msg::NavSatFix>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::msg::Sensors>
  : std::integral_constant<bool, has_bounded_size<airsim_interfaces::msg::Altimeter>::value && has_bounded_size<sensor_msgs::msg::Imu>::value && has_bounded_size<sensor_msgs::msg::MagneticField>::value && has_bounded_size<sensor_msgs::msg::NavSatFix>::value> {};

template<>
struct is_message<hrlsim_interfaces::msg::Sensors>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__TRAITS_HPP_
