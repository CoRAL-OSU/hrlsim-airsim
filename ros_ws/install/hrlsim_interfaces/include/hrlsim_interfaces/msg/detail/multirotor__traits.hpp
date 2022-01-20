// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__TRAITS_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__TRAITS_HPP_

#include "hrlsim_interfaces/msg/detail/multirotor__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'state'
#include "hrlsim_interfaces/msg/detail/state__traits.hpp"
// Member 'odom'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'looptime'
#include "std_msgs/msg/detail/float32__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::msg::Multirotor>()
{
  return "hrlsim_interfaces::msg::Multirotor";
}

template<>
inline const char * name<hrlsim_interfaces::msg::Multirotor>()
{
  return "hrlsim_interfaces/msg/Multirotor";
}

template<>
struct has_fixed_size<hrlsim_interfaces::msg::Multirotor>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<hrlsim_interfaces::msg::State>::value && has_fixed_size<std_msgs::msg::Float32>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::msg::Multirotor>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<hrlsim_interfaces::msg::State>::value && has_bounded_size<std_msgs::msg::Float32>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<hrlsim_interfaces::msg::Multirotor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__TRAITS_HPP_
