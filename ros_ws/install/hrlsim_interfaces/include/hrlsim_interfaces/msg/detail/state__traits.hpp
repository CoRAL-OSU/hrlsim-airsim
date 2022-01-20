// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hrlsim_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_

#include "hrlsim_interfaces/msg/detail/state__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__traits.hpp"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::msg::State>()
{
  return "hrlsim_interfaces::msg::State";
}

template<>
inline const char * name<hrlsim_interfaces::msg::State>()
{
  return "hrlsim_interfaces/msg/State";
}

template<>
struct has_fixed_size<hrlsim_interfaces::msg::State>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Accel>::value && has_fixed_size<geometry_msgs::msg::Pose>::value && has_fixed_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::msg::State>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Accel>::value && has_bounded_size<geometry_msgs::msg::Pose>::value && has_bounded_size<geometry_msgs::msg::Twist>::value> {};

template<>
struct is_message<hrlsim_interfaces::msg::State>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__STATE__TRAITS_HPP_
