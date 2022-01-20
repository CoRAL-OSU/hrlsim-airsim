// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hrlsim_interfaces:msg/State.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'vel'
#include "geometry_msgs/msg/detail/twist__struct.hpp"
// Member 'acc'
#include "geometry_msgs/msg/detail/accel__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__msg__State __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__msg__State __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct State_
{
  using Type = State_<ContainerAllocator>;

  explicit State_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init),
    vel(_init),
    acc(_init)
  {
    (void)_init;
  }

  explicit State_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init),
    vel(_alloc, _init),
    acc(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _vel_type =
    geometry_msgs::msg::Twist_<ContainerAllocator>;
  _vel_type vel;
  using _acc_type =
    geometry_msgs::msg::Accel_<ContainerAllocator>;
  _acc_type acc;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__vel(
    const geometry_msgs::msg::Twist_<ContainerAllocator> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__acc(
    const geometry_msgs::msg::Accel_<ContainerAllocator> & _arg)
  {
    this->acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::msg::State_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::msg::State_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::State_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::State_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__msg__State
    std::shared_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__msg__State
    std::shared_ptr<hrlsim_interfaces::msg::State_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const State_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const State_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct State_

// alias to use template instance with default allocator
using State =
  hrlsim_interfaces::msg::State_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__STATE__STRUCT_HPP_
