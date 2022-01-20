// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'state'
#include "hrlsim_interfaces/msg/detail/state__struct.hpp"
// Member 'odom'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'looptime'
#include "std_msgs/msg/detail/float32__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__msg__Multirotor __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__msg__Multirotor __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Multirotor_
{
  using Type = Multirotor_<ContainerAllocator>;

  explicit Multirotor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    state(_init),
    odom(_init),
    looptime(_init)
  {
    (void)_init;
  }

  explicit Multirotor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    state(_alloc, _init),
    odom(_alloc, _init),
    looptime(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    hrlsim_interfaces::msg::State_<ContainerAllocator>;
  _state_type state;
  using _odom_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _odom_type odom;
  using _looptime_type =
    std_msgs::msg::Float32_<ContainerAllocator>;
  _looptime_type looptime;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const hrlsim_interfaces::msg::State_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }
  Type & set__odom(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->odom = _arg;
    return *this;
  }
  Type & set__looptime(
    const std_msgs::msg::Float32_<ContainerAllocator> & _arg)
  {
    this->looptime = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__msg__Multirotor
    std::shared_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__msg__Multirotor
    std::shared_ptr<hrlsim_interfaces::msg::Multirotor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Multirotor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    if (this->odom != other.odom) {
      return false;
    }
    if (this->looptime != other.looptime) {
      return false;
    }
    return true;
  }
  bool operator!=(const Multirotor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Multirotor_

// alias to use template instance with default allocator
using Multirotor =
  hrlsim_interfaces::msg::Multirotor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__STRUCT_HPP_
