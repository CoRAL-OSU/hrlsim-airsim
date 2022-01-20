// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hrlsim_interfaces:msg/Sensors.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_HPP_
#define HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'imu'
#include "sensor_msgs/msg/detail/imu__struct.hpp"
// Member 'gps'
#include "sensor_msgs/msg/detail/nav_sat_fix__struct.hpp"
// Member 'altimeter'
#include "airsim_interfaces/msg/detail/altimeter__struct.hpp"
// Member 'magnetometer'
#include "sensor_msgs/msg/detail/magnetic_field__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__msg__Sensors __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__msg__Sensors __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sensors_
{
  using Type = Sensors_<ContainerAllocator>;

  explicit Sensors_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu(_init),
    gps(_init),
    altimeter(_init),
    magnetometer(_init)
  {
    (void)_init;
  }

  explicit Sensors_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : imu(_alloc, _init),
    gps(_alloc, _init),
    altimeter(_alloc, _init),
    magnetometer(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _imu_type =
    sensor_msgs::msg::Imu_<ContainerAllocator>;
  _imu_type imu;
  using _gps_type =
    sensor_msgs::msg::NavSatFix_<ContainerAllocator>;
  _gps_type gps;
  using _altimeter_type =
    airsim_interfaces::msg::Altimeter_<ContainerAllocator>;
  _altimeter_type altimeter;
  using _magnetometer_type =
    sensor_msgs::msg::MagneticField_<ContainerAllocator>;
  _magnetometer_type magnetometer;

  // setters for named parameter idiom
  Type & set__imu(
    const sensor_msgs::msg::Imu_<ContainerAllocator> & _arg)
  {
    this->imu = _arg;
    return *this;
  }
  Type & set__gps(
    const sensor_msgs::msg::NavSatFix_<ContainerAllocator> & _arg)
  {
    this->gps = _arg;
    return *this;
  }
  Type & set__altimeter(
    const airsim_interfaces::msg::Altimeter_<ContainerAllocator> & _arg)
  {
    this->altimeter = _arg;
    return *this;
  }
  Type & set__magnetometer(
    const sensor_msgs::msg::MagneticField_<ContainerAllocator> & _arg)
  {
    this->magnetometer = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::msg::Sensors_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::msg::Sensors_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::Sensors_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::msg::Sensors_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__msg__Sensors
    std::shared_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__msg__Sensors
    std::shared_ptr<hrlsim_interfaces::msg::Sensors_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sensors_ & other) const
  {
    if (this->imu != other.imu) {
      return false;
    }
    if (this->gps != other.gps) {
      return false;
    }
    if (this->altimeter != other.altimeter) {
      return false;
    }
    if (this->magnetometer != other.magnetometer) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sensors_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sensors_

// alias to use template instance with default allocator
using Sensors =
  hrlsim_interfaces::msg::Sensors_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__SENSORS__STRUCT_HPP_
