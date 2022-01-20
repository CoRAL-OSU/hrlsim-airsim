// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_HPP_
#define HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Goal __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Goal __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_Goal_
{
  using Type = MoveToLocation_Goal_<ContainerAllocator>;

  explicit MoveToLocation_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->target.begin(), this->target.end(), 0.0f);
      this->timeout = 0.0f;
      this->tolerance = 0.0f;
      this->speed = 0.0f;
      this->position_frame = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->fvel.begin(), this->fvel.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->facc.begin(), this->facc.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->fjrk.begin(), this->fjrk.end(), 0.0f);
      this->yaw_frame = 0;
      this->yaw = 0.0f;
    }
  }

  explicit MoveToLocation_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target(_alloc),
    fvel(_alloc),
    facc(_alloc),
    fjrk(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->target.begin(), this->target.end(), 0.0f);
      this->timeout = 0.0f;
      this->tolerance = 0.0f;
      this->speed = 0.0f;
      this->position_frame = 0;
      std::fill<typename std::array<float, 3>::iterator, float>(this->fvel.begin(), this->fvel.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->facc.begin(), this->facc.end(), 0.0f);
      std::fill<typename std::array<float, 3>::iterator, float>(this->fjrk.begin(), this->fjrk.end(), 0.0f);
      this->yaw_frame = 0;
      this->yaw = 0.0f;
    }
  }

  // field types and members
  using _target_type =
    std::array<float, 3>;
  _target_type target;
  using _timeout_type =
    float;
  _timeout_type timeout;
  using _tolerance_type =
    float;
  _tolerance_type tolerance;
  using _speed_type =
    float;
  _speed_type speed;
  using _position_frame_type =
    int16_t;
  _position_frame_type position_frame;
  using _fvel_type =
    std::array<float, 3>;
  _fvel_type fvel;
  using _facc_type =
    std::array<float, 3>;
  _facc_type facc;
  using _fjrk_type =
    std::array<float, 3>;
  _fjrk_type fjrk;
  using _yaw_frame_type =
    int16_t;
  _yaw_frame_type yaw_frame;
  using _yaw_type =
    float;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__target(
    const std::array<float, 3> & _arg)
  {
    this->target = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__tolerance(
    const float & _arg)
  {
    this->tolerance = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__position_frame(
    const int16_t & _arg)
  {
    this->position_frame = _arg;
    return *this;
  }
  Type & set__fvel(
    const std::array<float, 3> & _arg)
  {
    this->fvel = _arg;
    return *this;
  }
  Type & set__facc(
    const std::array<float, 3> & _arg)
  {
    this->facc = _arg;
    return *this;
  }
  Type & set__fjrk(
    const std::array<float, 3> & _arg)
  {
    this->fjrk = _arg;
    return *this;
  }
  Type & set__yaw_frame(
    const int16_t & _arg)
  {
    this->yaw_frame = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int16_t LOCAL_FRAME =
    0;
  static constexpr int16_t GLOBAL_FRAME =
    1;

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Goal
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Goal
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_Goal_ & other) const
  {
    if (this->target != other.target) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->tolerance != other.tolerance) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->position_frame != other.position_frame) {
      return false;
    }
    if (this->fvel != other.fvel) {
      return false;
    }
    if (this->facc != other.facc) {
      return false;
    }
    if (this->fjrk != other.fjrk) {
      return false;
    }
    if (this->yaw_frame != other.yaw_frame) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_Goal_

// alias to use template instance with default allocator
using MoveToLocation_Goal =
  hrlsim_interfaces::action::MoveToLocation_Goal_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int16_t MoveToLocation_Goal_<ContainerAllocator>::LOCAL_FRAME;
template<typename ContainerAllocator>
constexpr int16_t MoveToLocation_Goal_<ContainerAllocator>::GLOBAL_FRAME;

}  // namespace action

}  // namespace hrlsim_interfaces


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Result __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Result __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_Result_
{
  using Type = MoveToLocation_Result_<ContainerAllocator>;

  explicit MoveToLocation_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->location.begin(), this->location.end(), 0.0f);
      this->error = 0.0f;
      this->time_left = 0.0f;
    }
  }

  explicit MoveToLocation_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : location(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->location.begin(), this->location.end(), 0.0f);
      this->error = 0.0f;
      this->time_left = 0.0f;
    }
  }

  // field types and members
  using _location_type =
    std::array<float, 3>;
  _location_type location;
  using _error_type =
    float;
  _error_type error;
  using _time_left_type =
    float;
  _time_left_type time_left;

  // setters for named parameter idiom
  Type & set__location(
    const std::array<float, 3> & _arg)
  {
    this->location = _arg;
    return *this;
  }
  Type & set__error(
    const float & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__time_left(
    const float & _arg)
  {
    this->time_left = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Result
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Result
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_Result_ & other) const
  {
    if (this->location != other.location) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->time_left != other.time_left) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_Result_

// alias to use template instance with default allocator
using MoveToLocation_Result =
  hrlsim_interfaces::action::MoveToLocation_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Feedback __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_Feedback_
{
  using Type = MoveToLocation_Feedback_<ContainerAllocator>;

  explicit MoveToLocation_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->location.begin(), this->location.end(), 0.0f);
      this->error = 0.0f;
      this->time_left = 0.0f;
    }
  }

  explicit MoveToLocation_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : location(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->location.begin(), this->location.end(), 0.0f);
      this->error = 0.0f;
      this->time_left = 0.0f;
    }
  }

  // field types and members
  using _location_type =
    std::array<float, 3>;
  _location_type location;
  using _error_type =
    float;
  _error_type error;
  using _time_left_type =
    float;
  _time_left_type time_left;

  // setters for named parameter idiom
  Type & set__location(
    const std::array<float, 3> & _arg)
  {
    this->location = _arg;
    return *this;
  }
  Type & set__error(
    const float & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__time_left(
    const float & _arg)
  {
    this->time_left = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Feedback
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_Feedback
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_Feedback_ & other) const
  {
    if (this->location != other.location) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->time_left != other.time_left) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_Feedback_

// alias to use template instance with default allocator
using MoveToLocation_Feedback =
  hrlsim_interfaces::action::MoveToLocation_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Request __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_SendGoal_Request_
{
  using Type = MoveToLocation_SendGoal_Request_<ContainerAllocator>;

  explicit MoveToLocation_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit MoveToLocation_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    goal(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _goal_type =
    hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const hrlsim_interfaces::action::MoveToLocation_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Request
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Request
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_SendGoal_Request_

// alias to use template instance with default allocator
using MoveToLocation_SendGoal_Request =
  hrlsim_interfaces::action::MoveToLocation_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Response __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_SendGoal_Response_
{
  using Type = MoveToLocation_SendGoal_Response_<ContainerAllocator>;

  explicit MoveToLocation_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit MoveToLocation_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  // field types and members
  using _accepted_type =
    bool;
  _accepted_type accepted;
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;

  // setters for named parameter idiom
  Type & set__accepted(
    const bool & _arg)
  {
    this->accepted = _arg;
    return *this;
  }
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Response
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_SendGoal_Response
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_SendGoal_Response_

// alias to use template instance with default allocator
using MoveToLocation_SendGoal_Response =
  hrlsim_interfaces::action::MoveToLocation_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces

namespace hrlsim_interfaces
{

namespace action
{

struct MoveToLocation_SendGoal
{
  using Request = hrlsim_interfaces::action::MoveToLocation_SendGoal_Request;
  using Response = hrlsim_interfaces::action::MoveToLocation_SendGoal_Response;
};

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Request __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_GetResult_Request_
{
  using Type = MoveToLocation_GetResult_Request_<ContainerAllocator>;

  explicit MoveToLocation_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit MoveToLocation_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Request
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Request
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_GetResult_Request_

// alias to use template instance with default allocator
using MoveToLocation_GetResult_Request =
  hrlsim_interfaces::action::MoveToLocation_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Response __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_GetResult_Response_
{
  using Type = MoveToLocation_GetResult_Response_<ContainerAllocator>;

  explicit MoveToLocation_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit MoveToLocation_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  // field types and members
  using _status_type =
    int8_t;
  _status_type status;
  using _result_type =
    hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const hrlsim_interfaces::action::MoveToLocation_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Response
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_GetResult_Response
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_GetResult_Response_

// alias to use template instance with default allocator
using MoveToLocation_GetResult_Response =
  hrlsim_interfaces::action::MoveToLocation_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces

namespace hrlsim_interfaces
{

namespace action
{

struct MoveToLocation_GetResult
{
  using Request = hrlsim_interfaces::action::MoveToLocation_GetResult_Request;
  using Response = hrlsim_interfaces::action::MoveToLocation_GetResult_Response;
};

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__MoveToLocation_FeedbackMessage __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct MoveToLocation_FeedbackMessage_
{
  using Type = MoveToLocation_FeedbackMessage_<ContainerAllocator>;

  explicit MoveToLocation_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit MoveToLocation_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_alloc, _init),
    feedback(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _goal_id_type =
    unique_identifier_msgs::msg::UUID_<ContainerAllocator>;
  _goal_id_type goal_id;
  using _feedback_type =
    hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const hrlsim_interfaces::action::MoveToLocation_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_FeedbackMessage
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__MoveToLocation_FeedbackMessage
    std::shared_ptr<hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveToLocation_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveToLocation_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveToLocation_FeedbackMessage_

// alias to use template instance with default allocator
using MoveToLocation_FeedbackMessage =
  hrlsim_interfaces::action::MoveToLocation_FeedbackMessage_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

namespace hrlsim_interfaces
{

namespace action
{

struct MoveToLocation
{
  /// The goal message defined in the action definition.
  using Goal = hrlsim_interfaces::action::MoveToLocation_Goal;
  /// The result message defined in the action definition.
  using Result = hrlsim_interfaces::action::MoveToLocation_Result;
  /// The feedback message defined in the action definition.
  using Feedback = hrlsim_interfaces::action::MoveToLocation_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = hrlsim_interfaces::action::MoveToLocation_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = hrlsim_interfaces::action::MoveToLocation_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = hrlsim_interfaces::action::MoveToLocation_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct MoveToLocation MoveToLocation;

}  // namespace action

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_HPP_
