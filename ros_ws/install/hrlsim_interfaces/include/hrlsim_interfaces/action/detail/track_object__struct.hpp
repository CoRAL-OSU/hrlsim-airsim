// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_HPP_
#define HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Goal __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Goal __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_Goal_
{
  using Type = TrackObject_Goal_<ContainerAllocator>;

  explicit TrackObject_Goal_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_name = "";
      this->timeout = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->offset.begin(), this->offset.end(), 0.0f);
    }
  }

  explicit TrackObject_Goal_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : object_name(_alloc),
    offset(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->object_name = "";
      this->timeout = 0.0f;
      std::fill<typename std::array<float, 3>::iterator, float>(this->offset.begin(), this->offset.end(), 0.0f);
    }
  }

  // field types and members
  using _object_name_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _object_name_type object_name;
  using _timeout_type =
    float;
  _timeout_type timeout;
  using _offset_type =
    std::array<float, 3>;
  _offset_type offset;

  // setters for named parameter idiom
  Type & set__object_name(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->object_name = _arg;
    return *this;
  }
  Type & set__timeout(
    const float & _arg)
  {
    this->timeout = _arg;
    return *this;
  }
  Type & set__offset(
    const std::array<float, 3> & _arg)
  {
    this->offset = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Goal
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Goal
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_Goal_ & other) const
  {
    if (this->object_name != other.object_name) {
      return false;
    }
    if (this->timeout != other.timeout) {
      return false;
    }
    if (this->offset != other.offset) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_Goal_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_Goal_

// alias to use template instance with default allocator
using TrackObject_Goal =
  hrlsim_interfaces::action::TrackObject_Goal_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Result __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Result __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_Result_
{
  using Type = TrackObject_Result_<ContainerAllocator>;

  explicit TrackObject_Result_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->dist.begin(), this->dist.end(), 0.0f);
      this->dist_mag = 0.0f;
    }
  }

  explicit TrackObject_Result_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dist(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->dist.begin(), this->dist.end(), 0.0f);
      this->dist_mag = 0.0f;
    }
  }

  // field types and members
  using _dist_type =
    std::array<float, 3>;
  _dist_type dist;
  using _dist_mag_type =
    float;
  _dist_mag_type dist_mag;

  // setters for named parameter idiom
  Type & set__dist(
    const std::array<float, 3> & _arg)
  {
    this->dist = _arg;
    return *this;
  }
  Type & set__dist_mag(
    const float & _arg)
  {
    this->dist_mag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Result
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Result
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_Result_ & other) const
  {
    if (this->dist != other.dist) {
      return false;
    }
    if (this->dist_mag != other.dist_mag) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_Result_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_Result_

// alias to use template instance with default allocator
using TrackObject_Result =
  hrlsim_interfaces::action::TrackObject_Result_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Feedback __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_Feedback __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_Feedback_
{
  using Type = TrackObject_Feedback_<ContainerAllocator>;

  explicit TrackObject_Feedback_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->dist.begin(), this->dist.end(), 0.0f);
      this->dist_mag = 0.0f;
    }
  }

  explicit TrackObject_Feedback_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : dist(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->dist.begin(), this->dist.end(), 0.0f);
      this->dist_mag = 0.0f;
    }
  }

  // field types and members
  using _dist_type =
    std::array<float, 3>;
  _dist_type dist;
  using _dist_mag_type =
    float;
  _dist_mag_type dist_mag;

  // setters for named parameter idiom
  Type & set__dist(
    const std::array<float, 3> & _arg)
  {
    this->dist = _arg;
    return *this;
  }
  Type & set__dist_mag(
    const float & _arg)
  {
    this->dist_mag = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Feedback
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_Feedback
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_Feedback_ & other) const
  {
    if (this->dist != other.dist) {
      return false;
    }
    if (this->dist_mag != other.dist_mag) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_Feedback_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_Feedback_

// alias to use template instance with default allocator
using TrackObject_Feedback =
  hrlsim_interfaces::action::TrackObject_Feedback_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'goal'
#include "hrlsim_interfaces/action/detail/track_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Request __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Request __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_SendGoal_Request_
{
  using Type = TrackObject_SendGoal_Request_<ContainerAllocator>;

  explicit TrackObject_SendGoal_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    goal(_init)
  {
    (void)_init;
  }

  explicit TrackObject_SendGoal_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator>;
  _goal_type goal;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__goal(
    const hrlsim_interfaces::action::TrackObject_Goal_<ContainerAllocator> & _arg)
  {
    this->goal = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Request
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Request
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_SendGoal_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->goal != other.goal) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_SendGoal_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_SendGoal_Request_

// alias to use template instance with default allocator
using TrackObject_SendGoal_Request =
  hrlsim_interfaces::action::TrackObject_SendGoal_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Response __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Response __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_SendGoal_Response_
{
  using Type = TrackObject_SendGoal_Response_<ContainerAllocator>;

  explicit TrackObject_SendGoal_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->accepted = false;
    }
  }

  explicit TrackObject_SendGoal_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Response
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_SendGoal_Response
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_SendGoal_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_SendGoal_Response_ & other) const
  {
    if (this->accepted != other.accepted) {
      return false;
    }
    if (this->stamp != other.stamp) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_SendGoal_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_SendGoal_Response_

// alias to use template instance with default allocator
using TrackObject_SendGoal_Response =
  hrlsim_interfaces::action::TrackObject_SendGoal_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces

namespace hrlsim_interfaces
{

namespace action
{

struct TrackObject_SendGoal
{
  using Request = hrlsim_interfaces::action::TrackObject_SendGoal_Request;
  using Response = hrlsim_interfaces::action::TrackObject_SendGoal_Response;
};

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Request __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Request __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_GetResult_Request_
{
  using Type = TrackObject_GetResult_Request_<ContainerAllocator>;

  explicit TrackObject_GetResult_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init)
  {
    (void)_init;
  }

  explicit TrackObject_GetResult_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Request
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Request
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_GetResult_Request_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_GetResult_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_GetResult_Request_

// alias to use template instance with default allocator
using TrackObject_GetResult_Request =
  hrlsim_interfaces::action::TrackObject_GetResult_Request_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'result'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Response __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Response __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_GetResult_Response_
{
  using Type = TrackObject_GetResult_Response_<ContainerAllocator>;

  explicit TrackObject_GetResult_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : result(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0;
    }
  }

  explicit TrackObject_GetResult_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator>;
  _result_type result;

  // setters for named parameter idiom
  Type & set__status(
    const int8_t & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__result(
    const hrlsim_interfaces::action::TrackObject_Result_<ContainerAllocator> & _arg)
  {
    this->result = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Response
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_GetResult_Response
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_GetResult_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_GetResult_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->result != other.result) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_GetResult_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_GetResult_Response_

// alias to use template instance with default allocator
using TrackObject_GetResult_Response =
  hrlsim_interfaces::action::TrackObject_GetResult_Response_<std::allocator<void>>;

// constant definitions

}  // namespace action

}  // namespace hrlsim_interfaces

namespace hrlsim_interfaces
{

namespace action
{

struct TrackObject_GetResult
{
  using Request = hrlsim_interfaces::action::TrackObject_GetResult_Request;
  using Response = hrlsim_interfaces::action::TrackObject_GetResult_Response;
};

}  // namespace action

}  // namespace hrlsim_interfaces


// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.hpp"
// Member 'feedback'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_FeedbackMessage __attribute__((deprecated))
#else
# define DEPRECATED__hrlsim_interfaces__action__TrackObject_FeedbackMessage __declspec(deprecated)
#endif

namespace hrlsim_interfaces
{

namespace action
{

// message struct
template<class ContainerAllocator>
struct TrackObject_FeedbackMessage_
{
  using Type = TrackObject_FeedbackMessage_<ContainerAllocator>;

  explicit TrackObject_FeedbackMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : goal_id(_init),
    feedback(_init)
  {
    (void)_init;
  }

  explicit TrackObject_FeedbackMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator>;
  _feedback_type feedback;

  // setters for named parameter idiom
  Type & set__goal_id(
    const unique_identifier_msgs::msg::UUID_<ContainerAllocator> & _arg)
  {
    this->goal_id = _arg;
    return *this;
  }
  Type & set__feedback(
    const hrlsim_interfaces::action::TrackObject_Feedback_<ContainerAllocator> & _arg)
  {
    this->feedback = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_FeedbackMessage
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__hrlsim_interfaces__action__TrackObject_FeedbackMessage
    std::shared_ptr<hrlsim_interfaces::action::TrackObject_FeedbackMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackObject_FeedbackMessage_ & other) const
  {
    if (this->goal_id != other.goal_id) {
      return false;
    }
    if (this->feedback != other.feedback) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackObject_FeedbackMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackObject_FeedbackMessage_

// alias to use template instance with default allocator
using TrackObject_FeedbackMessage =
  hrlsim_interfaces::action::TrackObject_FeedbackMessage_<std::allocator<void>>;

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

struct TrackObject
{
  /// The goal message defined in the action definition.
  using Goal = hrlsim_interfaces::action::TrackObject_Goal;
  /// The result message defined in the action definition.
  using Result = hrlsim_interfaces::action::TrackObject_Result;
  /// The feedback message defined in the action definition.
  using Feedback = hrlsim_interfaces::action::TrackObject_Feedback;

  struct Impl
  {
    /// The send_goal service using a wrapped version of the goal message as a request.
    using SendGoalService = hrlsim_interfaces::action::TrackObject_SendGoal;
    /// The get_result service using a wrapped version of the result message as a response.
    using GetResultService = hrlsim_interfaces::action::TrackObject_GetResult;
    /// The feedback message with generic fields which wraps the feedback message.
    using FeedbackMessage = hrlsim_interfaces::action::TrackObject_FeedbackMessage;

    /// The generic service to cancel a goal.
    using CancelGoalService = action_msgs::srv::CancelGoal;
    /// The generic message for the status of a goal.
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
  };
};

typedef struct TrackObject TrackObject;

}  // namespace action

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_HPP_
