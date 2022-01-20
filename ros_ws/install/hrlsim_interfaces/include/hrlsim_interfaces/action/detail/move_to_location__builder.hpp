// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__BUILDER_HPP_
#define HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__BUILDER_HPP_

#include "hrlsim_interfaces/action/detail/move_to_location__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_Goal_yaw
{
public:
  explicit Init_MoveToLocation_Goal_yaw(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_Goal yaw(::hrlsim_interfaces::action::MoveToLocation_Goal::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_yaw_frame
{
public:
  explicit Init_MoveToLocation_Goal_yaw_frame(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_yaw yaw_frame(::hrlsim_interfaces::action::MoveToLocation_Goal::_yaw_frame_type arg)
  {
    msg_.yaw_frame = std::move(arg);
    return Init_MoveToLocation_Goal_yaw(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_fjrk
{
public:
  explicit Init_MoveToLocation_Goal_fjrk(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_yaw_frame fjrk(::hrlsim_interfaces::action::MoveToLocation_Goal::_fjrk_type arg)
  {
    msg_.fjrk = std::move(arg);
    return Init_MoveToLocation_Goal_yaw_frame(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_facc
{
public:
  explicit Init_MoveToLocation_Goal_facc(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_fjrk facc(::hrlsim_interfaces::action::MoveToLocation_Goal::_facc_type arg)
  {
    msg_.facc = std::move(arg);
    return Init_MoveToLocation_Goal_fjrk(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_fvel
{
public:
  explicit Init_MoveToLocation_Goal_fvel(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_facc fvel(::hrlsim_interfaces::action::MoveToLocation_Goal::_fvel_type arg)
  {
    msg_.fvel = std::move(arg);
    return Init_MoveToLocation_Goal_facc(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_position_frame
{
public:
  explicit Init_MoveToLocation_Goal_position_frame(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_fvel position_frame(::hrlsim_interfaces::action::MoveToLocation_Goal::_position_frame_type arg)
  {
    msg_.position_frame = std::move(arg);
    return Init_MoveToLocation_Goal_fvel(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_speed
{
public:
  explicit Init_MoveToLocation_Goal_speed(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_position_frame speed(::hrlsim_interfaces::action::MoveToLocation_Goal::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_MoveToLocation_Goal_position_frame(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_tolerance
{
public:
  explicit Init_MoveToLocation_Goal_tolerance(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_speed tolerance(::hrlsim_interfaces::action::MoveToLocation_Goal::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return Init_MoveToLocation_Goal_speed(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_timeout
{
public:
  explicit Init_MoveToLocation_Goal_timeout(::hrlsim_interfaces::action::MoveToLocation_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Goal_tolerance timeout(::hrlsim_interfaces::action::MoveToLocation_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_MoveToLocation_Goal_tolerance(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

class Init_MoveToLocation_Goal_target
{
public:
  Init_MoveToLocation_Goal_target()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_Goal_timeout target(::hrlsim_interfaces::action::MoveToLocation_Goal::_target_type arg)
  {
    msg_.target = std::move(arg);
    return Init_MoveToLocation_Goal_timeout(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_Goal>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_Goal_target();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_Result_time_left
{
public:
  explicit Init_MoveToLocation_Result_time_left(::hrlsim_interfaces::action::MoveToLocation_Result & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_Result time_left(::hrlsim_interfaces::action::MoveToLocation_Result::_time_left_type arg)
  {
    msg_.time_left = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Result msg_;
};

class Init_MoveToLocation_Result_error
{
public:
  explicit Init_MoveToLocation_Result_error(::hrlsim_interfaces::action::MoveToLocation_Result & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Result_time_left error(::hrlsim_interfaces::action::MoveToLocation_Result::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_MoveToLocation_Result_time_left(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Result msg_;
};

class Init_MoveToLocation_Result_location
{
public:
  Init_MoveToLocation_Result_location()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_Result_error location(::hrlsim_interfaces::action::MoveToLocation_Result::_location_type arg)
  {
    msg_.location = std::move(arg);
    return Init_MoveToLocation_Result_error(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_Result>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_Result_location();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_Feedback_time_left
{
public:
  explicit Init_MoveToLocation_Feedback_time_left(::hrlsim_interfaces::action::MoveToLocation_Feedback & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_Feedback time_left(::hrlsim_interfaces::action::MoveToLocation_Feedback::_time_left_type arg)
  {
    msg_.time_left = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Feedback msg_;
};

class Init_MoveToLocation_Feedback_error
{
public:
  explicit Init_MoveToLocation_Feedback_error(::hrlsim_interfaces::action::MoveToLocation_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveToLocation_Feedback_time_left error(::hrlsim_interfaces::action::MoveToLocation_Feedback::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_MoveToLocation_Feedback_time_left(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Feedback msg_;
};

class Init_MoveToLocation_Feedback_location
{
public:
  Init_MoveToLocation_Feedback_location()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_Feedback_error location(::hrlsim_interfaces::action::MoveToLocation_Feedback::_location_type arg)
  {
    msg_.location = std::move(arg);
    return Init_MoveToLocation_Feedback_error(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_Feedback>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_Feedback_location();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_SendGoal_Request_goal
{
public:
  explicit Init_MoveToLocation_SendGoal_Request_goal(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request goal(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request msg_;
};

class Init_MoveToLocation_SendGoal_Request_goal_id
{
public:
  Init_MoveToLocation_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_SendGoal_Request_goal goal_id(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToLocation_SendGoal_Request_goal(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_SendGoal_Request>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_SendGoal_Request_goal_id();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_SendGoal_Response_stamp
{
public:
  explicit Init_MoveToLocation_SendGoal_Response_stamp(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response stamp(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response msg_;
};

class Init_MoveToLocation_SendGoal_Response_accepted
{
public:
  Init_MoveToLocation_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_SendGoal_Response_stamp accepted(::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveToLocation_SendGoal_Response_stamp(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_SendGoal_Response>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_SendGoal_Response_accepted();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_GetResult_Request_goal_id
{
public:
  Init_MoveToLocation_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_GetResult_Request goal_id(::hrlsim_interfaces::action::MoveToLocation_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_GetResult_Request>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_GetResult_Request_goal_id();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_GetResult_Response_result
{
public:
  explicit Init_MoveToLocation_GetResult_Response_result(::hrlsim_interfaces::action::MoveToLocation_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_GetResult_Response result(::hrlsim_interfaces::action::MoveToLocation_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_GetResult_Response msg_;
};

class Init_MoveToLocation_GetResult_Response_status
{
public:
  Init_MoveToLocation_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_GetResult_Response_result status(::hrlsim_interfaces::action::MoveToLocation_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveToLocation_GetResult_Response_result(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_GetResult_Response>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_GetResult_Response_status();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_MoveToLocation_FeedbackMessage_feedback
{
public:
  explicit Init_MoveToLocation_FeedbackMessage_feedback(::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage feedback(::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage msg_;
};

class Init_MoveToLocation_FeedbackMessage_goal_id
{
public:
  Init_MoveToLocation_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveToLocation_FeedbackMessage_feedback goal_id(::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveToLocation_FeedbackMessage_feedback(msg_);
  }

private:
  ::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::MoveToLocation_FeedbackMessage>()
{
  return hrlsim_interfaces::action::builder::Init_MoveToLocation_FeedbackMessage_goal_id();
}

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__BUILDER_HPP_
