// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__BUILDER_HPP_
#define HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__BUILDER_HPP_

#include "hrlsim_interfaces/action/detail/track_object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_Goal_offset
{
public:
  explicit Init_TrackObject_Goal_offset(::hrlsim_interfaces::action::TrackObject_Goal & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_Goal offset(::hrlsim_interfaces::action::TrackObject_Goal::_offset_type arg)
  {
    msg_.offset = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Goal msg_;
};

class Init_TrackObject_Goal_timeout
{
public:
  explicit Init_TrackObject_Goal_timeout(::hrlsim_interfaces::action::TrackObject_Goal & msg)
  : msg_(msg)
  {}
  Init_TrackObject_Goal_offset timeout(::hrlsim_interfaces::action::TrackObject_Goal::_timeout_type arg)
  {
    msg_.timeout = std::move(arg);
    return Init_TrackObject_Goal_offset(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Goal msg_;
};

class Init_TrackObject_Goal_object_name
{
public:
  Init_TrackObject_Goal_object_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_Goal_timeout object_name(::hrlsim_interfaces::action::TrackObject_Goal::_object_name_type arg)
  {
    msg_.object_name = std::move(arg);
    return Init_TrackObject_Goal_timeout(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_Goal>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_Goal_object_name();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_Result_dist_mag
{
public:
  explicit Init_TrackObject_Result_dist_mag(::hrlsim_interfaces::action::TrackObject_Result & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_Result dist_mag(::hrlsim_interfaces::action::TrackObject_Result::_dist_mag_type arg)
  {
    msg_.dist_mag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Result msg_;
};

class Init_TrackObject_Result_dist
{
public:
  Init_TrackObject_Result_dist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_Result_dist_mag dist(::hrlsim_interfaces::action::TrackObject_Result::_dist_type arg)
  {
    msg_.dist = std::move(arg);
    return Init_TrackObject_Result_dist_mag(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_Result>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_Result_dist();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_Feedback_dist_mag
{
public:
  explicit Init_TrackObject_Feedback_dist_mag(::hrlsim_interfaces::action::TrackObject_Feedback & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_Feedback dist_mag(::hrlsim_interfaces::action::TrackObject_Feedback::_dist_mag_type arg)
  {
    msg_.dist_mag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Feedback msg_;
};

class Init_TrackObject_Feedback_dist
{
public:
  Init_TrackObject_Feedback_dist()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_Feedback_dist_mag dist(::hrlsim_interfaces::action::TrackObject_Feedback::_dist_type arg)
  {
    msg_.dist = std::move(arg);
    return Init_TrackObject_Feedback_dist_mag(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_Feedback>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_Feedback_dist();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_SendGoal_Request_goal
{
public:
  explicit Init_TrackObject_SendGoal_Request_goal(::hrlsim_interfaces::action::TrackObject_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Request goal(::hrlsim_interfaces::action::TrackObject_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Request msg_;
};

class Init_TrackObject_SendGoal_Request_goal_id
{
public:
  Init_TrackObject_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_SendGoal_Request_goal goal_id(::hrlsim_interfaces::action::TrackObject_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TrackObject_SendGoal_Request_goal(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_SendGoal_Request>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_SendGoal_Request_goal_id();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_SendGoal_Response_stamp
{
public:
  explicit Init_TrackObject_SendGoal_Response_stamp(::hrlsim_interfaces::action::TrackObject_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Response stamp(::hrlsim_interfaces::action::TrackObject_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Response msg_;
};

class Init_TrackObject_SendGoal_Response_accepted
{
public:
  Init_TrackObject_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_SendGoal_Response_stamp accepted(::hrlsim_interfaces::action::TrackObject_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_TrackObject_SendGoal_Response_stamp(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_SendGoal_Response>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_SendGoal_Response_accepted();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_GetResult_Request_goal_id
{
public:
  Init_TrackObject_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::hrlsim_interfaces::action::TrackObject_GetResult_Request goal_id(::hrlsim_interfaces::action::TrackObject_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_GetResult_Request>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_GetResult_Request_goal_id();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_GetResult_Response_result
{
public:
  explicit Init_TrackObject_GetResult_Response_result(::hrlsim_interfaces::action::TrackObject_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_GetResult_Response result(::hrlsim_interfaces::action::TrackObject_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_GetResult_Response msg_;
};

class Init_TrackObject_GetResult_Response_status
{
public:
  Init_TrackObject_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_GetResult_Response_result status(::hrlsim_interfaces::action::TrackObject_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_TrackObject_GetResult_Response_result(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_GetResult_Response>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_GetResult_Response_status();
}

}  // namespace hrlsim_interfaces


namespace hrlsim_interfaces
{

namespace action
{

namespace builder
{

class Init_TrackObject_FeedbackMessage_feedback
{
public:
  explicit Init_TrackObject_FeedbackMessage_feedback(::hrlsim_interfaces::action::TrackObject_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::hrlsim_interfaces::action::TrackObject_FeedbackMessage feedback(::hrlsim_interfaces::action::TrackObject_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_FeedbackMessage msg_;
};

class Init_TrackObject_FeedbackMessage_goal_id
{
public:
  Init_TrackObject_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackObject_FeedbackMessage_feedback goal_id(::hrlsim_interfaces::action::TrackObject_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_TrackObject_FeedbackMessage_feedback(msg_);
  }

private:
  ::hrlsim_interfaces::action::TrackObject_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::hrlsim_interfaces::action::TrackObject_FeedbackMessage>()
{
  return hrlsim_interfaces::action::builder::Init_TrackObject_FeedbackMessage_goal_id();
}

}  // namespace hrlsim_interfaces

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__BUILDER_HPP_
