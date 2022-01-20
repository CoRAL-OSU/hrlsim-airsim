// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__TRAITS_HPP_
#define HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__TRAITS_HPP_

#include "hrlsim_interfaces/action/detail/track_object__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_Goal>()
{
  return "hrlsim_interfaces::action::TrackObject_Goal";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_Goal>()
{
  return "hrlsim_interfaces/action/TrackObject_Goal";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_Goal>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_Goal>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_Result>()
{
  return "hrlsim_interfaces::action::TrackObject_Result";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_Result>()
{
  return "hrlsim_interfaces/action/TrackObject_Result";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_Result>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_Result>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_Result>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_Feedback>()
{
  return "hrlsim_interfaces::action::TrackObject_Feedback";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_Feedback>()
{
  return "hrlsim_interfaces/action/TrackObject_Feedback";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_Feedback>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_Feedback>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'goal'
#include "hrlsim_interfaces/action/detail/track_object__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_SendGoal_Request>()
{
  return "hrlsim_interfaces::action::TrackObject_SendGoal_Request";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_SendGoal_Request>()
{
  return "hrlsim_interfaces/action/TrackObject_SendGoal_Request";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_SendGoal_Request>
  : std::integral_constant<bool, has_fixed_size<hrlsim_interfaces::action::TrackObject_Goal>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_SendGoal_Request>
  : std::integral_constant<bool, has_bounded_size<hrlsim_interfaces::action::TrackObject_Goal>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_SendGoal_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_SendGoal_Response>()
{
  return "hrlsim_interfaces::action::TrackObject_SendGoal_Response";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_SendGoal_Response>()
{
  return "hrlsim_interfaces/action/TrackObject_SendGoal_Response";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_SendGoal_Response>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_SendGoal_Response>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_SendGoal_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_SendGoal>()
{
  return "hrlsim_interfaces::action::TrackObject_SendGoal";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_SendGoal>()
{
  return "hrlsim_interfaces/action/TrackObject_SendGoal";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_SendGoal>
  : std::integral_constant<
    bool,
    has_fixed_size<hrlsim_interfaces::action::TrackObject_SendGoal_Request>::value &&
    has_fixed_size<hrlsim_interfaces::action::TrackObject_SendGoal_Response>::value
  >
{
};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_SendGoal>
  : std::integral_constant<
    bool,
    has_bounded_size<hrlsim_interfaces::action::TrackObject_SendGoal_Request>::value &&
    has_bounded_size<hrlsim_interfaces::action::TrackObject_SendGoal_Response>::value
  >
{
};

template<>
struct is_service<hrlsim_interfaces::action::TrackObject_SendGoal>
  : std::true_type
{
};

template<>
struct is_service_request<hrlsim_interfaces::action::TrackObject_SendGoal_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hrlsim_interfaces::action::TrackObject_SendGoal_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_GetResult_Request>()
{
  return "hrlsim_interfaces::action::TrackObject_GetResult_Request";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_GetResult_Request>()
{
  return "hrlsim_interfaces/action/TrackObject_GetResult_Request";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_GetResult_Request>
  : std::integral_constant<bool, has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_GetResult_Request>
  : std::integral_constant<bool, has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_GetResult_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'result'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_GetResult_Response>()
{
  return "hrlsim_interfaces::action::TrackObject_GetResult_Response";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_GetResult_Response>()
{
  return "hrlsim_interfaces/action/TrackObject_GetResult_Response";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_GetResult_Response>
  : std::integral_constant<bool, has_fixed_size<hrlsim_interfaces::action::TrackObject_Result>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_GetResult_Response>
  : std::integral_constant<bool, has_bounded_size<hrlsim_interfaces::action::TrackObject_Result>::value> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_GetResult_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_GetResult>()
{
  return "hrlsim_interfaces::action::TrackObject_GetResult";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_GetResult>()
{
  return "hrlsim_interfaces/action/TrackObject_GetResult";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_GetResult>
  : std::integral_constant<
    bool,
    has_fixed_size<hrlsim_interfaces::action::TrackObject_GetResult_Request>::value &&
    has_fixed_size<hrlsim_interfaces::action::TrackObject_GetResult_Response>::value
  >
{
};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_GetResult>
  : std::integral_constant<
    bool,
    has_bounded_size<hrlsim_interfaces::action::TrackObject_GetResult_Request>::value &&
    has_bounded_size<hrlsim_interfaces::action::TrackObject_GetResult_Response>::value
  >
{
};

template<>
struct is_service<hrlsim_interfaces::action::TrackObject_GetResult>
  : std::true_type
{
};

template<>
struct is_service_request<hrlsim_interfaces::action::TrackObject_GetResult_Request>
  : std::true_type
{
};

template<>
struct is_service_response<hrlsim_interfaces::action::TrackObject_GetResult_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__traits.hpp"
// Member 'feedback'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<hrlsim_interfaces::action::TrackObject_FeedbackMessage>()
{
  return "hrlsim_interfaces::action::TrackObject_FeedbackMessage";
}

template<>
inline const char * name<hrlsim_interfaces::action::TrackObject_FeedbackMessage>()
{
  return "hrlsim_interfaces/action/TrackObject_FeedbackMessage";
}

template<>
struct has_fixed_size<hrlsim_interfaces::action::TrackObject_FeedbackMessage>
  : std::integral_constant<bool, has_fixed_size<hrlsim_interfaces::action::TrackObject_Feedback>::value && has_fixed_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct has_bounded_size<hrlsim_interfaces::action::TrackObject_FeedbackMessage>
  : std::integral_constant<bool, has_bounded_size<hrlsim_interfaces::action::TrackObject_Feedback>::value && has_bounded_size<unique_identifier_msgs::msg::UUID>::value> {};

template<>
struct is_message<hrlsim_interfaces::action::TrackObject_FeedbackMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits


namespace rosidl_generator_traits
{

template<>
struct is_action<hrlsim_interfaces::action::TrackObject>
  : std::true_type
{
};

template<>
struct is_action_goal<hrlsim_interfaces::action::TrackObject_Goal>
  : std::true_type
{
};

template<>
struct is_action_result<hrlsim_interfaces::action::TrackObject_Result>
  : std::true_type
{
};

template<>
struct is_action_feedback<hrlsim_interfaces::action::TrackObject_Feedback>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits


#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__TRAITS_HPP_
