// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_H_
#define HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'object_name'
#include "rosidl_runtime_c/string.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_Goal
{
  rosidl_runtime_c__String object_name;
  float timeout;
  float offset[3];
} hrlsim_interfaces__action__TrackObject_Goal;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_Goal.
typedef struct hrlsim_interfaces__action__TrackObject_Goal__Sequence
{
  hrlsim_interfaces__action__TrackObject_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_Result
{
  float dist[3];
  float dist_mag;
} hrlsim_interfaces__action__TrackObject_Result;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_Result.
typedef struct hrlsim_interfaces__action__TrackObject_Result__Sequence
{
  hrlsim_interfaces__action__TrackObject_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_Result__Sequence;


// Constants defined in the message

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_Feedback
{
  float dist[3];
  float dist_mag;
} hrlsim_interfaces__action__TrackObject_Feedback;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_Feedback.
typedef struct hrlsim_interfaces__action__TrackObject_Feedback__Sequence
{
  hrlsim_interfaces__action__TrackObject_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "hrlsim_interfaces/action/detail/track_object__struct.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  hrlsim_interfaces__action__TrackObject_Goal goal;
} hrlsim_interfaces__action__TrackObject_SendGoal_Request;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_SendGoal_Request.
typedef struct hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence
{
  hrlsim_interfaces__action__TrackObject_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} hrlsim_interfaces__action__TrackObject_SendGoal_Response;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_SendGoal_Response.
typedef struct hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence
{
  hrlsim_interfaces__action__TrackObject_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} hrlsim_interfaces__action__TrackObject_GetResult_Request;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_GetResult_Request.
typedef struct hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence
{
  hrlsim_interfaces__action__TrackObject_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_GetResult_Response
{
  int8_t status;
  hrlsim_interfaces__action__TrackObject_Result result;
} hrlsim_interfaces__action__TrackObject_GetResult_Response;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_GetResult_Response.
typedef struct hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence
{
  hrlsim_interfaces__action__TrackObject_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "hrlsim_interfaces/action/detail/track_object__struct.h"

// Struct defined in action/TrackObject in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__TrackObject_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  hrlsim_interfaces__action__TrackObject_Feedback feedback;
} hrlsim_interfaces__action__TrackObject_FeedbackMessage;

// Struct for a sequence of hrlsim_interfaces__action__TrackObject_FeedbackMessage.
typedef struct hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence
{
  hrlsim_interfaces__action__TrackObject_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__STRUCT_H_
