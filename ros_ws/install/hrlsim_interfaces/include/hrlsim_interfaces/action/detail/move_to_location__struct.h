// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hrlsim_interfaces:action/MoveToLocation.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_H_
#define HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LOCAL_FRAME'.
enum
{
  hrlsim_interfaces__action__MoveToLocation_Goal__LOCAL_FRAME = 0
};

/// Constant 'GLOBAL_FRAME'.
enum
{
  hrlsim_interfaces__action__MoveToLocation_Goal__GLOBAL_FRAME = 1
};

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_Goal
{
  float target[3];
  float timeout;
  float tolerance;
  float speed;
  int16_t position_frame;
  float fvel[3];
  float facc[3];
  float fjrk[3];
  int16_t yaw_frame;
  float yaw;
} hrlsim_interfaces__action__MoveToLocation_Goal;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_Goal.
typedef struct hrlsim_interfaces__action__MoveToLocation_Goal__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_Goal__Sequence;


// Constants defined in the message

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_Result
{
  float location[3];
  float error;
  float time_left;
} hrlsim_interfaces__action__MoveToLocation_Result;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_Result.
typedef struct hrlsim_interfaces__action__MoveToLocation_Result__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_Result__Sequence;


// Constants defined in the message

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_Feedback
{
  float location[3];
  float error;
  float time_left;
} hrlsim_interfaces__action__MoveToLocation_Feedback;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_Feedback.
typedef struct hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "hrlsim_interfaces/action/detail/move_to_location__struct.h"

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  hrlsim_interfaces__action__MoveToLocation_Goal goal;
} hrlsim_interfaces__action__MoveToLocation_SendGoal_Request;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_SendGoal_Request.
typedef struct hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} hrlsim_interfaces__action__MoveToLocation_SendGoal_Response;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_SendGoal_Response.
typedef struct hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} hrlsim_interfaces__action__MoveToLocation_GetResult_Request;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_GetResult_Request.
typedef struct hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_GetResult_Response
{
  int8_t status;
  hrlsim_interfaces__action__MoveToLocation_Result result;
} hrlsim_interfaces__action__MoveToLocation_GetResult_Response;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_GetResult_Response.
typedef struct hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "hrlsim_interfaces/action/detail/move_to_location__struct.h"

// Struct defined in action/MoveToLocation in the package hrlsim_interfaces.
typedef struct hrlsim_interfaces__action__MoveToLocation_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  hrlsim_interfaces__action__MoveToLocation_Feedback feedback;
} hrlsim_interfaces__action__MoveToLocation_FeedbackMessage;

// Struct for a sequence of hrlsim_interfaces__action__MoveToLocation_FeedbackMessage.
typedef struct hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence
{
  hrlsim_interfaces__action__MoveToLocation_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hrlsim_interfaces__action__MoveToLocation_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__MOVE_TO_LOCATION__STRUCT_H_
