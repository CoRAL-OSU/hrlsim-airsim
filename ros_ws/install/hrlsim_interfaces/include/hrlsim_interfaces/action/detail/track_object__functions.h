// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hrlsim_interfaces:action/TrackObject.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__FUNCTIONS_H_
#define HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hrlsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hrlsim_interfaces/action/detail/track_object__struct.h"

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_Goal
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_Goal__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Goal__init(hrlsim_interfaces__action__TrackObject_Goal * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Goal__fini(hrlsim_interfaces__action__TrackObject_Goal * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_Goal__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Goal *
hrlsim_interfaces__action__TrackObject_Goal__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Goal__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Goal__destroy(hrlsim_interfaces__action__TrackObject_Goal * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_Goal__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Goal__Sequence__init(hrlsim_interfaces__action__TrackObject_Goal__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Goal__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Goal__Sequence__fini(hrlsim_interfaces__action__TrackObject_Goal__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_Goal__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Goal__Sequence *
hrlsim_interfaces__action__TrackObject_Goal__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Goal__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Goal__Sequence__destroy(hrlsim_interfaces__action__TrackObject_Goal__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_Result
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_Result__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Result__init(hrlsim_interfaces__action__TrackObject_Result * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Result__fini(hrlsim_interfaces__action__TrackObject_Result * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_Result__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Result *
hrlsim_interfaces__action__TrackObject_Result__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Result__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Result__destroy(hrlsim_interfaces__action__TrackObject_Result * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_Result__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Result__Sequence__init(hrlsim_interfaces__action__TrackObject_Result__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Result__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Result__Sequence__fini(hrlsim_interfaces__action__TrackObject_Result__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_Result__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Result__Sequence *
hrlsim_interfaces__action__TrackObject_Result__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Result__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Result__Sequence__destroy(hrlsim_interfaces__action__TrackObject_Result__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_Feedback
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_Feedback__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Feedback__init(hrlsim_interfaces__action__TrackObject_Feedback * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Feedback__fini(hrlsim_interfaces__action__TrackObject_Feedback * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_Feedback__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Feedback *
hrlsim_interfaces__action__TrackObject_Feedback__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Feedback__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Feedback__destroy(hrlsim_interfaces__action__TrackObject_Feedback * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_Feedback__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_Feedback__Sequence__init(hrlsim_interfaces__action__TrackObject_Feedback__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Feedback__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Feedback__Sequence__fini(hrlsim_interfaces__action__TrackObject_Feedback__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_Feedback__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_Feedback__Sequence *
hrlsim_interfaces__action__TrackObject_Feedback__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_Feedback__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_Feedback__Sequence__destroy(hrlsim_interfaces__action__TrackObject_Feedback__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_SendGoal_Request__init(hrlsim_interfaces__action__TrackObject_SendGoal_Request * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Request__fini(hrlsim_interfaces__action__TrackObject_SendGoal_Request * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_SendGoal_Request *
hrlsim_interfaces__action__TrackObject_SendGoal_Request__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Request__destroy(hrlsim_interfaces__action__TrackObject_SendGoal_Request * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__init(hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__fini(hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence *
hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence__destroy(hrlsim_interfaces__action__TrackObject_SendGoal_Request__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_SendGoal_Response__init(hrlsim_interfaces__action__TrackObject_SendGoal_Response * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Response__fini(hrlsim_interfaces__action__TrackObject_SendGoal_Response * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_SendGoal_Response *
hrlsim_interfaces__action__TrackObject_SendGoal_Response__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Response__destroy(hrlsim_interfaces__action__TrackObject_SendGoal_Response * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__init(hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__fini(hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence *
hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence__destroy(hrlsim_interfaces__action__TrackObject_SendGoal_Response__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_GetResult_Request
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_GetResult_Request__init(hrlsim_interfaces__action__TrackObject_GetResult_Request * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Request__fini(hrlsim_interfaces__action__TrackObject_GetResult_Request * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_GetResult_Request *
hrlsim_interfaces__action__TrackObject_GetResult_Request__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Request__destroy(hrlsim_interfaces__action__TrackObject_GetResult_Request * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__init(hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__fini(hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence *
hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence__destroy(hrlsim_interfaces__action__TrackObject_GetResult_Request__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_GetResult_Response
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_GetResult_Response__init(hrlsim_interfaces__action__TrackObject_GetResult_Response * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Response__fini(hrlsim_interfaces__action__TrackObject_GetResult_Response * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_GetResult_Response *
hrlsim_interfaces__action__TrackObject_GetResult_Response__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Response__destroy(hrlsim_interfaces__action__TrackObject_GetResult_Response * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__init(hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__fini(hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence *
hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence__destroy(hrlsim_interfaces__action__TrackObject_GetResult_Response__Sequence * array);

/// Initialize action/TrackObject message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage
 * )) before or use
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_FeedbackMessage__init(hrlsim_interfaces__action__TrackObject_FeedbackMessage * msg);

/// Finalize action/TrackObject message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_FeedbackMessage__fini(hrlsim_interfaces__action__TrackObject_FeedbackMessage * msg);

/// Create action/TrackObject message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_FeedbackMessage *
hrlsim_interfaces__action__TrackObject_FeedbackMessage__create();

/// Destroy action/TrackObject message.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_FeedbackMessage__destroy(hrlsim_interfaces__action__TrackObject_FeedbackMessage * msg);


/// Initialize array of action/TrackObject messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__init(hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence * array, size_t size);

/// Finalize array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__fini(hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence * array);

/// Create array of action/TrackObject messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence *
hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__create(size_t size);

/// Destroy array of action/TrackObject messages.
/**
 * It calls
 * hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence__destroy(hrlsim_interfaces__action__TrackObject_FeedbackMessage__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__ACTION__DETAIL__TRACK_OBJECT__FUNCTIONS_H_
