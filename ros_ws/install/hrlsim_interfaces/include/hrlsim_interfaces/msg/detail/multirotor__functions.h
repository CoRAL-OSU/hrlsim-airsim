// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hrlsim_interfaces:msg/Multirotor.idl
// generated code does not contain a copyright notice

#ifndef HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__FUNCTIONS_H_
#define HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hrlsim_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "hrlsim_interfaces/msg/detail/multirotor__struct.h"

/// Initialize msg/Multirotor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hrlsim_interfaces__msg__Multirotor
 * )) before or use
 * hrlsim_interfaces__msg__Multirotor__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__msg__Multirotor__init(hrlsim_interfaces__msg__Multirotor * msg);

/// Finalize msg/Multirotor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__msg__Multirotor__fini(hrlsim_interfaces__msg__Multirotor * msg);

/// Create msg/Multirotor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hrlsim_interfaces__msg__Multirotor__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__msg__Multirotor *
hrlsim_interfaces__msg__Multirotor__create();

/// Destroy msg/Multirotor message.
/**
 * It calls
 * hrlsim_interfaces__msg__Multirotor__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__msg__Multirotor__destroy(hrlsim_interfaces__msg__Multirotor * msg);


/// Initialize array of msg/Multirotor messages.
/**
 * It allocates the memory for the number of elements and calls
 * hrlsim_interfaces__msg__Multirotor__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
bool
hrlsim_interfaces__msg__Multirotor__Sequence__init(hrlsim_interfaces__msg__Multirotor__Sequence * array, size_t size);

/// Finalize array of msg/Multirotor messages.
/**
 * It calls
 * hrlsim_interfaces__msg__Multirotor__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__msg__Multirotor__Sequence__fini(hrlsim_interfaces__msg__Multirotor__Sequence * array);

/// Create array of msg/Multirotor messages.
/**
 * It allocates the memory for the array and calls
 * hrlsim_interfaces__msg__Multirotor__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
hrlsim_interfaces__msg__Multirotor__Sequence *
hrlsim_interfaces__msg__Multirotor__Sequence__create(size_t size);

/// Destroy array of msg/Multirotor messages.
/**
 * It calls
 * hrlsim_interfaces__msg__Multirotor__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hrlsim_interfaces
void
hrlsim_interfaces__msg__Multirotor__Sequence__destroy(hrlsim_interfaces__msg__Multirotor__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // HRLSIM_INTERFACES__MSG__DETAIL__MULTIROTOR__FUNCTIONS_H_
