// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from uav_mode_supervisor:msg/SupervisorState.idl
// generated code does not contain a copyright notice

#ifndef UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__FUNCTIONS_H_
#define UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "uav_mode_supervisor/msg/rosidl_generator_c__visibility_control.h"

#include "uav_mode_supervisor/msg/detail/supervisor_state__struct.h"

/// Initialize msg/SupervisorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * uav_mode_supervisor__msg__SupervisorState
 * )) before or use
 * uav_mode_supervisor__msg__SupervisorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__init(uav_mode_supervisor__msg__SupervisorState * msg);

/// Finalize msg/SupervisorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
void
uav_mode_supervisor__msg__SupervisorState__fini(uav_mode_supervisor__msg__SupervisorState * msg);

/// Create msg/SupervisorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * uav_mode_supervisor__msg__SupervisorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
uav_mode_supervisor__msg__SupervisorState *
uav_mode_supervisor__msg__SupervisorState__create();

/// Destroy msg/SupervisorState message.
/**
 * It calls
 * uav_mode_supervisor__msg__SupervisorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
void
uav_mode_supervisor__msg__SupervisorState__destroy(uav_mode_supervisor__msg__SupervisorState * msg);

/// Check for msg/SupervisorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__are_equal(const uav_mode_supervisor__msg__SupervisorState * lhs, const uav_mode_supervisor__msg__SupervisorState * rhs);

/// Copy a msg/SupervisorState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__copy(
  const uav_mode_supervisor__msg__SupervisorState * input,
  uav_mode_supervisor__msg__SupervisorState * output);

/// Initialize array of msg/SupervisorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * uav_mode_supervisor__msg__SupervisorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__Sequence__init(uav_mode_supervisor__msg__SupervisorState__Sequence * array, size_t size);

/// Finalize array of msg/SupervisorState messages.
/**
 * It calls
 * uav_mode_supervisor__msg__SupervisorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
void
uav_mode_supervisor__msg__SupervisorState__Sequence__fini(uav_mode_supervisor__msg__SupervisorState__Sequence * array);

/// Create array of msg/SupervisorState messages.
/**
 * It allocates the memory for the array and calls
 * uav_mode_supervisor__msg__SupervisorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
uav_mode_supervisor__msg__SupervisorState__Sequence *
uav_mode_supervisor__msg__SupervisorState__Sequence__create(size_t size);

/// Destroy array of msg/SupervisorState messages.
/**
 * It calls
 * uav_mode_supervisor__msg__SupervisorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
void
uav_mode_supervisor__msg__SupervisorState__Sequence__destroy(uav_mode_supervisor__msg__SupervisorState__Sequence * array);

/// Check for msg/SupervisorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__Sequence__are_equal(const uav_mode_supervisor__msg__SupervisorState__Sequence * lhs, const uav_mode_supervisor__msg__SupervisorState__Sequence * rhs);

/// Copy an array of msg/SupervisorState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_uav_mode_supervisor
bool
uav_mode_supervisor__msg__SupervisorState__Sequence__copy(
  const uav_mode_supervisor__msg__SupervisorState__Sequence * input,
  uav_mode_supervisor__msg__SupervisorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // UAV_MODE_SUPERVISOR__MSG__DETAIL__SUPERVISOR_STATE__FUNCTIONS_H_
