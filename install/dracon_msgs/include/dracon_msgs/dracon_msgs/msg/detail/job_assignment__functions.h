// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dracon_msgs:msg/JobAssignment.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__FUNCTIONS_H_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dracon_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dracon_msgs/msg/detail/job_assignment__struct.h"

/// Initialize msg/JobAssignment message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dracon_msgs__msg__JobAssignment
 * )) before or use
 * dracon_msgs__msg__JobAssignment__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__init(dracon_msgs__msg__JobAssignment * msg);

/// Finalize msg/JobAssignment message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobAssignment__fini(dracon_msgs__msg__JobAssignment * msg);

/// Create msg/JobAssignment message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dracon_msgs__msg__JobAssignment__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__JobAssignment *
dracon_msgs__msg__JobAssignment__create();

/// Destroy msg/JobAssignment message.
/**
 * It calls
 * dracon_msgs__msg__JobAssignment__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobAssignment__destroy(dracon_msgs__msg__JobAssignment * msg);

/// Check for msg/JobAssignment message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__are_equal(const dracon_msgs__msg__JobAssignment * lhs, const dracon_msgs__msg__JobAssignment * rhs);

/// Copy a msg/JobAssignment message.
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
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__copy(
  const dracon_msgs__msg__JobAssignment * input,
  dracon_msgs__msg__JobAssignment * output);

/// Initialize array of msg/JobAssignment messages.
/**
 * It allocates the memory for the number of elements and calls
 * dracon_msgs__msg__JobAssignment__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__Sequence__init(dracon_msgs__msg__JobAssignment__Sequence * array, size_t size);

/// Finalize array of msg/JobAssignment messages.
/**
 * It calls
 * dracon_msgs__msg__JobAssignment__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobAssignment__Sequence__fini(dracon_msgs__msg__JobAssignment__Sequence * array);

/// Create array of msg/JobAssignment messages.
/**
 * It allocates the memory for the array and calls
 * dracon_msgs__msg__JobAssignment__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__JobAssignment__Sequence *
dracon_msgs__msg__JobAssignment__Sequence__create(size_t size);

/// Destroy array of msg/JobAssignment messages.
/**
 * It calls
 * dracon_msgs__msg__JobAssignment__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobAssignment__Sequence__destroy(dracon_msgs__msg__JobAssignment__Sequence * array);

/// Check for msg/JobAssignment message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__Sequence__are_equal(const dracon_msgs__msg__JobAssignment__Sequence * lhs, const dracon_msgs__msg__JobAssignment__Sequence * rhs);

/// Copy an array of msg/JobAssignment messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobAssignment__Sequence__copy(
  const dracon_msgs__msg__JobAssignment__Sequence * input,
  dracon_msgs__msg__JobAssignment__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__FUNCTIONS_H_
