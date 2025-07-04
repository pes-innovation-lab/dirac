// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dracon_msgs:msg/Job.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB__FUNCTIONS_H_
#define DRACON_MSGS__MSG__DETAIL__JOB__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dracon_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dracon_msgs/msg/detail/job__struct.h"

/// Initialize msg/Job message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dracon_msgs__msg__Job
 * )) before or use
 * dracon_msgs__msg__Job__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__Job__init(dracon_msgs__msg__Job * msg);

/// Finalize msg/Job message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__Job__fini(dracon_msgs__msg__Job * msg);

/// Create msg/Job message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dracon_msgs__msg__Job__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__Job *
dracon_msgs__msg__Job__create();

/// Destroy msg/Job message.
/**
 * It calls
 * dracon_msgs__msg__Job__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__Job__destroy(dracon_msgs__msg__Job * msg);

/// Check for msg/Job message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__Job__are_equal(const dracon_msgs__msg__Job * lhs, const dracon_msgs__msg__Job * rhs);

/// Copy a msg/Job message.
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
dracon_msgs__msg__Job__copy(
  const dracon_msgs__msg__Job * input,
  dracon_msgs__msg__Job * output);

/// Initialize array of msg/Job messages.
/**
 * It allocates the memory for the number of elements and calls
 * dracon_msgs__msg__Job__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__Job__Sequence__init(dracon_msgs__msg__Job__Sequence * array, size_t size);

/// Finalize array of msg/Job messages.
/**
 * It calls
 * dracon_msgs__msg__Job__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__Job__Sequence__fini(dracon_msgs__msg__Job__Sequence * array);

/// Create array of msg/Job messages.
/**
 * It allocates the memory for the array and calls
 * dracon_msgs__msg__Job__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__Job__Sequence *
dracon_msgs__msg__Job__Sequence__create(size_t size);

/// Destroy array of msg/Job messages.
/**
 * It calls
 * dracon_msgs__msg__Job__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__Job__Sequence__destroy(dracon_msgs__msg__Job__Sequence * array);

/// Check for msg/Job message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__Job__Sequence__are_equal(const dracon_msgs__msg__Job__Sequence * lhs, const dracon_msgs__msg__Job__Sequence * rhs);

/// Copy an array of msg/Job messages.
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
dracon_msgs__msg__Job__Sequence__copy(
  const dracon_msgs__msg__Job__Sequence * input,
  dracon_msgs__msg__Job__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DRACON_MSGS__MSG__DETAIL__JOB__FUNCTIONS_H_
