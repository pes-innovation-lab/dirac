// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dracon_msgs:msg/JobCostTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__FUNCTIONS_H_
#define DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dracon_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "dracon_msgs/msg/detail/job_cost_table__struct.h"

/// Initialize msg/JobCostTable message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dracon_msgs__msg__JobCostTable
 * )) before or use
 * dracon_msgs__msg__JobCostTable__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobCostTable__init(dracon_msgs__msg__JobCostTable * msg);

/// Finalize msg/JobCostTable message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobCostTable__fini(dracon_msgs__msg__JobCostTable * msg);

/// Create msg/JobCostTable message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dracon_msgs__msg__JobCostTable__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__JobCostTable *
dracon_msgs__msg__JobCostTable__create();

/// Destroy msg/JobCostTable message.
/**
 * It calls
 * dracon_msgs__msg__JobCostTable__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobCostTable__destroy(dracon_msgs__msg__JobCostTable * msg);

/// Check for msg/JobCostTable message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobCostTable__are_equal(const dracon_msgs__msg__JobCostTable * lhs, const dracon_msgs__msg__JobCostTable * rhs);

/// Copy a msg/JobCostTable message.
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
dracon_msgs__msg__JobCostTable__copy(
  const dracon_msgs__msg__JobCostTable * input,
  dracon_msgs__msg__JobCostTable * output);

/// Initialize array of msg/JobCostTable messages.
/**
 * It allocates the memory for the number of elements and calls
 * dracon_msgs__msg__JobCostTable__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobCostTable__Sequence__init(dracon_msgs__msg__JobCostTable__Sequence * array, size_t size);

/// Finalize array of msg/JobCostTable messages.
/**
 * It calls
 * dracon_msgs__msg__JobCostTable__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobCostTable__Sequence__fini(dracon_msgs__msg__JobCostTable__Sequence * array);

/// Create array of msg/JobCostTable messages.
/**
 * It allocates the memory for the array and calls
 * dracon_msgs__msg__JobCostTable__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
dracon_msgs__msg__JobCostTable__Sequence *
dracon_msgs__msg__JobCostTable__Sequence__create(size_t size);

/// Destroy array of msg/JobCostTable messages.
/**
 * It calls
 * dracon_msgs__msg__JobCostTable__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
void
dracon_msgs__msg__JobCostTable__Sequence__destroy(dracon_msgs__msg__JobCostTable__Sequence * array);

/// Check for msg/JobCostTable message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dracon_msgs
bool
dracon_msgs__msg__JobCostTable__Sequence__are_equal(const dracon_msgs__msg__JobCostTable__Sequence * lhs, const dracon_msgs__msg__JobCostTable__Sequence * rhs);

/// Copy an array of msg/JobCostTable messages.
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
dracon_msgs__msg__JobCostTable__Sequence__copy(
  const dracon_msgs__msg__JobCostTable__Sequence * input,
  dracon_msgs__msg__JobCostTable__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__FUNCTIONS_H_
