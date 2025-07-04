// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dracon_msgs:msg/JobCost.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_COST__STRUCT_H_
#define DRACON_MSGS__MSG__DETAIL__JOB_COST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'agent_id'
// Member 'job_id'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/JobCost in the package dracon_msgs.
typedef struct dracon_msgs__msg__JobCost
{
  rosidl_runtime_c__String agent_id;
  rosidl_runtime_c__String job_id;
  double cost;
} dracon_msgs__msg__JobCost;

// Struct for a sequence of dracon_msgs__msg__JobCost.
typedef struct dracon_msgs__msg__JobCost__Sequence
{
  dracon_msgs__msg__JobCost * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dracon_msgs__msg__JobCost__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_COST__STRUCT_H_
