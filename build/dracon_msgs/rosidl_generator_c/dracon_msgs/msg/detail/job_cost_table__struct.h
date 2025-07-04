// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dracon_msgs:msg/JobCostTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__STRUCT_H_
#define DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'costs'
#include "dracon_msgs/msg/detail/job_cost__struct.h"

/// Struct defined in msg/JobCostTable in the package dracon_msgs.
typedef struct dracon_msgs__msg__JobCostTable
{
  dracon_msgs__msg__JobCost__Sequence costs;
} dracon_msgs__msg__JobCostTable;

// Struct for a sequence of dracon_msgs__msg__JobCostTable.
typedef struct dracon_msgs__msg__JobCostTable__Sequence
{
  dracon_msgs__msg__JobCostTable * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dracon_msgs__msg__JobCostTable__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__STRUCT_H_
