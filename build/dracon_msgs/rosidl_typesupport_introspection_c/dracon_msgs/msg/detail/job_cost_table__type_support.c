// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dracon_msgs:msg/JobCostTable.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dracon_msgs/msg/detail/job_cost_table__rosidl_typesupport_introspection_c.h"
#include "dracon_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dracon_msgs/msg/detail/job_cost_table__functions.h"
#include "dracon_msgs/msg/detail/job_cost_table__struct.h"


// Include directives for member types
// Member `costs`
#include "dracon_msgs/msg/job_cost.h"
// Member `costs`
#include "dracon_msgs/msg/detail/job_cost__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dracon_msgs__msg__JobCostTable__init(message_memory);
}

void dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_fini_function(void * message_memory)
{
  dracon_msgs__msg__JobCostTable__fini(message_memory);
}

size_t dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__size_function__JobCostTable__costs(
  const void * untyped_member)
{
  const dracon_msgs__msg__JobCost__Sequence * member =
    (const dracon_msgs__msg__JobCost__Sequence *)(untyped_member);
  return member->size;
}

const void * dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_const_function__JobCostTable__costs(
  const void * untyped_member, size_t index)
{
  const dracon_msgs__msg__JobCost__Sequence * member =
    (const dracon_msgs__msg__JobCost__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_function__JobCostTable__costs(
  void * untyped_member, size_t index)
{
  dracon_msgs__msg__JobCost__Sequence * member =
    (dracon_msgs__msg__JobCost__Sequence *)(untyped_member);
  return &member->data[index];
}

void dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__fetch_function__JobCostTable__costs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dracon_msgs__msg__JobCost * item =
    ((const dracon_msgs__msg__JobCost *)
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_const_function__JobCostTable__costs(untyped_member, index));
  dracon_msgs__msg__JobCost * value =
    (dracon_msgs__msg__JobCost *)(untyped_value);
  *value = *item;
}

void dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__assign_function__JobCostTable__costs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dracon_msgs__msg__JobCost * item =
    ((dracon_msgs__msg__JobCost *)
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_function__JobCostTable__costs(untyped_member, index));
  const dracon_msgs__msg__JobCost * value =
    (const dracon_msgs__msg__JobCost *)(untyped_value);
  *item = *value;
}

bool dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__resize_function__JobCostTable__costs(
  void * untyped_member, size_t size)
{
  dracon_msgs__msg__JobCost__Sequence * member =
    (dracon_msgs__msg__JobCost__Sequence *)(untyped_member);
  dracon_msgs__msg__JobCost__Sequence__fini(member);
  return dracon_msgs__msg__JobCost__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_member_array[1] = {
  {
    "costs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs__msg__JobCostTable, costs),  // bytes offset in struct
    NULL,  // default value
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__size_function__JobCostTable__costs,  // size() function pointer
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_const_function__JobCostTable__costs,  // get_const(index) function pointer
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__get_function__JobCostTable__costs,  // get(index) function pointer
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__fetch_function__JobCostTable__costs,  // fetch(index, &value) function pointer
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__assign_function__JobCostTable__costs,  // assign(index, value) function pointer
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__resize_function__JobCostTable__costs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_members = {
  "dracon_msgs__msg",  // message namespace
  "JobCostTable",  // message name
  1,  // number of fields
  sizeof(dracon_msgs__msg__JobCostTable),
  dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_member_array,  // message members
  dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_init_function,  // function to initialize message memory (memory has to be allocated)
  dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_type_support_handle = {
  0,
  &dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dracon_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dracon_msgs, msg, JobCostTable)() {
  dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dracon_msgs, msg, JobCost)();
  if (!dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_type_support_handle.typesupport_identifier) {
    dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dracon_msgs__msg__JobCostTable__rosidl_typesupport_introspection_c__JobCostTable_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
