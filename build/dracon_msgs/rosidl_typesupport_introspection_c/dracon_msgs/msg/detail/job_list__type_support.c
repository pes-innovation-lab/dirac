// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dracon_msgs:msg/JobList.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dracon_msgs/msg/detail/job_list__rosidl_typesupport_introspection_c.h"
#include "dracon_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dracon_msgs/msg/detail/job_list__functions.h"
#include "dracon_msgs/msg/detail/job_list__struct.h"


// Include directives for member types
// Member `jobs`
#include "dracon_msgs/msg/job.h"
// Member `jobs`
#include "dracon_msgs/msg/detail/job__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dracon_msgs__msg__JobList__init(message_memory);
}

void dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_fini_function(void * message_memory)
{
  dracon_msgs__msg__JobList__fini(message_memory);
}

size_t dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__size_function__JobList__jobs(
  const void * untyped_member)
{
  const dracon_msgs__msg__Job__Sequence * member =
    (const dracon_msgs__msg__Job__Sequence *)(untyped_member);
  return member->size;
}

const void * dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_const_function__JobList__jobs(
  const void * untyped_member, size_t index)
{
  const dracon_msgs__msg__Job__Sequence * member =
    (const dracon_msgs__msg__Job__Sequence *)(untyped_member);
  return &member->data[index];
}

void * dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_function__JobList__jobs(
  void * untyped_member, size_t index)
{
  dracon_msgs__msg__Job__Sequence * member =
    (dracon_msgs__msg__Job__Sequence *)(untyped_member);
  return &member->data[index];
}

void dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__fetch_function__JobList__jobs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const dracon_msgs__msg__Job * item =
    ((const dracon_msgs__msg__Job *)
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_const_function__JobList__jobs(untyped_member, index));
  dracon_msgs__msg__Job * value =
    (dracon_msgs__msg__Job *)(untyped_value);
  *value = *item;
}

void dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__assign_function__JobList__jobs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  dracon_msgs__msg__Job * item =
    ((dracon_msgs__msg__Job *)
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_function__JobList__jobs(untyped_member, index));
  const dracon_msgs__msg__Job * value =
    (const dracon_msgs__msg__Job *)(untyped_value);
  *item = *value;
}

bool dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__resize_function__JobList__jobs(
  void * untyped_member, size_t size)
{
  dracon_msgs__msg__Job__Sequence * member =
    (dracon_msgs__msg__Job__Sequence *)(untyped_member);
  dracon_msgs__msg__Job__Sequence__fini(member);
  return dracon_msgs__msg__Job__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_member_array[1] = {
  {
    "jobs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs__msg__JobList, jobs),  // bytes offset in struct
    NULL,  // default value
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__size_function__JobList__jobs,  // size() function pointer
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_const_function__JobList__jobs,  // get_const(index) function pointer
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__get_function__JobList__jobs,  // get(index) function pointer
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__fetch_function__JobList__jobs,  // fetch(index, &value) function pointer
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__assign_function__JobList__jobs,  // assign(index, value) function pointer
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__resize_function__JobList__jobs  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_members = {
  "dracon_msgs__msg",  // message namespace
  "JobList",  // message name
  1,  // number of fields
  sizeof(dracon_msgs__msg__JobList),
  dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_member_array,  // message members
  dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_init_function,  // function to initialize message memory (memory has to be allocated)
  dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_type_support_handle = {
  0,
  &dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dracon_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dracon_msgs, msg, JobList)() {
  dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dracon_msgs, msg, Job)();
  if (!dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_type_support_handle.typesupport_identifier) {
    dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dracon_msgs__msg__JobList__rosidl_typesupport_introspection_c__JobList_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
