// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dracon_msgs:msg/Job.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dracon_msgs/msg/detail/job__rosidl_typesupport_introspection_c.h"
#include "dracon_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dracon_msgs/msg/detail/job__functions.h"
#include "dracon_msgs/msg/detail/job__struct.h"


// Include directives for member types
// Member `job_id`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dracon_msgs__msg__Job__init(message_memory);
}

void dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_fini_function(void * message_memory)
{
  dracon_msgs__msg__Job__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_member_array[3] = {
  {
    "job_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs__msg__Job, job_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs__msg__Job, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs__msg__Job, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_members = {
  "dracon_msgs__msg",  // message namespace
  "Job",  // message name
  3,  // number of fields
  sizeof(dracon_msgs__msg__Job),
  dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_member_array,  // message members
  dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_init_function,  // function to initialize message memory (memory has to be allocated)
  dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_type_support_handle = {
  0,
  &dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dracon_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dracon_msgs, msg, Job)() {
  if (!dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_type_support_handle.typesupport_identifier) {
    dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dracon_msgs__msg__Job__rosidl_typesupport_introspection_c__Job_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
