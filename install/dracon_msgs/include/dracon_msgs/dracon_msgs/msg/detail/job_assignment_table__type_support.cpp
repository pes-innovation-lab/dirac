// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dracon_msgs:msg/JobAssignmentTable.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dracon_msgs/msg/detail/job_assignment_table__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dracon_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JobAssignmentTable_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dracon_msgs::msg::JobAssignmentTable(_init);
}

void JobAssignmentTable_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dracon_msgs::msg::JobAssignmentTable *>(message_memory);
  typed_message->~JobAssignmentTable();
}

size_t size_function__JobAssignmentTable__assignments(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dracon_msgs::msg::JobAssignment> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JobAssignmentTable__assignments(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dracon_msgs::msg::JobAssignment> *>(untyped_member);
  return &member[index];
}

void * get_function__JobAssignmentTable__assignments(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dracon_msgs::msg::JobAssignment> *>(untyped_member);
  return &member[index];
}

void fetch_function__JobAssignmentTable__assignments(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dracon_msgs::msg::JobAssignment *>(
    get_const_function__JobAssignmentTable__assignments(untyped_member, index));
  auto & value = *reinterpret_cast<dracon_msgs::msg::JobAssignment *>(untyped_value);
  value = item;
}

void assign_function__JobAssignmentTable__assignments(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dracon_msgs::msg::JobAssignment *>(
    get_function__JobAssignmentTable__assignments(untyped_member, index));
  const auto & value = *reinterpret_cast<const dracon_msgs::msg::JobAssignment *>(untyped_value);
  item = value;
}

void resize_function__JobAssignmentTable__assignments(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dracon_msgs::msg::JobAssignment> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JobAssignmentTable_message_member_array[1] = {
  {
    "assignments",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dracon_msgs::msg::JobAssignment>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs::msg::JobAssignmentTable, assignments),  // bytes offset in struct
    nullptr,  // default value
    size_function__JobAssignmentTable__assignments,  // size() function pointer
    get_const_function__JobAssignmentTable__assignments,  // get_const(index) function pointer
    get_function__JobAssignmentTable__assignments,  // get(index) function pointer
    fetch_function__JobAssignmentTable__assignments,  // fetch(index, &value) function pointer
    assign_function__JobAssignmentTable__assignments,  // assign(index, value) function pointer
    resize_function__JobAssignmentTable__assignments  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JobAssignmentTable_message_members = {
  "dracon_msgs::msg",  // message namespace
  "JobAssignmentTable",  // message name
  1,  // number of fields
  sizeof(dracon_msgs::msg::JobAssignmentTable),
  JobAssignmentTable_message_member_array,  // message members
  JobAssignmentTable_init_function,  // function to initialize message memory (memory has to be allocated)
  JobAssignmentTable_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JobAssignmentTable_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JobAssignmentTable_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dracon_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dracon_msgs::msg::JobAssignmentTable>()
{
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobAssignmentTable_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dracon_msgs, msg, JobAssignmentTable)() {
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobAssignmentTable_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
