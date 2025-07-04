// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dracon_msgs:msg/JobCostTable.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dracon_msgs/msg/detail/job_cost_table__struct.hpp"
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

void JobCostTable_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dracon_msgs::msg::JobCostTable(_init);
}

void JobCostTable_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dracon_msgs::msg::JobCostTable *>(message_memory);
  typed_message->~JobCostTable();
}

size_t size_function__JobCostTable__costs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dracon_msgs::msg::JobCost> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JobCostTable__costs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dracon_msgs::msg::JobCost> *>(untyped_member);
  return &member[index];
}

void * get_function__JobCostTable__costs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dracon_msgs::msg::JobCost> *>(untyped_member);
  return &member[index];
}

void fetch_function__JobCostTable__costs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dracon_msgs::msg::JobCost *>(
    get_const_function__JobCostTable__costs(untyped_member, index));
  auto & value = *reinterpret_cast<dracon_msgs::msg::JobCost *>(untyped_value);
  value = item;
}

void assign_function__JobCostTable__costs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dracon_msgs::msg::JobCost *>(
    get_function__JobCostTable__costs(untyped_member, index));
  const auto & value = *reinterpret_cast<const dracon_msgs::msg::JobCost *>(untyped_value);
  item = value;
}

void resize_function__JobCostTable__costs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dracon_msgs::msg::JobCost> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JobCostTable_message_member_array[1] = {
  {
    "costs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dracon_msgs::msg::JobCost>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs::msg::JobCostTable, costs),  // bytes offset in struct
    nullptr,  // default value
    size_function__JobCostTable__costs,  // size() function pointer
    get_const_function__JobCostTable__costs,  // get_const(index) function pointer
    get_function__JobCostTable__costs,  // get(index) function pointer
    fetch_function__JobCostTable__costs,  // fetch(index, &value) function pointer
    assign_function__JobCostTable__costs,  // assign(index, value) function pointer
    resize_function__JobCostTable__costs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JobCostTable_message_members = {
  "dracon_msgs::msg",  // message namespace
  "JobCostTable",  // message name
  1,  // number of fields
  sizeof(dracon_msgs::msg::JobCostTable),
  JobCostTable_message_member_array,  // message members
  JobCostTable_init_function,  // function to initialize message memory (memory has to be allocated)
  JobCostTable_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JobCostTable_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JobCostTable_message_members,
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
get_message_type_support_handle<dracon_msgs::msg::JobCostTable>()
{
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobCostTable_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dracon_msgs, msg, JobCostTable)() {
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobCostTable_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
