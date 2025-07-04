// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dracon_msgs:msg/JobList.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dracon_msgs/msg/detail/job_list__struct.hpp"
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

void JobList_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dracon_msgs::msg::JobList(_init);
}

void JobList_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dracon_msgs::msg::JobList *>(message_memory);
  typed_message->~JobList();
}

size_t size_function__JobList__jobs(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<dracon_msgs::msg::Job> *>(untyped_member);
  return member->size();
}

const void * get_const_function__JobList__jobs(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<dracon_msgs::msg::Job> *>(untyped_member);
  return &member[index];
}

void * get_function__JobList__jobs(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<dracon_msgs::msg::Job> *>(untyped_member);
  return &member[index];
}

void fetch_function__JobList__jobs(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const dracon_msgs::msg::Job *>(
    get_const_function__JobList__jobs(untyped_member, index));
  auto & value = *reinterpret_cast<dracon_msgs::msg::Job *>(untyped_value);
  value = item;
}

void assign_function__JobList__jobs(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<dracon_msgs::msg::Job *>(
    get_function__JobList__jobs(untyped_member, index));
  const auto & value = *reinterpret_cast<const dracon_msgs::msg::Job *>(untyped_value);
  item = value;
}

void resize_function__JobList__jobs(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<dracon_msgs::msg::Job> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JobList_message_member_array[1] = {
  {
    "jobs",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<dracon_msgs::msg::Job>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dracon_msgs::msg::JobList, jobs),  // bytes offset in struct
    nullptr,  // default value
    size_function__JobList__jobs,  // size() function pointer
    get_const_function__JobList__jobs,  // get_const(index) function pointer
    get_function__JobList__jobs,  // get(index) function pointer
    fetch_function__JobList__jobs,  // fetch(index, &value) function pointer
    assign_function__JobList__jobs,  // assign(index, value) function pointer
    resize_function__JobList__jobs  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JobList_message_members = {
  "dracon_msgs::msg",  // message namespace
  "JobList",  // message name
  1,  // number of fields
  sizeof(dracon_msgs::msg::JobList),
  JobList_message_member_array,  // message members
  JobList_init_function,  // function to initialize message memory (memory has to be allocated)
  JobList_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JobList_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JobList_message_members,
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
get_message_type_support_handle<dracon_msgs::msg::JobList>()
{
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobList_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dracon_msgs, msg, JobList)() {
  return &::dracon_msgs::msg::rosidl_typesupport_introspection_cpp::JobList_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
