// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dracon_msgs:msg/JobAssignmentTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__TRAITS_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dracon_msgs/msg/detail/job_assignment_table__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'assignments'
#include "dracon_msgs/msg/detail/job_assignment__traits.hpp"

namespace dracon_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JobAssignmentTable & msg,
  std::ostream & out)
{
  out << "{";
  // member: assignments
  {
    if (msg.assignments.size() == 0) {
      out << "assignments: []";
    } else {
      out << "assignments: [";
      size_t pending_items = msg.assignments.size();
      for (auto item : msg.assignments) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JobAssignmentTable & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: assignments
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.assignments.size() == 0) {
      out << "assignments: []\n";
    } else {
      out << "assignments:\n";
      for (auto item : msg.assignments) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JobAssignmentTable & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace dracon_msgs

namespace rosidl_generator_traits
{

[[deprecated("use dracon_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dracon_msgs::msg::JobAssignmentTable & msg,
  std::ostream & out, size_t indentation = 0)
{
  dracon_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dracon_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dracon_msgs::msg::JobAssignmentTable & msg)
{
  return dracon_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dracon_msgs::msg::JobAssignmentTable>()
{
  return "dracon_msgs::msg::JobAssignmentTable";
}

template<>
inline const char * name<dracon_msgs::msg::JobAssignmentTable>()
{
  return "dracon_msgs/msg/JobAssignmentTable";
}

template<>
struct has_fixed_size<dracon_msgs::msg::JobAssignmentTable>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dracon_msgs::msg::JobAssignmentTable>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dracon_msgs::msg::JobAssignmentTable>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__TRAITS_HPP_
