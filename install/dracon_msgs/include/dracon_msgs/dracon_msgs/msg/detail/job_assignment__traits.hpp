// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dracon_msgs:msg/JobAssignment.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__TRAITS_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dracon_msgs/msg/detail/job_assignment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dracon_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JobAssignment & msg,
  std::ostream & out)
{
  out << "{";
  // member: agent_id
  {
    out << "agent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.agent_id, out);
    out << ", ";
  }

  // member: job_id
  {
    out << "job_id: ";
    rosidl_generator_traits::value_to_yaml(msg.job_id, out);
    out << ", ";
  }

  // member: job_x
  {
    out << "job_x: ";
    rosidl_generator_traits::value_to_yaml(msg.job_x, out);
    out << ", ";
  }

  // member: job_y
  {
    out << "job_y: ";
    rosidl_generator_traits::value_to_yaml(msg.job_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JobAssignment & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: agent_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "agent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.agent_id, out);
    out << "\n";
  }

  // member: job_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "job_id: ";
    rosidl_generator_traits::value_to_yaml(msg.job_id, out);
    out << "\n";
  }

  // member: job_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "job_x: ";
    rosidl_generator_traits::value_to_yaml(msg.job_x, out);
    out << "\n";
  }

  // member: job_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "job_y: ";
    rosidl_generator_traits::value_to_yaml(msg.job_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JobAssignment & msg, bool use_flow_style = false)
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
  const dracon_msgs::msg::JobAssignment & msg,
  std::ostream & out, size_t indentation = 0)
{
  dracon_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dracon_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dracon_msgs::msg::JobAssignment & msg)
{
  return dracon_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dracon_msgs::msg::JobAssignment>()
{
  return "dracon_msgs::msg::JobAssignment";
}

template<>
inline const char * name<dracon_msgs::msg::JobAssignment>()
{
  return "dracon_msgs/msg/JobAssignment";
}

template<>
struct has_fixed_size<dracon_msgs::msg::JobAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dracon_msgs::msg::JobAssignment>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dracon_msgs::msg::JobAssignment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__TRAITS_HPP_
