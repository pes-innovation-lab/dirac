// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dracon_msgs:msg/JobList.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_LIST__TRAITS_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_LIST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dracon_msgs/msg/detail/job_list__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'jobs'
#include "dracon_msgs/msg/detail/job__traits.hpp"

namespace dracon_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JobList & msg,
  std::ostream & out)
{
  out << "{";
  // member: jobs
  {
    if (msg.jobs.size() == 0) {
      out << "jobs: []";
    } else {
      out << "jobs: [";
      size_t pending_items = msg.jobs.size();
      for (auto item : msg.jobs) {
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
  const JobList & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: jobs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.jobs.size() == 0) {
      out << "jobs: []\n";
    } else {
      out << "jobs:\n";
      for (auto item : msg.jobs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JobList & msg, bool use_flow_style = false)
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
  const dracon_msgs::msg::JobList & msg,
  std::ostream & out, size_t indentation = 0)
{
  dracon_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dracon_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dracon_msgs::msg::JobList & msg)
{
  return dracon_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dracon_msgs::msg::JobList>()
{
  return "dracon_msgs::msg::JobList";
}

template<>
inline const char * name<dracon_msgs::msg::JobList>()
{
  return "dracon_msgs/msg/JobList";
}

template<>
struct has_fixed_size<dracon_msgs::msg::JobList>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dracon_msgs::msg::JobList>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dracon_msgs::msg::JobList>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_LIST__TRAITS_HPP_
