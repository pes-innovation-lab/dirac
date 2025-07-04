// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dracon_msgs:msg/JobCostTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__TRAITS_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dracon_msgs/msg/detail/job_cost_table__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'costs'
#include "dracon_msgs/msg/detail/job_cost__traits.hpp"

namespace dracon_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const JobCostTable & msg,
  std::ostream & out)
{
  out << "{";
  // member: costs
  {
    if (msg.costs.size() == 0) {
      out << "costs: []";
    } else {
      out << "costs: [";
      size_t pending_items = msg.costs.size();
      for (auto item : msg.costs) {
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
  const JobCostTable & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: costs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.costs.size() == 0) {
      out << "costs: []\n";
    } else {
      out << "costs:\n";
      for (auto item : msg.costs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JobCostTable & msg, bool use_flow_style = false)
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
  const dracon_msgs::msg::JobCostTable & msg,
  std::ostream & out, size_t indentation = 0)
{
  dracon_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dracon_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const dracon_msgs::msg::JobCostTable & msg)
{
  return dracon_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dracon_msgs::msg::JobCostTable>()
{
  return "dracon_msgs::msg::JobCostTable";
}

template<>
inline const char * name<dracon_msgs::msg::JobCostTable>()
{
  return "dracon_msgs/msg/JobCostTable";
}

template<>
struct has_fixed_size<dracon_msgs::msg::JobCostTable>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dracon_msgs::msg::JobCostTable>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dracon_msgs::msg::JobCostTable>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_COST_TABLE__TRAITS_HPP_
