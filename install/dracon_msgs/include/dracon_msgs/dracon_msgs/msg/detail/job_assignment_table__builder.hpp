// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/JobAssignmentTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job_assignment_table__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_JobAssignmentTable_assignments
{
public:
  Init_JobAssignmentTable_assignments()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dracon_msgs::msg::JobAssignmentTable assignments(::dracon_msgs::msg::JobAssignmentTable::_assignments_type arg)
  {
    msg_.assignments = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::JobAssignmentTable msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::JobAssignmentTable>()
{
  return dracon_msgs::msg::builder::Init_JobAssignmentTable_assignments();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__BUILDER_HPP_
