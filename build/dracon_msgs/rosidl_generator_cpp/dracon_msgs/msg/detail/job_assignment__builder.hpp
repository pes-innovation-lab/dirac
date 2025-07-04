// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/JobAssignment.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job_assignment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_JobAssignment_job_y
{
public:
  explicit Init_JobAssignment_job_y(::dracon_msgs::msg::JobAssignment & msg)
  : msg_(msg)
  {}
  ::dracon_msgs::msg::JobAssignment job_y(::dracon_msgs::msg::JobAssignment::_job_y_type arg)
  {
    msg_.job_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::JobAssignment msg_;
};

class Init_JobAssignment_job_x
{
public:
  explicit Init_JobAssignment_job_x(::dracon_msgs::msg::JobAssignment & msg)
  : msg_(msg)
  {}
  Init_JobAssignment_job_y job_x(::dracon_msgs::msg::JobAssignment::_job_x_type arg)
  {
    msg_.job_x = std::move(arg);
    return Init_JobAssignment_job_y(msg_);
  }

private:
  ::dracon_msgs::msg::JobAssignment msg_;
};

class Init_JobAssignment_job_id
{
public:
  explicit Init_JobAssignment_job_id(::dracon_msgs::msg::JobAssignment & msg)
  : msg_(msg)
  {}
  Init_JobAssignment_job_x job_id(::dracon_msgs::msg::JobAssignment::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_JobAssignment_job_x(msg_);
  }

private:
  ::dracon_msgs::msg::JobAssignment msg_;
};

class Init_JobAssignment_agent_id
{
public:
  Init_JobAssignment_agent_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JobAssignment_job_id agent_id(::dracon_msgs::msg::JobAssignment::_agent_id_type arg)
  {
    msg_.agent_id = std::move(arg);
    return Init_JobAssignment_job_id(msg_);
  }

private:
  ::dracon_msgs::msg::JobAssignment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::JobAssignment>()
{
  return dracon_msgs::msg::builder::Init_JobAssignment_agent_id();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT__BUILDER_HPP_
