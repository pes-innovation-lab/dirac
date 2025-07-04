// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/JobStatus.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_STATUS__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_JobStatus_status
{
public:
  explicit Init_JobStatus_status(::dracon_msgs::msg::JobStatus & msg)
  : msg_(msg)
  {}
  ::dracon_msgs::msg::JobStatus status(::dracon_msgs::msg::JobStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::JobStatus msg_;
};

class Init_JobStatus_job_id
{
public:
  explicit Init_JobStatus_job_id(::dracon_msgs::msg::JobStatus & msg)
  : msg_(msg)
  {}
  Init_JobStatus_status job_id(::dracon_msgs::msg::JobStatus::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_JobStatus_status(msg_);
  }

private:
  ::dracon_msgs::msg::JobStatus msg_;
};

class Init_JobStatus_agent_id
{
public:
  Init_JobStatus_agent_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JobStatus_job_id agent_id(::dracon_msgs::msg::JobStatus::_agent_id_type arg)
  {
    msg_.agent_id = std::move(arg);
    return Init_JobStatus_job_id(msg_);
  }

private:
  ::dracon_msgs::msg::JobStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::JobStatus>()
{
  return dracon_msgs::msg::builder::Init_JobStatus_agent_id();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_STATUS__BUILDER_HPP_
