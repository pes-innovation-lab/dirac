// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/JobCost.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_COST__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_COST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job_cost__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_JobCost_cost
{
public:
  explicit Init_JobCost_cost(::dracon_msgs::msg::JobCost & msg)
  : msg_(msg)
  {}
  ::dracon_msgs::msg::JobCost cost(::dracon_msgs::msg::JobCost::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::JobCost msg_;
};

class Init_JobCost_job_id
{
public:
  explicit Init_JobCost_job_id(::dracon_msgs::msg::JobCost & msg)
  : msg_(msg)
  {}
  Init_JobCost_cost job_id(::dracon_msgs::msg::JobCost::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_JobCost_cost(msg_);
  }

private:
  ::dracon_msgs::msg::JobCost msg_;
};

class Init_JobCost_agent_id
{
public:
  Init_JobCost_agent_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JobCost_job_id agent_id(::dracon_msgs::msg::JobCost::_agent_id_type arg)
  {
    msg_.agent_id = std::move(arg);
    return Init_JobCost_job_id(msg_);
  }

private:
  ::dracon_msgs::msg::JobCost msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::JobCost>()
{
  return dracon_msgs::msg::builder::Init_JobCost_agent_id();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_COST__BUILDER_HPP_
