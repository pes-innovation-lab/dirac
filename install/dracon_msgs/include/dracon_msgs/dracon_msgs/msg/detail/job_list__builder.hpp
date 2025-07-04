// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/JobList.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_LIST__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_LIST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job_list__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_JobList_jobs
{
public:
  Init_JobList_jobs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dracon_msgs::msg::JobList jobs(::dracon_msgs::msg::JobList::_jobs_type arg)
  {
    msg_.jobs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::JobList msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::JobList>()
{
  return dracon_msgs::msg::builder::Init_JobList_jobs();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_LIST__BUILDER_HPP_
