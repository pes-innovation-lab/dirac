// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dracon_msgs:msg/Job.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB__BUILDER_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dracon_msgs/msg/detail/job__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dracon_msgs
{

namespace msg
{

namespace builder
{

class Init_Job_y
{
public:
  explicit Init_Job_y(::dracon_msgs::msg::Job & msg)
  : msg_(msg)
  {}
  ::dracon_msgs::msg::Job y(::dracon_msgs::msg::Job::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dracon_msgs::msg::Job msg_;
};

class Init_Job_x
{
public:
  explicit Init_Job_x(::dracon_msgs::msg::Job & msg)
  : msg_(msg)
  {}
  Init_Job_y x(::dracon_msgs::msg::Job::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Job_y(msg_);
  }

private:
  ::dracon_msgs::msg::Job msg_;
};

class Init_Job_job_id
{
public:
  Init_Job_job_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Job_x job_id(::dracon_msgs::msg::Job::_job_id_type arg)
  {
    msg_.job_id = std::move(arg);
    return Init_Job_x(msg_);
  }

private:
  ::dracon_msgs::msg::Job msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dracon_msgs::msg::Job>()
{
  return dracon_msgs::msg::builder::Init_Job_job_id();
}

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB__BUILDER_HPP_
