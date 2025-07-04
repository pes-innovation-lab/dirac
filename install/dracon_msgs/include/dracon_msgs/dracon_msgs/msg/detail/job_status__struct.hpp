// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dracon_msgs:msg/JobStatus.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_STATUS__STRUCT_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dracon_msgs__msg__JobStatus __attribute__((deprecated))
#else
# define DEPRECATED__dracon_msgs__msg__JobStatus __declspec(deprecated)
#endif

namespace dracon_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JobStatus_
{
  using Type = JobStatus_<ContainerAllocator>;

  explicit JobStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = "";
      this->job_id = "";
      this->status = "";
    }
  }

  explicit JobStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : agent_id(_alloc),
    job_id(_alloc),
    status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->agent_id = "";
      this->job_id = "";
      this->status = "";
    }
  }

  // field types and members
  using _agent_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _agent_id_type agent_id;
  using _job_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _job_id_type job_id;
  using _status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_type status;

  // setters for named parameter idiom
  Type & set__agent_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->agent_id = _arg;
    return *this;
  }
  Type & set__job_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->job_id = _arg;
    return *this;
  }
  Type & set__status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dracon_msgs::msg::JobStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const dracon_msgs::msg::JobStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dracon_msgs__msg__JobStatus
    std::shared_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dracon_msgs__msg__JobStatus
    std::shared_ptr<dracon_msgs::msg::JobStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JobStatus_ & other) const
  {
    if (this->agent_id != other.agent_id) {
      return false;
    }
    if (this->job_id != other.job_id) {
      return false;
    }
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const JobStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JobStatus_

// alias to use template instance with default allocator
using JobStatus =
  dracon_msgs::msg::JobStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_STATUS__STRUCT_HPP_
