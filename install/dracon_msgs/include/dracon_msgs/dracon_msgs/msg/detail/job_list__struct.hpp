// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dracon_msgs:msg/JobList.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_LIST__STRUCT_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_LIST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'jobs'
#include "dracon_msgs/msg/detail/job__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dracon_msgs__msg__JobList __attribute__((deprecated))
#else
# define DEPRECATED__dracon_msgs__msg__JobList __declspec(deprecated)
#endif

namespace dracon_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JobList_
{
  using Type = JobList_<ContainerAllocator>;

  explicit JobList_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit JobList_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _jobs_type =
    std::vector<dracon_msgs::msg::Job_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dracon_msgs::msg::Job_<ContainerAllocator>>>;
  _jobs_type jobs;

  // setters for named parameter idiom
  Type & set__jobs(
    const std::vector<dracon_msgs::msg::Job_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dracon_msgs::msg::Job_<ContainerAllocator>>> & _arg)
  {
    this->jobs = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dracon_msgs::msg::JobList_<ContainerAllocator> *;
  using ConstRawPtr =
    const dracon_msgs::msg::JobList_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobList_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobList_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobList_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobList_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobList_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobList_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobList_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobList_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dracon_msgs__msg__JobList
    std::shared_ptr<dracon_msgs::msg::JobList_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dracon_msgs__msg__JobList
    std::shared_ptr<dracon_msgs::msg::JobList_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JobList_ & other) const
  {
    if (this->jobs != other.jobs) {
      return false;
    }
    return true;
  }
  bool operator!=(const JobList_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JobList_

// alias to use template instance with default allocator
using JobList =
  dracon_msgs::msg::JobList_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_LIST__STRUCT_HPP_
