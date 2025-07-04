// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dracon_msgs:msg/JobAssignmentTable.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__STRUCT_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'assignments'
#include "dracon_msgs/msg/detail/job_assignment__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__dracon_msgs__msg__JobAssignmentTable __attribute__((deprecated))
#else
# define DEPRECATED__dracon_msgs__msg__JobAssignmentTable __declspec(deprecated)
#endif

namespace dracon_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JobAssignmentTable_
{
  using Type = JobAssignmentTable_<ContainerAllocator>;

  explicit JobAssignmentTable_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit JobAssignmentTable_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _assignments_type =
    std::vector<dracon_msgs::msg::JobAssignment_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dracon_msgs::msg::JobAssignment_<ContainerAllocator>>>;
  _assignments_type assignments;

  // setters for named parameter idiom
  Type & set__assignments(
    const std::vector<dracon_msgs::msg::JobAssignment_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<dracon_msgs::msg::JobAssignment_<ContainerAllocator>>> & _arg)
  {
    this->assignments = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> *;
  using ConstRawPtr =
    const dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dracon_msgs__msg__JobAssignmentTable
    std::shared_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dracon_msgs__msg__JobAssignmentTable
    std::shared_ptr<dracon_msgs::msg::JobAssignmentTable_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JobAssignmentTable_ & other) const
  {
    if (this->assignments != other.assignments) {
      return false;
    }
    return true;
  }
  bool operator!=(const JobAssignmentTable_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JobAssignmentTable_

// alias to use template instance with default allocator
using JobAssignmentTable =
  dracon_msgs::msg::JobAssignmentTable_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB_ASSIGNMENT_TABLE__STRUCT_HPP_
