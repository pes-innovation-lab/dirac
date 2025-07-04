// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dracon_msgs:msg/Job.idl
// generated code does not contain a copyright notice

#ifndef DRACON_MSGS__MSG__DETAIL__JOB__STRUCT_HPP_
#define DRACON_MSGS__MSG__DETAIL__JOB__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dracon_msgs__msg__Job __attribute__((deprecated))
#else
# define DEPRECATED__dracon_msgs__msg__Job __declspec(deprecated)
#endif

namespace dracon_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Job_
{
  using Type = Job_<ContainerAllocator>;

  explicit Job_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->job_id = "";
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit Job_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : job_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->job_id = "";
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _job_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _job_id_type job_id;
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type & set__job_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->job_id = _arg;
    return *this;
  }
  Type & set__x(
    const double & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const double & _arg)
  {
    this->y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dracon_msgs::msg::Job_<ContainerAllocator> *;
  using ConstRawPtr =
    const dracon_msgs::msg::Job_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dracon_msgs::msg::Job_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dracon_msgs::msg::Job_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::Job_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::Job_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dracon_msgs::msg::Job_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dracon_msgs::msg::Job_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dracon_msgs::msg::Job_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dracon_msgs::msg::Job_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dracon_msgs__msg__Job
    std::shared_ptr<dracon_msgs::msg::Job_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dracon_msgs__msg__Job
    std::shared_ptr<dracon_msgs::msg::Job_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Job_ & other) const
  {
    if (this->job_id != other.job_id) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Job_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Job_

// alias to use template instance with default allocator
using Job =
  dracon_msgs::msg::Job_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dracon_msgs

#endif  // DRACON_MSGS__MSG__DETAIL__JOB__STRUCT_HPP_
