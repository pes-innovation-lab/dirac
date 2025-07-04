// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dracon_msgs:msg/JobCost.idl
// generated code does not contain a copyright notice
#include "dracon_msgs/msg/detail/job_cost__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `agent_id`
// Member `job_id`
#include "rosidl_runtime_c/string_functions.h"

bool
dracon_msgs__msg__JobCost__init(dracon_msgs__msg__JobCost * msg)
{
  if (!msg) {
    return false;
  }
  // agent_id
  if (!rosidl_runtime_c__String__init(&msg->agent_id)) {
    dracon_msgs__msg__JobCost__fini(msg);
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__init(&msg->job_id)) {
    dracon_msgs__msg__JobCost__fini(msg);
    return false;
  }
  // cost
  return true;
}

void
dracon_msgs__msg__JobCost__fini(dracon_msgs__msg__JobCost * msg)
{
  if (!msg) {
    return;
  }
  // agent_id
  rosidl_runtime_c__String__fini(&msg->agent_id);
  // job_id
  rosidl_runtime_c__String__fini(&msg->job_id);
  // cost
}

bool
dracon_msgs__msg__JobCost__are_equal(const dracon_msgs__msg__JobCost * lhs, const dracon_msgs__msg__JobCost * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // agent_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->agent_id), &(rhs->agent_id)))
  {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->job_id), &(rhs->job_id)))
  {
    return false;
  }
  // cost
  if (lhs->cost != rhs->cost) {
    return false;
  }
  return true;
}

bool
dracon_msgs__msg__JobCost__copy(
  const dracon_msgs__msg__JobCost * input,
  dracon_msgs__msg__JobCost * output)
{
  if (!input || !output) {
    return false;
  }
  // agent_id
  if (!rosidl_runtime_c__String__copy(
      &(input->agent_id), &(output->agent_id)))
  {
    return false;
  }
  // job_id
  if (!rosidl_runtime_c__String__copy(
      &(input->job_id), &(output->job_id)))
  {
    return false;
  }
  // cost
  output->cost = input->cost;
  return true;
}

dracon_msgs__msg__JobCost *
dracon_msgs__msg__JobCost__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobCost * msg = (dracon_msgs__msg__JobCost *)allocator.allocate(sizeof(dracon_msgs__msg__JobCost), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dracon_msgs__msg__JobCost));
  bool success = dracon_msgs__msg__JobCost__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dracon_msgs__msg__JobCost__destroy(dracon_msgs__msg__JobCost * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dracon_msgs__msg__JobCost__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dracon_msgs__msg__JobCost__Sequence__init(dracon_msgs__msg__JobCost__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobCost * data = NULL;

  if (size) {
    data = (dracon_msgs__msg__JobCost *)allocator.zero_allocate(size, sizeof(dracon_msgs__msg__JobCost), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dracon_msgs__msg__JobCost__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dracon_msgs__msg__JobCost__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dracon_msgs__msg__JobCost__Sequence__fini(dracon_msgs__msg__JobCost__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dracon_msgs__msg__JobCost__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dracon_msgs__msg__JobCost__Sequence *
dracon_msgs__msg__JobCost__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobCost__Sequence * array = (dracon_msgs__msg__JobCost__Sequence *)allocator.allocate(sizeof(dracon_msgs__msg__JobCost__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dracon_msgs__msg__JobCost__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dracon_msgs__msg__JobCost__Sequence__destroy(dracon_msgs__msg__JobCost__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dracon_msgs__msg__JobCost__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dracon_msgs__msg__JobCost__Sequence__are_equal(const dracon_msgs__msg__JobCost__Sequence * lhs, const dracon_msgs__msg__JobCost__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dracon_msgs__msg__JobCost__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dracon_msgs__msg__JobCost__Sequence__copy(
  const dracon_msgs__msg__JobCost__Sequence * input,
  dracon_msgs__msg__JobCost__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dracon_msgs__msg__JobCost);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dracon_msgs__msg__JobCost * data =
      (dracon_msgs__msg__JobCost *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dracon_msgs__msg__JobCost__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dracon_msgs__msg__JobCost__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dracon_msgs__msg__JobCost__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
