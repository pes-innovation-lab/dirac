// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dracon_msgs:msg/JobAssignmentTable.idl
// generated code does not contain a copyright notice
#include "dracon_msgs/msg/detail/job_assignment_table__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `assignments`
#include "dracon_msgs/msg/detail/job_assignment__functions.h"

bool
dracon_msgs__msg__JobAssignmentTable__init(dracon_msgs__msg__JobAssignmentTable * msg)
{
  if (!msg) {
    return false;
  }
  // assignments
  if (!dracon_msgs__msg__JobAssignment__Sequence__init(&msg->assignments, 0)) {
    dracon_msgs__msg__JobAssignmentTable__fini(msg);
    return false;
  }
  return true;
}

void
dracon_msgs__msg__JobAssignmentTable__fini(dracon_msgs__msg__JobAssignmentTable * msg)
{
  if (!msg) {
    return;
  }
  // assignments
  dracon_msgs__msg__JobAssignment__Sequence__fini(&msg->assignments);
}

bool
dracon_msgs__msg__JobAssignmentTable__are_equal(const dracon_msgs__msg__JobAssignmentTable * lhs, const dracon_msgs__msg__JobAssignmentTable * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // assignments
  if (!dracon_msgs__msg__JobAssignment__Sequence__are_equal(
      &(lhs->assignments), &(rhs->assignments)))
  {
    return false;
  }
  return true;
}

bool
dracon_msgs__msg__JobAssignmentTable__copy(
  const dracon_msgs__msg__JobAssignmentTable * input,
  dracon_msgs__msg__JobAssignmentTable * output)
{
  if (!input || !output) {
    return false;
  }
  // assignments
  if (!dracon_msgs__msg__JobAssignment__Sequence__copy(
      &(input->assignments), &(output->assignments)))
  {
    return false;
  }
  return true;
}

dracon_msgs__msg__JobAssignmentTable *
dracon_msgs__msg__JobAssignmentTable__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobAssignmentTable * msg = (dracon_msgs__msg__JobAssignmentTable *)allocator.allocate(sizeof(dracon_msgs__msg__JobAssignmentTable), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dracon_msgs__msg__JobAssignmentTable));
  bool success = dracon_msgs__msg__JobAssignmentTable__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dracon_msgs__msg__JobAssignmentTable__destroy(dracon_msgs__msg__JobAssignmentTable * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dracon_msgs__msg__JobAssignmentTable__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dracon_msgs__msg__JobAssignmentTable__Sequence__init(dracon_msgs__msg__JobAssignmentTable__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobAssignmentTable * data = NULL;

  if (size) {
    data = (dracon_msgs__msg__JobAssignmentTable *)allocator.zero_allocate(size, sizeof(dracon_msgs__msg__JobAssignmentTable), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dracon_msgs__msg__JobAssignmentTable__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dracon_msgs__msg__JobAssignmentTable__fini(&data[i - 1]);
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
dracon_msgs__msg__JobAssignmentTable__Sequence__fini(dracon_msgs__msg__JobAssignmentTable__Sequence * array)
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
      dracon_msgs__msg__JobAssignmentTable__fini(&array->data[i]);
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

dracon_msgs__msg__JobAssignmentTable__Sequence *
dracon_msgs__msg__JobAssignmentTable__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dracon_msgs__msg__JobAssignmentTable__Sequence * array = (dracon_msgs__msg__JobAssignmentTable__Sequence *)allocator.allocate(sizeof(dracon_msgs__msg__JobAssignmentTable__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dracon_msgs__msg__JobAssignmentTable__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dracon_msgs__msg__JobAssignmentTable__Sequence__destroy(dracon_msgs__msg__JobAssignmentTable__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dracon_msgs__msg__JobAssignmentTable__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dracon_msgs__msg__JobAssignmentTable__Sequence__are_equal(const dracon_msgs__msg__JobAssignmentTable__Sequence * lhs, const dracon_msgs__msg__JobAssignmentTable__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dracon_msgs__msg__JobAssignmentTable__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dracon_msgs__msg__JobAssignmentTable__Sequence__copy(
  const dracon_msgs__msg__JobAssignmentTable__Sequence * input,
  dracon_msgs__msg__JobAssignmentTable__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dracon_msgs__msg__JobAssignmentTable);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dracon_msgs__msg__JobAssignmentTable * data =
      (dracon_msgs__msg__JobAssignmentTable *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dracon_msgs__msg__JobAssignmentTable__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dracon_msgs__msg__JobAssignmentTable__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dracon_msgs__msg__JobAssignmentTable__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
