// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mycar_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice
#include "mycar_interfaces/msg/detail/system_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `target_pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"
// Member `detail`
#include "rosidl_runtime_c/string_functions.h"

bool
mycar_interfaces__msg__SystemStatus__init(mycar_interfaces__msg__SystemStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mycar_interfaces__msg__SystemStatus__fini(msg);
    return false;
  }
  // mode
  // target_visible
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->target_pose)) {
    mycar_interfaces__msg__SystemStatus__fini(msg);
    return false;
  }
  // confidence
  // waypoint_index
  // detail
  if (!rosidl_runtime_c__String__init(&msg->detail)) {
    mycar_interfaces__msg__SystemStatus__fini(msg);
    return false;
  }
  return true;
}

void
mycar_interfaces__msg__SystemStatus__fini(mycar_interfaces__msg__SystemStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // mode
  // target_visible
  // target_pose
  geometry_msgs__msg__PoseStamped__fini(&msg->target_pose);
  // confidence
  // waypoint_index
  // detail
  rosidl_runtime_c__String__fini(&msg->detail);
}

bool
mycar_interfaces__msg__SystemStatus__are_equal(const mycar_interfaces__msg__SystemStatus * lhs, const mycar_interfaces__msg__SystemStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // target_visible
  if (lhs->target_visible != rhs->target_visible) {
    return false;
  }
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->target_pose), &(rhs->target_pose)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // waypoint_index
  if (lhs->waypoint_index != rhs->waypoint_index) {
    return false;
  }
  // detail
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->detail), &(rhs->detail)))
  {
    return false;
  }
  return true;
}

bool
mycar_interfaces__msg__SystemStatus__copy(
  const mycar_interfaces__msg__SystemStatus * input,
  mycar_interfaces__msg__SystemStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // mode
  output->mode = input->mode;
  // target_visible
  output->target_visible = input->target_visible;
  // target_pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->target_pose), &(output->target_pose)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // waypoint_index
  output->waypoint_index = input->waypoint_index;
  // detail
  if (!rosidl_runtime_c__String__copy(
      &(input->detail), &(output->detail)))
  {
    return false;
  }
  return true;
}

mycar_interfaces__msg__SystemStatus *
mycar_interfaces__msg__SystemStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycar_interfaces__msg__SystemStatus * msg = (mycar_interfaces__msg__SystemStatus *)allocator.allocate(sizeof(mycar_interfaces__msg__SystemStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mycar_interfaces__msg__SystemStatus));
  bool success = mycar_interfaces__msg__SystemStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mycar_interfaces__msg__SystemStatus__destroy(mycar_interfaces__msg__SystemStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mycar_interfaces__msg__SystemStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mycar_interfaces__msg__SystemStatus__Sequence__init(mycar_interfaces__msg__SystemStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycar_interfaces__msg__SystemStatus * data = NULL;

  if (size) {
    data = (mycar_interfaces__msg__SystemStatus *)allocator.zero_allocate(size, sizeof(mycar_interfaces__msg__SystemStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mycar_interfaces__msg__SystemStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mycar_interfaces__msg__SystemStatus__fini(&data[i - 1]);
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
mycar_interfaces__msg__SystemStatus__Sequence__fini(mycar_interfaces__msg__SystemStatus__Sequence * array)
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
      mycar_interfaces__msg__SystemStatus__fini(&array->data[i]);
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

mycar_interfaces__msg__SystemStatus__Sequence *
mycar_interfaces__msg__SystemStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mycar_interfaces__msg__SystemStatus__Sequence * array = (mycar_interfaces__msg__SystemStatus__Sequence *)allocator.allocate(sizeof(mycar_interfaces__msg__SystemStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mycar_interfaces__msg__SystemStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mycar_interfaces__msg__SystemStatus__Sequence__destroy(mycar_interfaces__msg__SystemStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mycar_interfaces__msg__SystemStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mycar_interfaces__msg__SystemStatus__Sequence__are_equal(const mycar_interfaces__msg__SystemStatus__Sequence * lhs, const mycar_interfaces__msg__SystemStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mycar_interfaces__msg__SystemStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mycar_interfaces__msg__SystemStatus__Sequence__copy(
  const mycar_interfaces__msg__SystemStatus__Sequence * input,
  mycar_interfaces__msg__SystemStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mycar_interfaces__msg__SystemStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mycar_interfaces__msg__SystemStatus * data =
      (mycar_interfaces__msg__SystemStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mycar_interfaces__msg__SystemStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mycar_interfaces__msg__SystemStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mycar_interfaces__msg__SystemStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
