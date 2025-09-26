// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ch4_interfaces:srv/Patrol.idl
// generated code does not contain a copyright notice
#include "ch4_interfaces/srv/detail/patrol__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
ch4_interfaces__srv__Patrol_Request__init(ch4_interfaces__srv__Patrol_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target_x
  // target_y
  return true;
}

void
ch4_interfaces__srv__Patrol_Request__fini(ch4_interfaces__srv__Patrol_Request * msg)
{
  if (!msg) {
    return;
  }
  // target_x
  // target_y
}

bool
ch4_interfaces__srv__Patrol_Request__are_equal(const ch4_interfaces__srv__Patrol_Request * lhs, const ch4_interfaces__srv__Patrol_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target_x
  if (lhs->target_x != rhs->target_x) {
    return false;
  }
  // target_y
  if (lhs->target_y != rhs->target_y) {
    return false;
  }
  return true;
}

bool
ch4_interfaces__srv__Patrol_Request__copy(
  const ch4_interfaces__srv__Patrol_Request * input,
  ch4_interfaces__srv__Patrol_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target_x
  output->target_x = input->target_x;
  // target_y
  output->target_y = input->target_y;
  return true;
}

ch4_interfaces__srv__Patrol_Request *
ch4_interfaces__srv__Patrol_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Request * msg = (ch4_interfaces__srv__Patrol_Request *)allocator.allocate(sizeof(ch4_interfaces__srv__Patrol_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ch4_interfaces__srv__Patrol_Request));
  bool success = ch4_interfaces__srv__Patrol_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ch4_interfaces__srv__Patrol_Request__destroy(ch4_interfaces__srv__Patrol_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ch4_interfaces__srv__Patrol_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ch4_interfaces__srv__Patrol_Request__Sequence__init(ch4_interfaces__srv__Patrol_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Request * data = NULL;

  if (size) {
    data = (ch4_interfaces__srv__Patrol_Request *)allocator.zero_allocate(size, sizeof(ch4_interfaces__srv__Patrol_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ch4_interfaces__srv__Patrol_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ch4_interfaces__srv__Patrol_Request__fini(&data[i - 1]);
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
ch4_interfaces__srv__Patrol_Request__Sequence__fini(ch4_interfaces__srv__Patrol_Request__Sequence * array)
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
      ch4_interfaces__srv__Patrol_Request__fini(&array->data[i]);
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

ch4_interfaces__srv__Patrol_Request__Sequence *
ch4_interfaces__srv__Patrol_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Request__Sequence * array = (ch4_interfaces__srv__Patrol_Request__Sequence *)allocator.allocate(sizeof(ch4_interfaces__srv__Patrol_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ch4_interfaces__srv__Patrol_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ch4_interfaces__srv__Patrol_Request__Sequence__destroy(ch4_interfaces__srv__Patrol_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ch4_interfaces__srv__Patrol_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ch4_interfaces__srv__Patrol_Request__Sequence__are_equal(const ch4_interfaces__srv__Patrol_Request__Sequence * lhs, const ch4_interfaces__srv__Patrol_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ch4_interfaces__srv__Patrol_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ch4_interfaces__srv__Patrol_Request__Sequence__copy(
  const ch4_interfaces__srv__Patrol_Request__Sequence * input,
  ch4_interfaces__srv__Patrol_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ch4_interfaces__srv__Patrol_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ch4_interfaces__srv__Patrol_Request * data =
      (ch4_interfaces__srv__Patrol_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ch4_interfaces__srv__Patrol_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ch4_interfaces__srv__Patrol_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ch4_interfaces__srv__Patrol_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ch4_interfaces__srv__Patrol_Response__init(ch4_interfaces__srv__Patrol_Response * msg)
{
  if (!msg) {
    return false;
  }
  // result
  return true;
}

void
ch4_interfaces__srv__Patrol_Response__fini(ch4_interfaces__srv__Patrol_Response * msg)
{
  if (!msg) {
    return;
  }
  // result
}

bool
ch4_interfaces__srv__Patrol_Response__are_equal(const ch4_interfaces__srv__Patrol_Response * lhs, const ch4_interfaces__srv__Patrol_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (lhs->result != rhs->result) {
    return false;
  }
  return true;
}

bool
ch4_interfaces__srv__Patrol_Response__copy(
  const ch4_interfaces__srv__Patrol_Response * input,
  ch4_interfaces__srv__Patrol_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  output->result = input->result;
  return true;
}

ch4_interfaces__srv__Patrol_Response *
ch4_interfaces__srv__Patrol_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Response * msg = (ch4_interfaces__srv__Patrol_Response *)allocator.allocate(sizeof(ch4_interfaces__srv__Patrol_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ch4_interfaces__srv__Patrol_Response));
  bool success = ch4_interfaces__srv__Patrol_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ch4_interfaces__srv__Patrol_Response__destroy(ch4_interfaces__srv__Patrol_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ch4_interfaces__srv__Patrol_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ch4_interfaces__srv__Patrol_Response__Sequence__init(ch4_interfaces__srv__Patrol_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Response * data = NULL;

  if (size) {
    data = (ch4_interfaces__srv__Patrol_Response *)allocator.zero_allocate(size, sizeof(ch4_interfaces__srv__Patrol_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ch4_interfaces__srv__Patrol_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ch4_interfaces__srv__Patrol_Response__fini(&data[i - 1]);
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
ch4_interfaces__srv__Patrol_Response__Sequence__fini(ch4_interfaces__srv__Patrol_Response__Sequence * array)
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
      ch4_interfaces__srv__Patrol_Response__fini(&array->data[i]);
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

ch4_interfaces__srv__Patrol_Response__Sequence *
ch4_interfaces__srv__Patrol_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ch4_interfaces__srv__Patrol_Response__Sequence * array = (ch4_interfaces__srv__Patrol_Response__Sequence *)allocator.allocate(sizeof(ch4_interfaces__srv__Patrol_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ch4_interfaces__srv__Patrol_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ch4_interfaces__srv__Patrol_Response__Sequence__destroy(ch4_interfaces__srv__Patrol_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ch4_interfaces__srv__Patrol_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ch4_interfaces__srv__Patrol_Response__Sequence__are_equal(const ch4_interfaces__srv__Patrol_Response__Sequence * lhs, const ch4_interfaces__srv__Patrol_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ch4_interfaces__srv__Patrol_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ch4_interfaces__srv__Patrol_Response__Sequence__copy(
  const ch4_interfaces__srv__Patrol_Response__Sequence * input,
  ch4_interfaces__srv__Patrol_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ch4_interfaces__srv__Patrol_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ch4_interfaces__srv__Patrol_Response * data =
      (ch4_interfaces__srv__Patrol_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ch4_interfaces__srv__Patrol_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ch4_interfaces__srv__Patrol_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ch4_interfaces__srv__Patrol_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
