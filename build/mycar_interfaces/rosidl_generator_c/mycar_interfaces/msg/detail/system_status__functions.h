// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mycar_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#ifndef MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__FUNCTIONS_H_
#define MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "mycar_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "mycar_interfaces/msg/detail/system_status__struct.h"

/// Initialize msg/SystemStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mycar_interfaces__msg__SystemStatus
 * )) before or use
 * mycar_interfaces__msg__SystemStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__init(mycar_interfaces__msg__SystemStatus * msg);

/// Finalize msg/SystemStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
void
mycar_interfaces__msg__SystemStatus__fini(mycar_interfaces__msg__SystemStatus * msg);

/// Create msg/SystemStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mycar_interfaces__msg__SystemStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
mycar_interfaces__msg__SystemStatus *
mycar_interfaces__msg__SystemStatus__create();

/// Destroy msg/SystemStatus message.
/**
 * It calls
 * mycar_interfaces__msg__SystemStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
void
mycar_interfaces__msg__SystemStatus__destroy(mycar_interfaces__msg__SystemStatus * msg);

/// Check for msg/SystemStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__are_equal(const mycar_interfaces__msg__SystemStatus * lhs, const mycar_interfaces__msg__SystemStatus * rhs);

/// Copy a msg/SystemStatus message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__copy(
  const mycar_interfaces__msg__SystemStatus * input,
  mycar_interfaces__msg__SystemStatus * output);

/// Initialize array of msg/SystemStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * mycar_interfaces__msg__SystemStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__Sequence__init(mycar_interfaces__msg__SystemStatus__Sequence * array, size_t size);

/// Finalize array of msg/SystemStatus messages.
/**
 * It calls
 * mycar_interfaces__msg__SystemStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
void
mycar_interfaces__msg__SystemStatus__Sequence__fini(mycar_interfaces__msg__SystemStatus__Sequence * array);

/// Create array of msg/SystemStatus messages.
/**
 * It allocates the memory for the array and calls
 * mycar_interfaces__msg__SystemStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
mycar_interfaces__msg__SystemStatus__Sequence *
mycar_interfaces__msg__SystemStatus__Sequence__create(size_t size);

/// Destroy array of msg/SystemStatus messages.
/**
 * It calls
 * mycar_interfaces__msg__SystemStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
void
mycar_interfaces__msg__SystemStatus__Sequence__destroy(mycar_interfaces__msg__SystemStatus__Sequence * array);

/// Check for msg/SystemStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__Sequence__are_equal(const mycar_interfaces__msg__SystemStatus__Sequence * lhs, const mycar_interfaces__msg__SystemStatus__Sequence * rhs);

/// Copy an array of msg/SystemStatus messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mycar_interfaces
bool
mycar_interfaces__msg__SystemStatus__Sequence__copy(
  const mycar_interfaces__msg__SystemStatus__Sequence * input,
  mycar_interfaces__msg__SystemStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MYCAR_INTERFACES__MSG__DETAIL__SYSTEM_STATUS__FUNCTIONS_H_
