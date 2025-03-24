// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from perception_ros2_msg:msg/Point2f.idl
// generated code does not contain a copyright notice

#ifndef PERCEPTION_ROS2_MSG__MSG__POINT2F__FUNCTIONS_H_
#define PERCEPTION_ROS2_MSG__MSG__POINT2F__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "perception_ros2_msg/msg/rosidl_generator_c__visibility_control.h"

#include "perception_ros2_msg/msg/point2f__struct.h"

/// Initialize msg/Point2f message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * perception_ros2_msg__msg__Point2f
 * )) before or use
 * perception_ros2_msg__msg__Point2f__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
bool
perception_ros2_msg__msg__Point2f__init(perception_ros2_msg__msg__Point2f * msg);

/// Finalize msg/Point2f message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
void
perception_ros2_msg__msg__Point2f__fini(perception_ros2_msg__msg__Point2f * msg);

/// Create msg/Point2f message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * perception_ros2_msg__msg__Point2f__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
perception_ros2_msg__msg__Point2f *
perception_ros2_msg__msg__Point2f__create();

/// Destroy msg/Point2f message.
/**
 * It calls
 * perception_ros2_msg__msg__Point2f__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
void
perception_ros2_msg__msg__Point2f__destroy(perception_ros2_msg__msg__Point2f * msg);


/// Initialize array of msg/Point2f messages.
/**
 * It allocates the memory for the number of elements and calls
 * perception_ros2_msg__msg__Point2f__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
bool
perception_ros2_msg__msg__Point2f__Sequence__init(perception_ros2_msg__msg__Point2f__Sequence * array, size_t size);

/// Finalize array of msg/Point2f messages.
/**
 * It calls
 * perception_ros2_msg__msg__Point2f__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
void
perception_ros2_msg__msg__Point2f__Sequence__fini(perception_ros2_msg__msg__Point2f__Sequence * array);

/// Create array of msg/Point2f messages.
/**
 * It allocates the memory for the array and calls
 * perception_ros2_msg__msg__Point2f__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
perception_ros2_msg__msg__Point2f__Sequence *
perception_ros2_msg__msg__Point2f__Sequence__create(size_t size);

/// Destroy array of msg/Point2f messages.
/**
 * It calls
 * perception_ros2_msg__msg__Point2f__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_perception_ros2_msg
void
perception_ros2_msg__msg__Point2f__Sequence__destroy(perception_ros2_msg__msg__Point2f__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // PERCEPTION_ROS2_MSG__MSG__POINT2F__FUNCTIONS_H_
