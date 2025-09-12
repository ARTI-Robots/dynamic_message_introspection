// Copyright 2020 Open Source Robotics Foundation, Inc.
// Copyright 2021 Christophe Bedard
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNMSG__TYPESUPPORT_HPP_
#define DYNMSG__TYPESUPPORT_HPP_

#include <utility>
#include <string>
#include <vector>

#include <boost/type_index.hpp>

#include "rcutils/allocator.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "dynmsg/types.h"

extern "C"
{
// Structure used to store the type support for a single interface type
using TypeSupport = rosidl_message_type_support_t;
// Structure used to store the introspection information for a single interface type
using TypeInfo_C = rosidl_typesupport_introspection_c__MessageMembers;
// Structure used to store the introspection information for a single field of a interface type
using MemberInfo_C = rosidl_typesupport_introspection_c__MessageMember;

using TypeInfo_Cpp = rosidl_typesupport_introspection_cpp::MessageMembers;
using MemberInfo_Cpp = rosidl_typesupport_introspection_cpp::MessageMember;

// A ROS message, stored in a binary buffer with attached introspection information
typedef struct RosMessage_C
{
  const TypeInfo_C * type_info;
  uint8_t * data;
} RosMessage_C;

typedef struct RosMessage_Cpp
{
  const TypeInfo_Cpp * type_info;
  uint8_t * data;
} RosMessage_Cpp;

using TypeInfo = TypeInfo_C;
using MemberInfo = MemberInfo_C;
using RosMessage = RosMessage_C;

typedef const rosidl_message_type_support_t * (* get_message_ts_func)();

// An interface type can be identified by its namespace (i.e. the package that stores it) and its
// type name
using InterfaceTypeName = std::pair<std::string, std::string>;
}  // extern "C"

namespace dynmsg
{

namespace c
{

/// Search for and load the introspection library for a single interface type.
/**
 * This function will search the system's configured library search paths (which should include the
 * ROS paths) to find a dynamic library named following the pattern
 * "lib[namespace]__rosidl_typesupport_introspection_c.so".
 * When found, it opens that library and loads a function named following the pattern
 * "rosidl_typesupport_introspection_c__get_message_type_support_handle__[namespace]__msg__[type]".
 * This function, when called, provides a pointer to the introspection structure for the specified
 * interface type. This pointer is returned. The information contained in this structure can be
 * used to understand a ROS message stored in a binary buffer, or to construct a ROS message in a
 * binary buffer.
 */
const TypeInfo * get_type_info(const InterfaceTypeName & interface_type);

/// Initialise a RosMessage structure.
/**
 * The introspection information for the specified interface type is loaded from its shared library
 * and stored in the type_info field. The ros_msg buffer is allocated with enough space to store
 * one ROS message of the specified type.
 * When finshed with the RosMessage instance, call ros_message_destroy() to clean up allocated
 * memory.
 */
dynmsg_ret_t ros_message_init(const InterfaceTypeName & interface_type, RosMessage * ros_msg);

/// Version of ros_message_init() but with TypeInfo directly and an allocator.
/**
 * \see ros_message_init()
 */
dynmsg_ret_t ros_message_with_typeinfo_init(
  const TypeInfo * type_info,
  RosMessage * ros_msg,
  rcutils_allocator_t * allocator);

/// Clean up a RosMessage instance by freeing its resources.
void ros_message_destroy(RosMessage * ros_msg);

/// Version of ros_message_destroy but with an allocator.
/**
 * \see ros_message_destroy()
 */
void ros_message_destroy_with_allocator(RosMessage * ros_msg, rcutils_allocator_t * allocator);

}  // namespace c

namespace cpp
{

// Split a fully-qualified type name into its components at "::"
inline std::vector<std::string> split_namespace(const std::string & input)
{
  std::vector<std::string> parts;
  size_t pos = 0;

  while (true) {
    size_t next = input.find("::", pos);
    if (next == std::string::npos) {
      parts.emplace_back(input.substr(pos));
      break;
    }
    parts.emplace_back(input.substr(pos, next - pos));
    pos = next + 2;  // skip "::"
  }
  return parts;
}

inline std::string remove_ros_msg_name_template_suffix(const std::string & input)
{
  // an example input is std_msgs::msg::String_<std::allocator<void> >
  // and the output should be std_msgs::msg::String

  // find the last '>' character, search for the before matching '<' character
  // and remove everything in between
  size_t end_pos = input.rfind('>');
  if (end_pos == std::string::npos) {
    // no template suffix found
    return input;
  }
  int level_count = 1;
  size_t begin_pos = end_pos;
  for (int i = static_cast<int>(end_pos) - 1; i >= 0; i--) {
    if (input[i] == '>') {
      level_count++;
    } else if (input[i] == '<') {
      level_count--;
      if (level_count == 0) {
        // found the matching '<'
        begin_pos = i;
      }
    }
  }

  // -1 because we also need to remove the '_' before the first '<'
  std::string ret = input.substr(0, begin_pos - 1);
  if (end_pos < (input.size() - 1)) {
    ret += input.substr(end_pos + 1);
  }

  return ret;
}

/// C++ version of dynmsg::c::get_type_info()
/**
 * \see dynmsg::c::get_type_info()
 */
const TypeInfo_Cpp * get_type_info(const InterfaceTypeName & interface_type);

template<class T>
const TypeInfo_Cpp * get_type_info()
{
  std::string full_ros_msg_type = boost::typeindex::type_id<T>().pretty_name();

  std::string cleaned_ros_msg_type = remove_ros_msg_name_template_suffix(full_ros_msg_type);

  auto parts = split_namespace(cleaned_ros_msg_type);

  if (parts.size() < 2) {
    // Invalid type name
    return nullptr;
  }

  std::string package_name = parts.front();
  std::string message_name = parts.back();

  InterfaceTypeName interface{package_name, message_name};

  return get_type_info(interface);
}

/// C++ version of dynmsg::c::ros_message_with_typeinfo_init()
/**
 * \see dynmsg::c::ros_message_with_typeinfo_init()
 */
dynmsg_ret_t ros_message_with_typeinfo_init(
  const TypeInfo_Cpp * type_info,
  RosMessage_Cpp * ros_msg,
  rcutils_allocator_t * allocator);


/// C++ version of dynmsg::c::ros_message_destroy_with_allocator()
/**
 * \see dynmsg::c::ros_message_destroy_with_allocator()
 */
void ros_message_destroy_with_allocator(RosMessage_Cpp * ros_msg, rcutils_allocator_t * allocator);

}  // namespace cpp

}  // namespace dynmsg

#endif  // DYNMSG__TYPESUPPORT_HPP_
