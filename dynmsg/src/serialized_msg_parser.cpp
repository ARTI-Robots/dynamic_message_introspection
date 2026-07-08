// Copyright 2026 Fabian Hirmann
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

#include <dynmsg/serialized_msg_parser.hpp>

#include <yaml-cpp/yaml.h>
#include <rcutils/logging_macros.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

#include <rclcpp/serialization.hpp>
#include <dynmsg/message_reading.hpp>

namespace dynmsg
{

namespace cpp
{

SerializedMsgParser::SerializedMsgParser(const std::string & topic_type)
{
  const auto ts_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  const auto type_support = rclcpp::get_typesupport_handle(
    topic_type, "rosidl_typesupport_cpp", *ts_lib);

  // Create a non-templated serializer
  serializer_ = std::make_unique<rclcpp::SerializationBase>(type_support);

  // Set up the ros message to yaml conversion
  const std::string pkg = topic_type.substr(0, topic_type.find('/'));
  const std::string name = topic_type.substr(topic_type.rfind('/') + 1);
  const InterfaceTypeName interface_type{pkg, name};
  RCUTILS_LOG_DEBUG_NAMED(
    "dynmsg", "Interface type: %s::%s", interface_type.first.c_str(),
    interface_type.second.c_str());

  type_info_ = dynmsg::cpp::get_type_info(interface_type);
  if (type_info_ == nullptr) {
    RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Failed to get type info for type");
    throw std::runtime_error("Failed to get type info for type");
  }
}

YAML::Node SerializedMsgParser::parse(const rclcpp::SerializedMessage & serialized_msg) const
{
  RosMessage_Cpp ros_msg;
  // Load the introspection information and allocate space for the ROS message's binary
  // representation
  auto default_allocator = rcutils_get_default_allocator();
  if (DYNMSG_RET_OK !=
    dynmsg::cpp::ros_message_with_typeinfo_init(type_info_, &ros_msg, &default_allocator))
  {
    RCUTILS_LOG_DEBUG_NAMED("dynmsg", "Message init with type info failed");
    throw std::runtime_error("Message init with type info failed");
  }

  // Deserialize
  serializer_->deserialize_message(&serialized_msg, ros_msg.data);

  // convert the deserialized binary message to yaml
  const auto msg_yaml = dynmsg::cpp::message_to_yaml(ros_msg);

  // destroy the allocated ros message again after conversion
  dynmsg::cpp::ros_message_destroy_with_allocator(&ros_msg, &default_allocator);

  return msg_yaml;
}

}  // namespace cpp

}  // namespace dynmsg
