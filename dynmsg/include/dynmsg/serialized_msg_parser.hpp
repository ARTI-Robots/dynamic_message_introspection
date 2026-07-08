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

#ifndef DYNMSG__SERIALIZED_MSG_PARSER_HPP_
#define DYNMSG__SERIALIZED_MSG_PARSER_HPP_

#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <dynmsg/typesupport.hpp>

namespace dynmsg
{

namespace cpp
{
class SerializedMsgParser
{
public:
  explicit SerializedMsgParser(const std::string & topic_type);
  YAML::Node parse(const rclcpp::SerializedMessage & serialized_msg) const;

private:
  std::unique_ptr<rclcpp::SerializationBase> serializer_;
  const TypeInfo_Cpp * type_info_;
};

}  // namespace cpp

}  // namespace dynmsg

#endif  // DYNMSG__SERIALIZED_MSG_PARSER_HPP_
