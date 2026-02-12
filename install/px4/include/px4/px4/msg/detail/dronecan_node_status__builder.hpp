// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DronecanNodeStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DRONECAN_NODE_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__DRONECAN_NODE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/dronecan_node_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DronecanNodeStatus_vendor_specific_status_code
{
public:
  explicit Init_DronecanNodeStatus_vendor_specific_status_code(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::DronecanNodeStatus vendor_specific_status_code(::px4::msg::DronecanNodeStatus::_vendor_specific_status_code_type arg)
  {
    msg_.vendor_specific_status_code = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_sub_mode
{
public:
  explicit Init_DronecanNodeStatus_sub_mode(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DronecanNodeStatus_vendor_specific_status_code sub_mode(::px4::msg::DronecanNodeStatus::_sub_mode_type arg)
  {
    msg_.sub_mode = std::move(arg);
    return Init_DronecanNodeStatus_vendor_specific_status_code(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_mode
{
public:
  explicit Init_DronecanNodeStatus_mode(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DronecanNodeStatus_sub_mode mode(::px4::msg::DronecanNodeStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_DronecanNodeStatus_sub_mode(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_health
{
public:
  explicit Init_DronecanNodeStatus_health(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DronecanNodeStatus_mode health(::px4::msg::DronecanNodeStatus::_health_type arg)
  {
    msg_.health = std::move(arg);
    return Init_DronecanNodeStatus_mode(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_uptime_sec
{
public:
  explicit Init_DronecanNodeStatus_uptime_sec(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DronecanNodeStatus_health uptime_sec(::px4::msg::DronecanNodeStatus::_uptime_sec_type arg)
  {
    msg_.uptime_sec = std::move(arg);
    return Init_DronecanNodeStatus_health(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_node_id
{
public:
  explicit Init_DronecanNodeStatus_node_id(::px4::msg::DronecanNodeStatus & msg)
  : msg_(msg)
  {}
  Init_DronecanNodeStatus_uptime_sec node_id(::px4::msg::DronecanNodeStatus::_node_id_type arg)
  {
    msg_.node_id = std::move(arg);
    return Init_DronecanNodeStatus_uptime_sec(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

class Init_DronecanNodeStatus_timestamp
{
public:
  Init_DronecanNodeStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DronecanNodeStatus_node_id timestamp(::px4::msg::DronecanNodeStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DronecanNodeStatus_node_id(msg_);
  }

private:
  ::px4::msg::DronecanNodeStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DronecanNodeStatus>()
{
  return px4::msg::builder::Init_DronecanNodeStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DRONECAN_NODE_STATUS__BUILDER_HPP_
