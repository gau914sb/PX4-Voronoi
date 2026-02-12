// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/RoverSpeedSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ROVER_SPEED_SETPOINT__BUILDER_HPP_
#define PX4__MSG__DETAIL__ROVER_SPEED_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/rover_speed_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_RoverSpeedSetpoint_speed_body_y
{
public:
  explicit Init_RoverSpeedSetpoint_speed_body_y(::px4::msg::RoverSpeedSetpoint & msg)
  : msg_(msg)
  {}
  ::px4::msg::RoverSpeedSetpoint speed_body_y(::px4::msg::RoverSpeedSetpoint::_speed_body_y_type arg)
  {
    msg_.speed_body_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::RoverSpeedSetpoint msg_;
};

class Init_RoverSpeedSetpoint_speed_body_x
{
public:
  explicit Init_RoverSpeedSetpoint_speed_body_x(::px4::msg::RoverSpeedSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverSpeedSetpoint_speed_body_y speed_body_x(::px4::msg::RoverSpeedSetpoint::_speed_body_x_type arg)
  {
    msg_.speed_body_x = std::move(arg);
    return Init_RoverSpeedSetpoint_speed_body_y(msg_);
  }

private:
  ::px4::msg::RoverSpeedSetpoint msg_;
};

class Init_RoverSpeedSetpoint_timestamp
{
public:
  Init_RoverSpeedSetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverSpeedSetpoint_speed_body_x timestamp(::px4::msg::RoverSpeedSetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverSpeedSetpoint_speed_body_x(msg_);
  }

private:
  ::px4::msg::RoverSpeedSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::RoverSpeedSetpoint>()
{
  return px4::msg::builder::Init_RoverSpeedSetpoint_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ROVER_SPEED_SETPOINT__BUILDER_HPP_
