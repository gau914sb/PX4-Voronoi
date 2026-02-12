// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/SensorTemp.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__SENSOR_TEMP__BUILDER_HPP_
#define PX4__MSG__DETAIL__SENSOR_TEMP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/sensor_temp__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_SensorTemp_temperature
{
public:
  explicit Init_SensorTemp_temperature(::px4::msg::SensorTemp & msg)
  : msg_(msg)
  {}
  ::px4::msg::SensorTemp temperature(::px4::msg::SensorTemp::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::SensorTemp msg_;
};

class Init_SensorTemp_device_id
{
public:
  explicit Init_SensorTemp_device_id(::px4::msg::SensorTemp & msg)
  : msg_(msg)
  {}
  Init_SensorTemp_temperature device_id(::px4::msg::SensorTemp::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_SensorTemp_temperature(msg_);
  }

private:
  ::px4::msg::SensorTemp msg_;
};

class Init_SensorTemp_timestamp
{
public:
  Init_SensorTemp_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorTemp_device_id timestamp(::px4::msg::SensorTemp::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SensorTemp_device_id(msg_);
  }

private:
  ::px4::msg::SensorTemp msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::SensorTemp>()
{
  return px4::msg::builder::Init_SensorTemp_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__SENSOR_TEMP__BUILDER_HPP_
