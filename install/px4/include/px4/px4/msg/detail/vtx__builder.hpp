// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Vtx.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__VTX__BUILDER_HPP_
#define PX4__MSG__DETAIL__VTX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/vtx__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Vtx_power_label
{
public:
  explicit Init_Vtx_power_label(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  ::px4::msg::Vtx power_label(::px4::msg::Vtx::_power_label_type arg)
  {
    msg_.power_label = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_power_level
{
public:
  explicit Init_Vtx_power_level(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_power_label power_level(::px4::msg::Vtx::_power_level_type arg)
  {
    msg_.power_level = std::move(arg);
    return Init_Vtx_power_label(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_band_name
{
public:
  explicit Init_Vtx_band_name(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_power_level band_name(::px4::msg::Vtx::_band_name_type arg)
  {
    msg_.band_name = std::move(arg);
    return Init_Vtx_power_level(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_band_letter
{
public:
  explicit Init_Vtx_band_letter(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_band_name band_letter(::px4::msg::Vtx::_band_letter_type arg)
  {
    msg_.band_letter = std::move(arg);
    return Init_Vtx_band_name(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_frequency
{
public:
  explicit Init_Vtx_frequency(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_band_letter frequency(::px4::msg::Vtx::_frequency_type arg)
  {
    msg_.frequency = std::move(arg);
    return Init_Vtx_band_letter(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_channel
{
public:
  explicit Init_Vtx_channel(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_frequency channel(::px4::msg::Vtx::_channel_type arg)
  {
    msg_.channel = std::move(arg);
    return Init_Vtx_frequency(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_band
{
public:
  explicit Init_Vtx_band(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_channel band(::px4::msg::Vtx::_band_type arg)
  {
    msg_.band = std::move(arg);
    return Init_Vtx_channel(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_mode
{
public:
  explicit Init_Vtx_mode(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_band mode(::px4::msg::Vtx::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_Vtx_band(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_device
{
public:
  explicit Init_Vtx_device(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_mode device(::px4::msg::Vtx::_device_type arg)
  {
    msg_.device = std::move(arg);
    return Init_Vtx_mode(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_protocol
{
public:
  explicit Init_Vtx_protocol(::px4::msg::Vtx & msg)
  : msg_(msg)
  {}
  Init_Vtx_device protocol(::px4::msg::Vtx::_protocol_type arg)
  {
    msg_.protocol = std::move(arg);
    return Init_Vtx_device(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

class Init_Vtx_timestamp
{
public:
  Init_Vtx_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Vtx_protocol timestamp(::px4::msg::Vtx::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Vtx_protocol(msg_);
  }

private:
  ::px4::msg::Vtx msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Vtx>()
{
  return px4::msg::builder::Init_Vtx_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__VTX__BUILDER_HPP_
