// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ch4_interfaces:srv/Patrol.idl
// generated code does not contain a copyright notice

#ifndef CH4_INTERFACES__SRV__DETAIL__PATROL__TRAITS_HPP_
#define CH4_INTERFACES__SRV__DETAIL__PATROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ch4_interfaces/srv/detail/patrol__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ch4_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Patrol_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: target_x
  {
    out << "target_x: ";
    rosidl_generator_traits::value_to_yaml(msg.target_x, out);
    out << ", ";
  }

  // member: target_y
  {
    out << "target_y: ";
    rosidl_generator_traits::value_to_yaml(msg.target_y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Patrol_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: target_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_x: ";
    rosidl_generator_traits::value_to_yaml(msg.target_x, out);
    out << "\n";
  }

  // member: target_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "target_y: ";
    rosidl_generator_traits::value_to_yaml(msg.target_y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Patrol_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ch4_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ch4_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ch4_interfaces::srv::Patrol_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  ch4_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ch4_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ch4_interfaces::srv::Patrol_Request & msg)
{
  return ch4_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ch4_interfaces::srv::Patrol_Request>()
{
  return "ch4_interfaces::srv::Patrol_Request";
}

template<>
inline const char * name<ch4_interfaces::srv::Patrol_Request>()
{
  return "ch4_interfaces/srv/Patrol_Request";
}

template<>
struct has_fixed_size<ch4_interfaces::srv::Patrol_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ch4_interfaces::srv::Patrol_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ch4_interfaces::srv::Patrol_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace ch4_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Patrol_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: result
  {
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Patrol_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "result: ";
    rosidl_generator_traits::value_to_yaml(msg.result, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Patrol_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace ch4_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ch4_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ch4_interfaces::srv::Patrol_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  ch4_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ch4_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const ch4_interfaces::srv::Patrol_Response & msg)
{
  return ch4_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<ch4_interfaces::srv::Patrol_Response>()
{
  return "ch4_interfaces::srv::Patrol_Response";
}

template<>
inline const char * name<ch4_interfaces::srv::Patrol_Response>()
{
  return "ch4_interfaces/srv/Patrol_Response";
}

template<>
struct has_fixed_size<ch4_interfaces::srv::Patrol_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ch4_interfaces::srv::Patrol_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ch4_interfaces::srv::Patrol_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<ch4_interfaces::srv::Patrol>()
{
  return "ch4_interfaces::srv::Patrol";
}

template<>
inline const char * name<ch4_interfaces::srv::Patrol>()
{
  return "ch4_interfaces/srv/Patrol";
}

template<>
struct has_fixed_size<ch4_interfaces::srv::Patrol>
  : std::integral_constant<
    bool,
    has_fixed_size<ch4_interfaces::srv::Patrol_Request>::value &&
    has_fixed_size<ch4_interfaces::srv::Patrol_Response>::value
  >
{
};

template<>
struct has_bounded_size<ch4_interfaces::srv::Patrol>
  : std::integral_constant<
    bool,
    has_bounded_size<ch4_interfaces::srv::Patrol_Request>::value &&
    has_bounded_size<ch4_interfaces::srv::Patrol_Response>::value
  >
{
};

template<>
struct is_service<ch4_interfaces::srv::Patrol>
  : std::true_type
{
};

template<>
struct is_service_request<ch4_interfaces::srv::Patrol_Request>
  : std::true_type
{
};

template<>
struct is_service_response<ch4_interfaces::srv::Patrol_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CH4_INTERFACES__SRV__DETAIL__PATROL__TRAITS_HPP_
