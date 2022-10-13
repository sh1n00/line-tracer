// Generated by gencpp from file lightrover_ros/Wrc201Msg.msg
// DO NOT EDIT!


#ifndef LIGHTROVER_ROS_MESSAGE_WRC201MSG_H
#define LIGHTROVER_ROS_MESSAGE_WRC201MSG_H

#include <ros/service_traits.h>


#include <lightrover_ros/Wrc201MsgRequest.h>
#include <lightrover_ros/Wrc201MsgResponse.h>


namespace lightrover_ros
{

struct Wrc201Msg
{

typedef Wrc201MsgRequest Request;
typedef Wrc201MsgResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Wrc201Msg
} // namespace lightrover_ros


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::lightrover_ros::Wrc201Msg > {
  static const char* value()
  {
    return "43791c45179089218511643bd58fdb58";
  }

  static const char* value(const ::lightrover_ros::Wrc201Msg&) { return value(); }
};

template<>
struct DataType< ::lightrover_ros::Wrc201Msg > {
  static const char* value()
  {
    return "lightrover_ros/Wrc201Msg";
  }

  static const char* value(const ::lightrover_ros::Wrc201Msg&) { return value(); }
};


// service_traits::MD5Sum< ::lightrover_ros::Wrc201MsgRequest> should match
// service_traits::MD5Sum< ::lightrover_ros::Wrc201Msg >
template<>
struct MD5Sum< ::lightrover_ros::Wrc201MsgRequest>
{
  static const char* value()
  {
    return MD5Sum< ::lightrover_ros::Wrc201Msg >::value();
  }
  static const char* value(const ::lightrover_ros::Wrc201MsgRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::lightrover_ros::Wrc201MsgRequest> should match
// service_traits::DataType< ::lightrover_ros::Wrc201Msg >
template<>
struct DataType< ::lightrover_ros::Wrc201MsgRequest>
{
  static const char* value()
  {
    return DataType< ::lightrover_ros::Wrc201Msg >::value();
  }
  static const char* value(const ::lightrover_ros::Wrc201MsgRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::lightrover_ros::Wrc201MsgResponse> should match
// service_traits::MD5Sum< ::lightrover_ros::Wrc201Msg >
template<>
struct MD5Sum< ::lightrover_ros::Wrc201MsgResponse>
{
  static const char* value()
  {
    return MD5Sum< ::lightrover_ros::Wrc201Msg >::value();
  }
  static const char* value(const ::lightrover_ros::Wrc201MsgResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::lightrover_ros::Wrc201MsgResponse> should match
// service_traits::DataType< ::lightrover_ros::Wrc201Msg >
template<>
struct DataType< ::lightrover_ros::Wrc201MsgResponse>
{
  static const char* value()
  {
    return DataType< ::lightrover_ros::Wrc201Msg >::value();
  }
  static const char* value(const ::lightrover_ros::Wrc201MsgResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LIGHTROVER_ROS_MESSAGE_WRC201MSG_H
