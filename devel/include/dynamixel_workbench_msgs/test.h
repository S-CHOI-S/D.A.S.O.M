// Generated by gencpp from file dynamixel_workbench_msgs/test.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_TEST_H
#define DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_TEST_H

#include <ros/service_traits.h>


#include <dynamixel_workbench_msgs/testRequest.h>
#include <dynamixel_workbench_msgs/testResponse.h>


namespace dynamixel_workbench_msgs
{

struct test
{

typedef testRequest Request;
typedef testResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct test
} // namespace dynamixel_workbench_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamixel_workbench_msgs::test > {
  static const char* value()
  {
    return "a7851f940124222501142592f81f3c11";
  }

  static const char* value(const ::dynamixel_workbench_msgs::test&) { return value(); }
};

template<>
struct DataType< ::dynamixel_workbench_msgs::test > {
  static const char* value()
  {
    return "dynamixel_workbench_msgs/test";
  }

  static const char* value(const ::dynamixel_workbench_msgs::test&) { return value(); }
};


// service_traits::MD5Sum< ::dynamixel_workbench_msgs::testRequest> should match
// service_traits::MD5Sum< ::dynamixel_workbench_msgs::test >
template<>
struct MD5Sum< ::dynamixel_workbench_msgs::testRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_workbench_msgs::test >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::testRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_workbench_msgs::testRequest> should match
// service_traits::DataType< ::dynamixel_workbench_msgs::test >
template<>
struct DataType< ::dynamixel_workbench_msgs::testRequest>
{
  static const char* value()
  {
    return DataType< ::dynamixel_workbench_msgs::test >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::testRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamixel_workbench_msgs::testResponse> should match
// service_traits::MD5Sum< ::dynamixel_workbench_msgs::test >
template<>
struct MD5Sum< ::dynamixel_workbench_msgs::testResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_workbench_msgs::test >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::testResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_workbench_msgs::testResponse> should match
// service_traits::DataType< ::dynamixel_workbench_msgs::test >
template<>
struct DataType< ::dynamixel_workbench_msgs::testResponse>
{
  static const char* value()
  {
    return DataType< ::dynamixel_workbench_msgs::test >::value();
  }
  static const char* value(const ::dynamixel_workbench_msgs::testResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_TEST_H
