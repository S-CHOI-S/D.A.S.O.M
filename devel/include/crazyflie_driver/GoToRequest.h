// Generated by gencpp from file crazyflie_driver/GoToRequest.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_GOTOREQUEST_H
#define CRAZYFLIE_DRIVER_MESSAGE_GOTOREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace crazyflie_driver
{
template <class ContainerAllocator>
struct GoToRequest_
{
  typedef GoToRequest_<ContainerAllocator> Type;

  GoToRequest_()
    : groupMask(0)
    , relative(false)
    , goal()
    , yaw(0.0)
    , duration()  {
    }
  GoToRequest_(const ContainerAllocator& _alloc)
    : groupMask(0)
    , relative(false)
    , goal(_alloc)
    , yaw(0.0)
    , duration()  {
  (void)_alloc;
    }



   typedef uint8_t _groupMask_type;
  _groupMask_type groupMask;

   typedef uint8_t _relative_type;
  _relative_type relative;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _goal_type;
  _goal_type goal;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef ros::Duration _duration_type;
  _duration_type duration;





  typedef boost::shared_ptr< ::crazyflie_driver::GoToRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::GoToRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GoToRequest_

typedef ::crazyflie_driver::GoToRequest_<std::allocator<void> > GoToRequest;

typedef boost::shared_ptr< ::crazyflie_driver::GoToRequest > GoToRequestPtr;
typedef boost::shared_ptr< ::crazyflie_driver::GoToRequest const> GoToRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::GoToRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::crazyflie_driver::GoToRequest_<ContainerAllocator1> & lhs, const ::crazyflie_driver::GoToRequest_<ContainerAllocator2> & rhs)
{
  return lhs.groupMask == rhs.groupMask &&
    lhs.relative == rhs.relative &&
    lhs.goal == rhs.goal &&
    lhs.yaw == rhs.yaw &&
    lhs.duration == rhs.duration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::crazyflie_driver::GoToRequest_<ContainerAllocator1> & lhs, const ::crazyflie_driver::GoToRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::GoToRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::GoToRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::GoToRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "82856b48a972d6af023d961a655bcf75";
  }

  static const char* value(const ::crazyflie_driver::GoToRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x82856b48a972d6afULL;
  static const uint64_t static_value2 = 0x023d961a655bcf75ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/GoToRequest";
  }

  static const char* value(const ::crazyflie_driver::GoToRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 groupMask\n"
"bool relative\n"
"geometry_msgs/Point goal\n"
"float32 yaw # deg\n"
"duration duration\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::crazyflie_driver::GoToRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.groupMask);
      stream.next(m.relative);
      stream.next(m.goal);
      stream.next(m.yaw);
      stream.next(m.duration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GoToRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::GoToRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::GoToRequest_<ContainerAllocator>& v)
  {
    s << indent << "groupMask: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.groupMask);
    s << indent << "relative: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.relative);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "duration: ";
    Printer<ros::Duration>::stream(s, indent + "  ", v.duration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_GOTOREQUEST_H
