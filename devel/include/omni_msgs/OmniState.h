// Generated by gencpp from file omni_msgs/OmniState.msg
// DO NOT EDIT!


#ifndef OMNI_MSGS_MESSAGE_OMNISTATE_H
#define OMNI_MSGS_MESSAGE_OMNISTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3.h>

namespace omni_msgs
{
template <class ContainerAllocator>
struct OmniState_
{
  typedef OmniState_<ContainerAllocator> Type;

  OmniState_()
    : header()
    , locked(false)
    , close_gripper(false)
    , pose()
    , current()
    , velocity()  {
    }
  OmniState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , locked(false)
    , close_gripper(false)
    , pose(_alloc)
    , current(_alloc)
    , velocity(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef uint8_t _locked_type;
  _locked_type locked;

   typedef uint8_t _close_gripper_type;
  _close_gripper_type close_gripper;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _current_type;
  _current_type current;

   typedef  ::geometry_msgs::Vector3_<ContainerAllocator>  _velocity_type;
  _velocity_type velocity;





  typedef boost::shared_ptr< ::omni_msgs::OmniState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::omni_msgs::OmniState_<ContainerAllocator> const> ConstPtr;

}; // struct OmniState_

typedef ::omni_msgs::OmniState_<std::allocator<void> > OmniState;

typedef boost::shared_ptr< ::omni_msgs::OmniState > OmniStatePtr;
typedef boost::shared_ptr< ::omni_msgs::OmniState const> OmniStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::omni_msgs::OmniState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::omni_msgs::OmniState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::omni_msgs::OmniState_<ContainerAllocator1> & lhs, const ::omni_msgs::OmniState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.locked == rhs.locked &&
    lhs.close_gripper == rhs.close_gripper &&
    lhs.pose == rhs.pose &&
    lhs.current == rhs.current &&
    lhs.velocity == rhs.velocity;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::omni_msgs::OmniState_<ContainerAllocator1> & lhs, const ::omni_msgs::OmniState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace omni_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::omni_msgs::OmniState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::omni_msgs::OmniState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::omni_msgs::OmniState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::omni_msgs::OmniState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_msgs::OmniState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::omni_msgs::OmniState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::omni_msgs::OmniState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "89c2a741de66e9e904f59a02b171dd6e";
  }

  static const char* value(const ::omni_msgs::OmniState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x89c2a741de66e9e9ULL;
  static const uint64_t static_value2 = 0x04f59a02b171dd6eULL;
};

template<class ContainerAllocator>
struct DataType< ::omni_msgs::OmniState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "omni_msgs/OmniState";
  }

  static const char* value(const ::omni_msgs::OmniState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::omni_msgs::OmniState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "std_msgs/Header         header\n"
"\n"
"bool                    locked\n"
"\n"
"bool                    close_gripper\n"
"\n"
"geometry_msgs/Pose      pose        # meters\n"
"\n"
"geometry_msgs/Vector3   current     # Amperes\n"
"\n"
"geometry_msgs/Vector3   velocity    # meters/s\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::omni_msgs::OmniState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::omni_msgs::OmniState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.locked);
      stream.next(m.close_gripper);
      stream.next(m.pose);
      stream.next(m.current);
      stream.next(m.velocity);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct OmniState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::omni_msgs::OmniState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::omni_msgs::OmniState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "locked: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.locked);
    s << indent << "close_gripper: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.close_gripper);
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "current: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.current);
    s << indent << "velocity: ";
    s << std::endl;
    Printer< ::geometry_msgs::Vector3_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OMNI_MSGS_MESSAGE_OMNISTATE_H
