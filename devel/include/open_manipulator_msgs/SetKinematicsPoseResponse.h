// Generated by gencpp from file open_manipulator_msgs/SetKinematicsPoseResponse.msg
// DO NOT EDIT!


#ifndef OPEN_MANIPULATOR_MSGS_MESSAGE_SETKINEMATICSPOSERESPONSE_H
#define OPEN_MANIPULATOR_MSGS_MESSAGE_SETKINEMATICSPOSERESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace open_manipulator_msgs
{
template <class ContainerAllocator>
struct SetKinematicsPoseResponse_
{
  typedef SetKinematicsPoseResponse_<ContainerAllocator> Type;

  SetKinematicsPoseResponse_()
    : isPlanned(false)  {
    }
  SetKinematicsPoseResponse_(const ContainerAllocator& _alloc)
    : isPlanned(false)  {
  (void)_alloc;
    }



   typedef uint8_t _isPlanned_type;
  _isPlanned_type isPlanned;





  typedef boost::shared_ptr< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SetKinematicsPoseResponse_

typedef ::open_manipulator_msgs::SetKinematicsPoseResponse_<std::allocator<void> > SetKinematicsPoseResponse;

typedef boost::shared_ptr< ::open_manipulator_msgs::SetKinematicsPoseResponse > SetKinematicsPoseResponsePtr;
typedef boost::shared_ptr< ::open_manipulator_msgs::SetKinematicsPoseResponse const> SetKinematicsPoseResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator1> & lhs, const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator2> & rhs)
{
  return lhs.isPlanned == rhs.isPlanned;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator1> & lhs, const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace open_manipulator_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c4a8e64ceeeccdab98609099e2b0c166";
  }

  static const char* value(const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc4a8e64ceeeccdabULL;
  static const uint64_t static_value2 = 0x98609099e2b0c166ULL;
};

template<class ContainerAllocator>
struct DataType< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "open_manipulator_msgs/SetKinematicsPoseResponse";
  }

  static const char* value(const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool isPlanned\n"
"\n"
;
  }

  static const char* value(const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.isPlanned);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SetKinematicsPoseResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::open_manipulator_msgs::SetKinematicsPoseResponse_<ContainerAllocator>& v)
  {
    s << indent << "isPlanned: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isPlanned);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPEN_MANIPULATOR_MSGS_MESSAGE_SETKINEMATICSPOSERESPONSE_H