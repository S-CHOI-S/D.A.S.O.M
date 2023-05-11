// Generated by gencpp from file test/Torqbian.msg
// DO NOT EDIT!


#ifndef TEST_MESSAGE_TORQBIAN_H
#define TEST_MESSAGE_TORQBIAN_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace test
{
template <class ContainerAllocator>
struct Torqbian_
{
  typedef Torqbian_<ContainerAllocator> Type;

  Torqbian_()
    : header()
    , X(0.0)
    , Y(0.0)
    , cmd_position()  {
    }
  Torqbian_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , X(0.0)
    , Y(0.0)
    , cmd_position(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef double _X_type;
  _X_type X;

   typedef double _Y_type;
  _Y_type Y;

   typedef std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> _cmd_position_type;
  _cmd_position_type cmd_position;





  typedef boost::shared_ptr< ::test::Torqbian_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::test::Torqbian_<ContainerAllocator> const> ConstPtr;

}; // struct Torqbian_

typedef ::test::Torqbian_<std::allocator<void> > Torqbian;

typedef boost::shared_ptr< ::test::Torqbian > TorqbianPtr;
typedef boost::shared_ptr< ::test::Torqbian const> TorqbianConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::test::Torqbian_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::test::Torqbian_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::test::Torqbian_<ContainerAllocator1> & lhs, const ::test::Torqbian_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.X == rhs.X &&
    lhs.Y == rhs.Y &&
    lhs.cmd_position == rhs.cmd_position;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::test::Torqbian_<ContainerAllocator1> & lhs, const ::test::Torqbian_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace test

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::test::Torqbian_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::test::Torqbian_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test::Torqbian_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::test::Torqbian_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test::Torqbian_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::test::Torqbian_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::test::Torqbian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "06d2af97fff100f9314c1ca3f4dfbf26";
  }

  static const char* value(const ::test::Torqbian_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x06d2af97fff100f9ULL;
  static const uint64_t static_value2 = 0x314c1ca3f4dfbf26ULL;
};

template<class ContainerAllocator>
struct DataType< ::test::Torqbian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "test/Torqbian";
  }

  static const char* value(const ::test::Torqbian_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::test::Torqbian_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n"
"\n"
"float64 X\n"
"float64 Y\n"
"float64[] cmd_position\n"
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
;
  }

  static const char* value(const ::test::Torqbian_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::test::Torqbian_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.X);
      stream.next(m.Y);
      stream.next(m.cmd_position);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Torqbian_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::test::Torqbian_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::test::Torqbian_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "X: ";
    Printer<double>::stream(s, indent + "  ", v.X);
    s << indent << "Y: ";
    Printer<double>::stream(s, indent + "  ", v.Y);
    s << indent << "cmd_position[]" << std::endl;
    for (size_t i = 0; i < v.cmd_position.size(); ++i)
    {
      s << indent << "  cmd_position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cmd_position[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // TEST_MESSAGE_TORQBIAN_H