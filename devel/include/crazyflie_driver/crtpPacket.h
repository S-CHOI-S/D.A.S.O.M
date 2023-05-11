// Generated by gencpp from file crazyflie_driver/crtpPacket.msg
// DO NOT EDIT!


#ifndef CRAZYFLIE_DRIVER_MESSAGE_CRTPPACKET_H
#define CRAZYFLIE_DRIVER_MESSAGE_CRTPPACKET_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace crazyflie_driver
{
template <class ContainerAllocator>
struct crtpPacket_
{
  typedef crtpPacket_<ContainerAllocator> Type;

  crtpPacket_()
    : size(0)
    , header(0)
    , data()  {
      data.assign(0);
  }
  crtpPacket_(const ContainerAllocator& _alloc)
    : size(0)
    , header(0)
    , data()  {
  (void)_alloc;
      data.assign(0);
  }



   typedef uint8_t _size_type;
  _size_type size;

   typedef uint8_t _header_type;
  _header_type header;

   typedef boost::array<uint8_t, 30>  _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::crazyflie_driver::crtpPacket_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::crazyflie_driver::crtpPacket_<ContainerAllocator> const> ConstPtr;

}; // struct crtpPacket_

typedef ::crazyflie_driver::crtpPacket_<std::allocator<void> > crtpPacket;

typedef boost::shared_ptr< ::crazyflie_driver::crtpPacket > crtpPacketPtr;
typedef boost::shared_ptr< ::crazyflie_driver::crtpPacket const> crtpPacketConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::crazyflie_driver::crtpPacket_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::crazyflie_driver::crtpPacket_<ContainerAllocator1> & lhs, const ::crazyflie_driver::crtpPacket_<ContainerAllocator2> & rhs)
{
  return lhs.size == rhs.size &&
    lhs.header == rhs.header &&
    lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::crazyflie_driver::crtpPacket_<ContainerAllocator1> & lhs, const ::crazyflie_driver::crtpPacket_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace crazyflie_driver

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::crazyflie_driver::crtpPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::crazyflie_driver::crtpPacket_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::crazyflie_driver::crtpPacket_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "211163da2417112110f499fc3a0bedf0";
  }

  static const char* value(const ::crazyflie_driver::crtpPacket_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x211163da24171121ULL;
  static const uint64_t static_value2 = 0x10f499fc3a0bedf0ULL;
};

template<class ContainerAllocator>
struct DataType< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "crazyflie_driver/crtpPacket";
  }

  static const char* value(const ::crazyflie_driver::crtpPacket_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint8 size\n"
"uint8 header\n"
"uint8[30] data\n"
;
  }

  static const char* value(const ::crazyflie_driver::crtpPacket_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.size);
      stream.next(m.header);
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct crtpPacket_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::crazyflie_driver::crtpPacket_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::crazyflie_driver::crtpPacket_<ContainerAllocator>& v)
  {
    s << indent << "size: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.size);
    s << indent << "header: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.header);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CRAZYFLIE_DRIVER_MESSAGE_CRTPPACKET_H