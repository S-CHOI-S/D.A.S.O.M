// Generated by gencpp from file two_link/paramRequest.msg
// DO NOT EDIT!


#ifndef TWO_LINK_MESSAGE_PARAMREQUEST_H
#define TWO_LINK_MESSAGE_PARAMREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace two_link
{
template <class ContainerAllocator>
struct paramRequest_
{
  typedef paramRequest_<ContainerAllocator> Type;

  paramRequest_()
    : amplitude(0.0)
    , period(0.0)  {
    }
  paramRequest_(const ContainerAllocator& _alloc)
    : amplitude(0.0)
    , period(0.0)  {
  (void)_alloc;
    }



   typedef double _amplitude_type;
  _amplitude_type amplitude;

   typedef double _period_type;
  _period_type period;





  typedef boost::shared_ptr< ::two_link::paramRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::two_link::paramRequest_<ContainerAllocator> const> ConstPtr;

}; // struct paramRequest_

typedef ::two_link::paramRequest_<std::allocator<void> > paramRequest;

typedef boost::shared_ptr< ::two_link::paramRequest > paramRequestPtr;
typedef boost::shared_ptr< ::two_link::paramRequest const> paramRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::two_link::paramRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::two_link::paramRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::two_link::paramRequest_<ContainerAllocator1> & lhs, const ::two_link::paramRequest_<ContainerAllocator2> & rhs)
{
  return lhs.amplitude == rhs.amplitude &&
    lhs.period == rhs.period;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::two_link::paramRequest_<ContainerAllocator1> & lhs, const ::two_link::paramRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace two_link

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::two_link::paramRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::two_link::paramRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::two_link::paramRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::two_link::paramRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::two_link::paramRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::two_link::paramRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::two_link::paramRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da3e0ad9469889454218ee3b1f92c73a";
  }

  static const char* value(const ::two_link::paramRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda3e0ad946988945ULL;
  static const uint64_t static_value2 = 0x4218ee3b1f92c73aULL;
};

template<class ContainerAllocator>
struct DataType< ::two_link::paramRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "two_link/paramRequest";
  }

  static const char* value(const ::two_link::paramRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::two_link::paramRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 amplitude\n"
"float64 period\n"
;
  }

  static const char* value(const ::two_link::paramRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::two_link::paramRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.amplitude);
      stream.next(m.period);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct paramRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::two_link::paramRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::two_link::paramRequest_<ContainerAllocator>& v)
  {
    s << indent << "amplitude: ";
    Printer<double>::stream(s, indent + "  ", v.amplitude);
    s << indent << "period: ";
    Printer<double>::stream(s, indent + "  ", v.period);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TWO_LINK_MESSAGE_PARAMREQUEST_H
