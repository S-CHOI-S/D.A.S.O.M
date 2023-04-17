// Generated by gencpp from file dynamixel_workbench_msgs/MXExt.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_MXEXT_H
#define DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_MXEXT_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_workbench_msgs
{
template <class ContainerAllocator>
struct MXExt_
{
  typedef MXExt_<ContainerAllocator> Type;

  MXExt_()
    : Model_Number(0)
    , Firmware_Version(0)
    , ID(0)
    , Baud_Rate(0)
    , Return_Delay_Time(0)
    , CW_Angle_Limit(0)
    , CCW_Angle_Limit(0)
    , Temperature_Limit(0)
    , Min_Voltage_Limit(0)
    , Max_Voltage_Limit(0)
    , Max_Torque(0)
    , Status_Return_Level(0)
    , Alarm_LED(0)
    , Shutdown(0)
    , Multi_Turn_Offset(0)
    , Resolution_Divider(0)
    , Torque_Enable(0)
    , LED(0)
    , D_gain(0)
    , I_gain(0)
    , P_gain(0)
    , Goal_Position(0)
    , Moving_Speed(0)
    , Torque_Limit(0)
    , Present_Position(0)
    , Present_Speed(0)
    , Present_Load(0)
    , Present_Voltage(0)
    , Present_Temperature(0)
    , Registered(0)
    , Moving(0)
    , Lock(0)
    , Punch(0)
    , Current(0)
    , Torque_Control_Mode_Enable(0)
    , Goal_Torque(0)
    , Goal_Acceleration(0)  {
    }
  MXExt_(const ContainerAllocator& _alloc)
    : Model_Number(0)
    , Firmware_Version(0)
    , ID(0)
    , Baud_Rate(0)
    , Return_Delay_Time(0)
    , CW_Angle_Limit(0)
    , CCW_Angle_Limit(0)
    , Temperature_Limit(0)
    , Min_Voltage_Limit(0)
    , Max_Voltage_Limit(0)
    , Max_Torque(0)
    , Status_Return_Level(0)
    , Alarm_LED(0)
    , Shutdown(0)
    , Multi_Turn_Offset(0)
    , Resolution_Divider(0)
    , Torque_Enable(0)
    , LED(0)
    , D_gain(0)
    , I_gain(0)
    , P_gain(0)
    , Goal_Position(0)
    , Moving_Speed(0)
    , Torque_Limit(0)
    , Present_Position(0)
    , Present_Speed(0)
    , Present_Load(0)
    , Present_Voltage(0)
    , Present_Temperature(0)
    , Registered(0)
    , Moving(0)
    , Lock(0)
    , Punch(0)
    , Current(0)
    , Torque_Control_Mode_Enable(0)
    , Goal_Torque(0)
    , Goal_Acceleration(0)  {
  (void)_alloc;
    }



   typedef uint16_t _Model_Number_type;
  _Model_Number_type Model_Number;

   typedef uint8_t _Firmware_Version_type;
  _Firmware_Version_type Firmware_Version;

   typedef uint8_t _ID_type;
  _ID_type ID;

   typedef uint8_t _Baud_Rate_type;
  _Baud_Rate_type Baud_Rate;

   typedef uint8_t _Return_Delay_Time_type;
  _Return_Delay_Time_type Return_Delay_Time;

   typedef uint16_t _CW_Angle_Limit_type;
  _CW_Angle_Limit_type CW_Angle_Limit;

   typedef uint16_t _CCW_Angle_Limit_type;
  _CCW_Angle_Limit_type CCW_Angle_Limit;

   typedef uint8_t _Temperature_Limit_type;
  _Temperature_Limit_type Temperature_Limit;

   typedef uint8_t _Min_Voltage_Limit_type;
  _Min_Voltage_Limit_type Min_Voltage_Limit;

   typedef uint8_t _Max_Voltage_Limit_type;
  _Max_Voltage_Limit_type Max_Voltage_Limit;

   typedef uint16_t _Max_Torque_type;
  _Max_Torque_type Max_Torque;

   typedef uint8_t _Status_Return_Level_type;
  _Status_Return_Level_type Status_Return_Level;

   typedef uint8_t _Alarm_LED_type;
  _Alarm_LED_type Alarm_LED;

   typedef uint8_t _Shutdown_type;
  _Shutdown_type Shutdown;

   typedef uint16_t _Multi_Turn_Offset_type;
  _Multi_Turn_Offset_type Multi_Turn_Offset;

   typedef uint8_t _Resolution_Divider_type;
  _Resolution_Divider_type Resolution_Divider;

   typedef uint8_t _Torque_Enable_type;
  _Torque_Enable_type Torque_Enable;

   typedef uint8_t _LED_type;
  _LED_type LED;

   typedef uint8_t _D_gain_type;
  _D_gain_type D_gain;

   typedef uint8_t _I_gain_type;
  _I_gain_type I_gain;

   typedef uint8_t _P_gain_type;
  _P_gain_type P_gain;

   typedef uint16_t _Goal_Position_type;
  _Goal_Position_type Goal_Position;

   typedef uint16_t _Moving_Speed_type;
  _Moving_Speed_type Moving_Speed;

   typedef uint16_t _Torque_Limit_type;
  _Torque_Limit_type Torque_Limit;

   typedef uint16_t _Present_Position_type;
  _Present_Position_type Present_Position;

   typedef uint16_t _Present_Speed_type;
  _Present_Speed_type Present_Speed;

   typedef uint16_t _Present_Load_type;
  _Present_Load_type Present_Load;

   typedef uint8_t _Present_Voltage_type;
  _Present_Voltage_type Present_Voltage;

   typedef uint8_t _Present_Temperature_type;
  _Present_Temperature_type Present_Temperature;

   typedef uint8_t _Registered_type;
  _Registered_type Registered;

   typedef uint8_t _Moving_type;
  _Moving_type Moving;

   typedef uint8_t _Lock_type;
  _Lock_type Lock;

   typedef uint16_t _Punch_type;
  _Punch_type Punch;

   typedef uint16_t _Current_type;
  _Current_type Current;

   typedef uint8_t _Torque_Control_Mode_Enable_type;
  _Torque_Control_Mode_Enable_type Torque_Control_Mode_Enable;

   typedef uint16_t _Goal_Torque_type;
  _Goal_Torque_type Goal_Torque;

   typedef uint8_t _Goal_Acceleration_type;
  _Goal_Acceleration_type Goal_Acceleration;





  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> const> ConstPtr;

}; // struct MXExt_

typedef ::dynamixel_workbench_msgs::MXExt_<std::allocator<void> > MXExt;

typedef boost::shared_ptr< ::dynamixel_workbench_msgs::MXExt > MXExtPtr;
typedef boost::shared_ptr< ::dynamixel_workbench_msgs::MXExt const> MXExtConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator1> & lhs, const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator2> & rhs)
{
  return lhs.Model_Number == rhs.Model_Number &&
    lhs.Firmware_Version == rhs.Firmware_Version &&
    lhs.ID == rhs.ID &&
    lhs.Baud_Rate == rhs.Baud_Rate &&
    lhs.Return_Delay_Time == rhs.Return_Delay_Time &&
    lhs.CW_Angle_Limit == rhs.CW_Angle_Limit &&
    lhs.CCW_Angle_Limit == rhs.CCW_Angle_Limit &&
    lhs.Temperature_Limit == rhs.Temperature_Limit &&
    lhs.Min_Voltage_Limit == rhs.Min_Voltage_Limit &&
    lhs.Max_Voltage_Limit == rhs.Max_Voltage_Limit &&
    lhs.Max_Torque == rhs.Max_Torque &&
    lhs.Status_Return_Level == rhs.Status_Return_Level &&
    lhs.Alarm_LED == rhs.Alarm_LED &&
    lhs.Shutdown == rhs.Shutdown &&
    lhs.Multi_Turn_Offset == rhs.Multi_Turn_Offset &&
    lhs.Resolution_Divider == rhs.Resolution_Divider &&
    lhs.Torque_Enable == rhs.Torque_Enable &&
    lhs.LED == rhs.LED &&
    lhs.D_gain == rhs.D_gain &&
    lhs.I_gain == rhs.I_gain &&
    lhs.P_gain == rhs.P_gain &&
    lhs.Goal_Position == rhs.Goal_Position &&
    lhs.Moving_Speed == rhs.Moving_Speed &&
    lhs.Torque_Limit == rhs.Torque_Limit &&
    lhs.Present_Position == rhs.Present_Position &&
    lhs.Present_Speed == rhs.Present_Speed &&
    lhs.Present_Load == rhs.Present_Load &&
    lhs.Present_Voltage == rhs.Present_Voltage &&
    lhs.Present_Temperature == rhs.Present_Temperature &&
    lhs.Registered == rhs.Registered &&
    lhs.Moving == rhs.Moving &&
    lhs.Lock == rhs.Lock &&
    lhs.Punch == rhs.Punch &&
    lhs.Current == rhs.Current &&
    lhs.Torque_Control_Mode_Enable == rhs.Torque_Control_Mode_Enable &&
    lhs.Goal_Torque == rhs.Goal_Torque &&
    lhs.Goal_Acceleration == rhs.Goal_Acceleration;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator1> & lhs, const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_workbench_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2fbd27c9b5d24a5196a094d168160140";
  }

  static const char* value(const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2fbd27c9b5d24a51ULL;
  static const uint64_t static_value2 = 0x96a094d168160140ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_workbench_msgs/MXExt";
  }

  static const char* value(const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# This message is compatible with control table of Dynamixel MX Series (MX-64T/MX64-R/MX-64AT/MX-64AR, MX-106T/MX-106R)\n"
"# If you want to specific information about control table, please follow the link (http://emanual.robotis.com/)\n"
"\n"
"uint16 Model_Number\n"
"uint8  Firmware_Version\n"
"uint8  ID\n"
"uint8  Baud_Rate\n"
"uint8  Return_Delay_Time\n"
"uint16 CW_Angle_Limit\n"
"uint16 CCW_Angle_Limit\n"
"uint8  Temperature_Limit\n"
"uint8  Min_Voltage_Limit\n"
"uint8  Max_Voltage_Limit\n"
"uint16 Max_Torque\n"
"uint8  Status_Return_Level\n"
"uint8  Alarm_LED\n"
"uint8  Shutdown\n"
"uint16 Multi_Turn_Offset\n"
"uint8  Resolution_Divider\n"
"\n"
"uint8  Torque_Enable\n"
"uint8  LED\n"
"uint8  D_gain\n"
"uint8  I_gain\n"
"uint8  P_gain\n"
"uint16 Goal_Position\n"
"uint16 Moving_Speed\n"
"uint16 Torque_Limit\n"
"uint16 Present_Position\n"
"uint16 Present_Speed\n"
"uint16 Present_Load\n"
"uint8  Present_Voltage\n"
"uint8  Present_Temperature\n"
"uint8  Registered\n"
"uint8  Moving\n"
"uint8  Lock\n"
"uint16 Punch\n"
"uint16 Current\n"
"uint8  Torque_Control_Mode_Enable\n"
"uint16 Goal_Torque\n"
"uint8  Goal_Acceleration\n"
;
  }

  static const char* value(const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.Model_Number);
      stream.next(m.Firmware_Version);
      stream.next(m.ID);
      stream.next(m.Baud_Rate);
      stream.next(m.Return_Delay_Time);
      stream.next(m.CW_Angle_Limit);
      stream.next(m.CCW_Angle_Limit);
      stream.next(m.Temperature_Limit);
      stream.next(m.Min_Voltage_Limit);
      stream.next(m.Max_Voltage_Limit);
      stream.next(m.Max_Torque);
      stream.next(m.Status_Return_Level);
      stream.next(m.Alarm_LED);
      stream.next(m.Shutdown);
      stream.next(m.Multi_Turn_Offset);
      stream.next(m.Resolution_Divider);
      stream.next(m.Torque_Enable);
      stream.next(m.LED);
      stream.next(m.D_gain);
      stream.next(m.I_gain);
      stream.next(m.P_gain);
      stream.next(m.Goal_Position);
      stream.next(m.Moving_Speed);
      stream.next(m.Torque_Limit);
      stream.next(m.Present_Position);
      stream.next(m.Present_Speed);
      stream.next(m.Present_Load);
      stream.next(m.Present_Voltage);
      stream.next(m.Present_Temperature);
      stream.next(m.Registered);
      stream.next(m.Moving);
      stream.next(m.Lock);
      stream.next(m.Punch);
      stream.next(m.Current);
      stream.next(m.Torque_Control_Mode_Enable);
      stream.next(m.Goal_Torque);
      stream.next(m.Goal_Acceleration);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct MXExt_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_workbench_msgs::MXExt_<ContainerAllocator>& v)
  {
    s << indent << "Model_Number: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Model_Number);
    s << indent << "Firmware_Version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Firmware_Version);
    s << indent << "ID: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.ID);
    s << indent << "Baud_Rate: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Baud_Rate);
    s << indent << "Return_Delay_Time: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Return_Delay_Time);
    s << indent << "CW_Angle_Limit: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.CW_Angle_Limit);
    s << indent << "CCW_Angle_Limit: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.CCW_Angle_Limit);
    s << indent << "Temperature_Limit: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Temperature_Limit);
    s << indent << "Min_Voltage_Limit: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Min_Voltage_Limit);
    s << indent << "Max_Voltage_Limit: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Max_Voltage_Limit);
    s << indent << "Max_Torque: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Max_Torque);
    s << indent << "Status_Return_Level: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Status_Return_Level);
    s << indent << "Alarm_LED: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Alarm_LED);
    s << indent << "Shutdown: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Shutdown);
    s << indent << "Multi_Turn_Offset: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Multi_Turn_Offset);
    s << indent << "Resolution_Divider: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Resolution_Divider);
    s << indent << "Torque_Enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Torque_Enable);
    s << indent << "LED: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.LED);
    s << indent << "D_gain: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.D_gain);
    s << indent << "I_gain: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.I_gain);
    s << indent << "P_gain: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.P_gain);
    s << indent << "Goal_Position: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Goal_Position);
    s << indent << "Moving_Speed: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Moving_Speed);
    s << indent << "Torque_Limit: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Torque_Limit);
    s << indent << "Present_Position: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Present_Position);
    s << indent << "Present_Speed: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Present_Speed);
    s << indent << "Present_Load: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Present_Load);
    s << indent << "Present_Voltage: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Present_Voltage);
    s << indent << "Present_Temperature: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Present_Temperature);
    s << indent << "Registered: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Registered);
    s << indent << "Moving: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Moving);
    s << indent << "Lock: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Lock);
    s << indent << "Punch: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Punch);
    s << indent << "Current: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Current);
    s << indent << "Torque_Control_Mode_Enable: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Torque_Control_Mode_Enable);
    s << indent << "Goal_Torque: ";
    Printer<uint16_t>::stream(s, indent + "  ", v.Goal_Torque);
    s << indent << "Goal_Acceleration: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.Goal_Acceleration);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_WORKBENCH_MSGS_MESSAGE_MXEXT_H
