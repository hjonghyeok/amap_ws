// Generated by gencpp from file ublox_msgs/NavVELECEF.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_NAVVELECEF_H
#define UBLOX_MSGS_MESSAGE_NAVVELECEF_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace ublox_msgs
{
template <class ContainerAllocator>
struct NavVELECEF_
{
  typedef NavVELECEF_<ContainerAllocator> Type;

  NavVELECEF_()
    : iTOW(0)
    , ecefVX(0)
    , ecefVY(0)
    , ecefVZ(0)
    , sAcc(0)  {
    }
  NavVELECEF_(const ContainerAllocator& _alloc)
    : iTOW(0)
    , ecefVX(0)
    , ecefVY(0)
    , ecefVZ(0)
    , sAcc(0)  {
  (void)_alloc;
    }



   typedef uint32_t _iTOW_type;
  _iTOW_type iTOW;

   typedef int32_t _ecefVX_type;
  _ecefVX_type ecefVX;

   typedef int32_t _ecefVY_type;
  _ecefVY_type ecefVY;

   typedef int32_t _ecefVZ_type;
  _ecefVZ_type ecefVZ;

   typedef uint32_t _sAcc_type;
  _sAcc_type sAcc;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif

  enum {
    CLASS_ID = 1u,
    MESSAGE_ID = 17u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::NavVELECEF_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::NavVELECEF_<ContainerAllocator> const> ConstPtr;

}; // struct NavVELECEF_

typedef ::ublox_msgs::NavVELECEF_<std::allocator<void> > NavVELECEF;

typedef boost::shared_ptr< ::ublox_msgs::NavVELECEF > NavVELECEFPtr;
typedef boost::shared_ptr< ::ublox_msgs::NavVELECEF const> NavVELECEFConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::NavVELECEF_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::NavVELECEF_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavVELECEF_<ContainerAllocator2> & rhs)
{
  return lhs.iTOW == rhs.iTOW &&
    lhs.ecefVX == rhs.ecefVX &&
    lhs.ecefVY == rhs.ecefVY &&
    lhs.ecefVZ == rhs.ecefVZ &&
    lhs.sAcc == rhs.sAcc;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::NavVELECEF_<ContainerAllocator1> & lhs, const ::ublox_msgs::NavVELECEF_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::NavVELECEF_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::NavVELECEF_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::NavVELECEF_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "97299f597364a39a6c0e96ed2ee4e702";
  }

  static const char* value(const ::ublox_msgs::NavVELECEF_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x97299f597364a39aULL;
  static const uint64_t static_value2 = 0x6c0e96ed2ee4e702ULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/NavVELECEF";
  }

  static const char* value(const ::ublox_msgs::NavVELECEF_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# NAV-VELECEF (0x01 0x11)\n"
"# Velocity Solution in ECEF\n"
"#\n"
"# See important comments concerning validity of velocity given in section\n"
"# Navigation Output Filters.\n"
"#\n"
"\n"
"uint8 CLASS_ID = 1\n"
"uint8 MESSAGE_ID = 17\n"
"\n"
"uint32 iTOW             # GPS Millisecond time of week [ms]\n"
"\n"
"int32 ecefVX            # ECEF X velocity [cm/s]\n"
"int32 ecefVY            # ECEF Y velocity [cm/s]\n"
"int32 ecefVZ            # ECEF Z velocity [cm/s]\n"
"uint32 sAcc             # Speed Accuracy Estimate [cm/s]\n"
;
  }

  static const char* value(const ::ublox_msgs::NavVELECEF_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.iTOW);
      stream.next(m.ecefVX);
      stream.next(m.ecefVY);
      stream.next(m.ecefVZ);
      stream.next(m.sAcc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavVELECEF_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::NavVELECEF_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::NavVELECEF_<ContainerAllocator>& v)
  {
    s << indent << "iTOW: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.iTOW);
    s << indent << "ecefVX: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefVX);
    s << indent << "ecefVY: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefVY);
    s << indent << "ecefVZ: ";
    Printer<int32_t>::stream(s, indent + "  ", v.ecefVZ);
    s << indent << "sAcc: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.sAcc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_NAVVELECEF_H
