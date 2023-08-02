// Generated by gencpp from file ublox_msgs/RxmSFRBX.msg
// DO NOT EDIT!


#ifndef UBLOX_MSGS_MESSAGE_RXMSFRBX_H
#define UBLOX_MSGS_MESSAGE_RXMSFRBX_H


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
struct RxmSFRBX_
{
  typedef RxmSFRBX_<ContainerAllocator> Type;

  RxmSFRBX_()
    : gnssId(0)
    , svId(0)
    , reserved0(0)
    , freqId(0)
    , numWords(0)
    , chn(0)
    , version(0)
    , reserved1(0)
    , dwrd()  {
    }
  RxmSFRBX_(const ContainerAllocator& _alloc)
    : gnssId(0)
    , svId(0)
    , reserved0(0)
    , freqId(0)
    , numWords(0)
    , chn(0)
    , version(0)
    , reserved1(0)
    , dwrd(_alloc)  {
  (void)_alloc;
    }



   typedef uint8_t _gnssId_type;
  _gnssId_type gnssId;

   typedef uint8_t _svId_type;
  _svId_type svId;

   typedef uint8_t _reserved0_type;
  _reserved0_type reserved0;

   typedef uint8_t _freqId_type;
  _freqId_type freqId;

   typedef uint8_t _numWords_type;
  _numWords_type numWords;

   typedef uint8_t _chn_type;
  _chn_type chn;

   typedef uint8_t _version_type;
  _version_type version;

   typedef uint8_t _reserved1_type;
  _reserved1_type reserved1;

   typedef std::vector<uint32_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint32_t>> _dwrd_type;
  _dwrd_type dwrd;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(CLASS_ID)
  #undef CLASS_ID
#endif
#if defined(_WIN32) && defined(MESSAGE_ID)
  #undef MESSAGE_ID
#endif

  enum {
    CLASS_ID = 2u,
    MESSAGE_ID = 19u,
  };


  typedef boost::shared_ptr< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> const> ConstPtr;

}; // struct RxmSFRBX_

typedef ::ublox_msgs::RxmSFRBX_<std::allocator<void> > RxmSFRBX;

typedef boost::shared_ptr< ::ublox_msgs::RxmSFRBX > RxmSFRBXPtr;
typedef boost::shared_ptr< ::ublox_msgs::RxmSFRBX const> RxmSFRBXConstPtr;

// constants requiring out of line definition

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::ublox_msgs::RxmSFRBX_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::ublox_msgs::RxmSFRBX_<ContainerAllocator1> & lhs, const ::ublox_msgs::RxmSFRBX_<ContainerAllocator2> & rhs)
{
  return lhs.gnssId == rhs.gnssId &&
    lhs.svId == rhs.svId &&
    lhs.reserved0 == rhs.reserved0 &&
    lhs.freqId == rhs.freqId &&
    lhs.numWords == rhs.numWords &&
    lhs.chn == rhs.chn &&
    lhs.version == rhs.version &&
    lhs.reserved1 == rhs.reserved1 &&
    lhs.dwrd == rhs.dwrd;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::ublox_msgs::RxmSFRBX_<ContainerAllocator1> & lhs, const ::ublox_msgs::RxmSFRBX_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace ublox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "c76473d828cc8e80de3a2d83cd6b9dff";
  }

  static const char* value(const ::ublox_msgs::RxmSFRBX_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xc76473d828cc8e80ULL;
  static const uint64_t static_value2 = 0xde3a2d83cd6b9dffULL;
};

template<class ContainerAllocator>
struct DataType< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ublox_msgs/RxmSFRBX";
  }

  static const char* value(const ::ublox_msgs::RxmSFRBX_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# RXM-SFRB (0x02 0x13)\n"
"# Subframe Buffer\n"
"#\n"
"# This message reports a complete subframe of broadcast navigation data decoded \n"
"# from a single signal. The number of data words reported in each message\n"
"# depends on the nature of the signal. See the section on Broadcast Navigation\n"
"# Data for further details.\n"
"#\n"
"\n"
"uint8 CLASS_ID = 2\n"
"uint8 MESSAGE_ID = 19\n"
"\n"
"uint8 gnssId            # GNSS identifier (see Cfg GNSS for constants)\n"
"\n"
"uint8 svId              # Satellite identifier within corresponding GNSS system\n"
"uint8 reserved0         # Reserved\n"
"uint8 freqId            # Only used for GLONASS: This is the frequency\n"
"                        # slot + 7 (range from 0 to 13)\n"
"uint8 numWords          # The number of data words contained in this message (up\n"
"                        # to 10, for currently supported signals)\n"
"uint8 chn               # The tracking channel number the message was received\n"
"                        # on\n"
"uint8 version           # Message version, (0x02 for this version)\n"
"uint8 reserved1         # Reserved\n"
"\n"
"# Start of repeated block (numWords times)\n"
"uint32[] dwrd           # The data words\n"
"# End of repeated block\n"
;
  }

  static const char* value(const ::ublox_msgs::RxmSFRBX_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.gnssId);
      stream.next(m.svId);
      stream.next(m.reserved0);
      stream.next(m.freqId);
      stream.next(m.numWords);
      stream.next(m.chn);
      stream.next(m.version);
      stream.next(m.reserved1);
      stream.next(m.dwrd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RxmSFRBX_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::ublox_msgs::RxmSFRBX_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::ublox_msgs::RxmSFRBX_<ContainerAllocator>& v)
  {
    s << indent << "gnssId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.gnssId);
    s << indent << "svId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.svId);
    s << indent << "reserved0: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved0);
    s << indent << "freqId: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.freqId);
    s << indent << "numWords: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.numWords);
    s << indent << "chn: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.chn);
    s << indent << "version: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.version);
    s << indent << "reserved1: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.reserved1);
    s << indent << "dwrd[]" << std::endl;
    for (size_t i = 0; i < v.dwrd.size(); ++i)
    {
      s << indent << "  dwrd[" << i << "]: ";
      Printer<uint32_t>::stream(s, indent + "  ", v.dwrd[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // UBLOX_MSGS_MESSAGE_RXMSFRBX_H
