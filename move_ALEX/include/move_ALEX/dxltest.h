// Generated by gencpp from file move_ALEX/dxltest.msg
// DO NOT EDIT!


#ifndef MOVE_ALEX_MESSAGE_DXLTEST_H
#define MOVE_ALEX_MESSAGE_DXLTEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace move_ALEX
{
template <class ContainerAllocator>
struct dxltest_
{
  typedef dxltest_<ContainerAllocator> Type;

  dxltest_()
    : dxl_0(0.0)
    , dxl_1(0.0)
    , dxl_2(0.0)
    , dxl_3(0.0)
    , dxl_4(0.0)
    , dxl_5(0.0)  {
    }
  dxltest_(const ContainerAllocator& _alloc)
    : dxl_0(0.0)
    , dxl_1(0.0)
    , dxl_2(0.0)
    , dxl_3(0.0)
    , dxl_4(0.0)
    , dxl_5(0.0)  {
  (void)_alloc;
    }



   typedef float _dxl_0_type;
  _dxl_0_type dxl_0;

   typedef float _dxl_1_type;
  _dxl_1_type dxl_1;

   typedef float _dxl_2_type;
  _dxl_2_type dxl_2;

   typedef float _dxl_3_type;
  _dxl_3_type dxl_3;

   typedef float _dxl_4_type;
  _dxl_4_type dxl_4;

   typedef float _dxl_5_type;
  _dxl_5_type dxl_5;





  typedef boost::shared_ptr< ::move_ALEX::dxltest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::move_ALEX::dxltest_<ContainerAllocator> const> ConstPtr;

}; // struct dxltest_

typedef ::move_ALEX::dxltest_<std::allocator<void> > dxltest;

typedef boost::shared_ptr< ::move_ALEX::dxltest > dxltestPtr;
typedef boost::shared_ptr< ::move_ALEX::dxltest const> dxltestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::move_ALEX::dxltest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::move_ALEX::dxltest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace move_ALEX

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'move_ALEX': ['/home/robit/catkin_ws/src/move_ALEX/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::move_ALEX::dxltest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_ALEX::dxltest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_ALEX::dxltest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_ALEX::dxltest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_ALEX::dxltest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_ALEX::dxltest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::move_ALEX::dxltest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "51334e504d8ff5ea9bf38b886a148f92";
  }

  static const char* value(const ::move_ALEX::dxltest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x51334e504d8ff5eaULL;
  static const uint64_t static_value2 = 0x9bf38b886a148f92ULL;
};

template<class ContainerAllocator>
struct DataType< ::move_ALEX::dxltest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "move_ALEX/dxltest";
  }

  static const char* value(const ::move_ALEX::dxltest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::move_ALEX::dxltest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32 dxl_0\n\
float32 dxl_1\n\
float32 dxl_2\n\
float32 dxl_3\n\
float32 dxl_4\n\
float32 dxl_5\n\
";
  }

  static const char* value(const ::move_ALEX::dxltest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::move_ALEX::dxltest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.dxl_0);
      stream.next(m.dxl_1);
      stream.next(m.dxl_2);
      stream.next(m.dxl_3);
      stream.next(m.dxl_4);
      stream.next(m.dxl_5);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct dxltest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::move_ALEX::dxltest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::move_ALEX::dxltest_<ContainerAllocator>& v)
  {
    s << indent << "dxl_0: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_0);
    s << indent << "dxl_1: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_1);
    s << indent << "dxl_2: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_2);
    s << indent << "dxl_3: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_3);
    s << indent << "dxl_4: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_4);
    s << indent << "dxl_5: ";
    Printer<float>::stream(s, indent + "  ", v.dxl_5);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVE_ALEX_MESSAGE_DXLTEST_H
