// Generated by gencpp from file move_ALEX/move2order.msg
// DO NOT EDIT!


#ifndef MOVE_ALEX_MESSAGE_MOVE2ORDER_H
#define MOVE_ALEX_MESSAGE_MOVE2ORDER_H


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
struct move2order_
{
  typedef move2order_<ContainerAllocator> Type;

  move2order_()
    : end_flag(false)
    , pX(0.0)
    , pY(0.0)
    , pZ(0.0)
    , rX(0.0)
    , rY(0.0)
    , rZ(0.0)  {
    }
  move2order_(const ContainerAllocator& _alloc)
    : end_flag(false)
    , pX(0.0)
    , pY(0.0)
    , pZ(0.0)
    , rX(0.0)
    , rY(0.0)
    , rZ(0.0)  {
  (void)_alloc;
    }



   typedef uint8_t _end_flag_type;
  _end_flag_type end_flag;

   typedef double _pX_type;
  _pX_type pX;

   typedef double _pY_type;
  _pY_type pY;

   typedef double _pZ_type;
  _pZ_type pZ;

   typedef double _rX_type;
  _rX_type rX;

   typedef double _rY_type;
  _rY_type rY;

   typedef double _rZ_type;
  _rZ_type rZ;





  typedef boost::shared_ptr< ::move_ALEX::move2order_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::move_ALEX::move2order_<ContainerAllocator> const> ConstPtr;

}; // struct move2order_

typedef ::move_ALEX::move2order_<std::allocator<void> > move2order;

typedef boost::shared_ptr< ::move_ALEX::move2order > move2orderPtr;
typedef boost::shared_ptr< ::move_ALEX::move2order const> move2orderConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::move_ALEX::move2order_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::move_ALEX::move2order_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::move_ALEX::move2order_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::move_ALEX::move2order_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_ALEX::move2order_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::move_ALEX::move2order_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_ALEX::move2order_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::move_ALEX::move2order_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::move_ALEX::move2order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ee0967bce06f2ab9f171adc4cabe9f72";
  }

  static const char* value(const ::move_ALEX::move2order_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xee0967bce06f2ab9ULL;
  static const uint64_t static_value2 = 0xf171adc4cabe9f72ULL;
};

template<class ContainerAllocator>
struct DataType< ::move_ALEX::move2order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "move_ALEX/move2order";
  }

  static const char* value(const ::move_ALEX::move2order_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::move_ALEX::move2order_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool end_flag\n\
float64 pX\n\
float64 pY\n\
float64 pZ\n\
float64 rX\n\
float64 rY\n\
float64 rZ\n\
\n\
";
  }

  static const char* value(const ::move_ALEX::move2order_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::move_ALEX::move2order_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.end_flag);
      stream.next(m.pX);
      stream.next(m.pY);
      stream.next(m.pZ);
      stream.next(m.rX);
      stream.next(m.rY);
      stream.next(m.rZ);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct move2order_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::move_ALEX::move2order_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::move_ALEX::move2order_<ContainerAllocator>& v)
  {
    s << indent << "end_flag: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.end_flag);
    s << indent << "pX: ";
    Printer<double>::stream(s, indent + "  ", v.pX);
    s << indent << "pY: ";
    Printer<double>::stream(s, indent + "  ", v.pY);
    s << indent << "pZ: ";
    Printer<double>::stream(s, indent + "  ", v.pZ);
    s << indent << "rX: ";
    Printer<double>::stream(s, indent + "  ", v.rX);
    s << indent << "rY: ";
    Printer<double>::stream(s, indent + "  ", v.rY);
    s << indent << "rZ: ";
    Printer<double>::stream(s, indent + "  ", v.rZ);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOVE_ALEX_MESSAGE_MOVE2ORDER_H