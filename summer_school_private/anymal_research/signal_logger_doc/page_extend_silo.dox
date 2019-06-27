/*! \page page_extend_silo Extend the signal logger with additional log types

<H3>Add a new data type to the logger</H3>
In order to add a type you have to provide a complete set of traits in order to
publish and save a log element of that type. The ros logger traits have to be added signal_logger_ros_traits.hpp and signal_logger_std_traits.hpp.
Moreover it makes sense to add this new type to LogElementTypes.hpp.

<i>NOTE</i>: The buffer is templated on the log type. If you use a type the buffer can not handle, you can also
try to add your new type to the buffer customization in signal_logger/Buffer.hpp.

<H3>Example: Add a Circle type to the signal logger</H3>

<H5> 1. Add your type to the logger type header </H5>
LogElementTypes.hpp :
\code{c}
struct Circle {
  double diameter;
  Eigen::Vector2d center;
};
\endcode

<H5> 2. Add type to the std logger</H5>
The std logger traits are written in a way, such that reusing the trait in other "wrapper"-types is possible.
Every function receives an accessor function that returns a pointer to a value type (in this case signal_logger::Circle) from a pointer
to a ContainerType_ type. In your custom trait you can therefore reuse built-in or custom traits. Provide a lambda that accesses the
corresponding entry and pass it to the trait.

signal_logger_std_traits.hpp :
\code{c}
template<typename ContainerType_>
struct sls_traits<signal_logger::Circle, ContainerType_>
{
  static void writeLogElementToStreams(std::stringstream* text,
                                       std::stringstream* binary,
                                       signal_logger::LogFileType fileType,
                                       const signal_logger::Buffer<ContainerType_> & buffer,
                                       const std::string & name,
                                       const std::size_t divider,
                                       const unsigned int startDiff,
                                       const unsigned int endDiff,
                                       const std::function<const signal_logger::Circle *(const ContainerType_ * const)> & accessor = [](const ContainerType_ * const v) { return v; })
  {
    // Use already defined double type trait to write the diameter
    auto getDiameter = [accessor](const ContainerType_ * const v) { return &(accessor(v)->diameter); };
    sls_traits<double, ContainerType_>::writeLogElementToStreams(
      text, binary, fileType, buffer, name + "_diameter", divider, startDiff, endDiff, getDiameter);

    // Use already defined eigen matrix type trait to write the center
    auto getCenter = [accessor](const ContainerType_ * const v) { return &(accessor(v)->center); };
    sls_traits<Eigen::Vector2d, ContainerType_>::writeLogElementToStreams(
      text, binary, fileType, buffer, name + "_diameter", divider, startDiff, endDiff, getCenter);
  }
};
\endcode

<H5> 3. Add type to the ros logger</H5>
For the ros logger we can add a custom msg type to signal_logger_msgs. Lets call it CircleStamped.msg.
Don't forget to add this msg to the CMakeLists.txt of the signal_logger_msgs package.
\code{yaml}
# CircleStamped
Header header
float64 diameter
float64 center_x
float64 center_y
\endcode

signal_logger_ros_traits.hpp :
\code{c}
template<>
struct slr_msg_traits<signal_logger::Circle>
{
  typedef signal_logger_msgs::CircleStamped         msgtype;
};

template <>
struct slr_update_traits<signal_logger::Circle>
{
  static void updateMsg(const signal_logger::Circle* var,
                        typename slr_msg_traits<signal_logger::Circle>::msgtype* const msg,
                        const ros::Time& timeStamp)
  {
    msg->header.stamp = timeStamp;
    msg->diameter = var->diameter;
    msg->center_x = var->center(0);
    msg->center_y = var->center(1);
  }
};
\endcode


<H3>Add a new data type custom to your project to the logger</H3>
When adding a type would add unnecessary dependencies to the signal_logger or the log type is very specific to your application, you should not add this type to
the signal logger itself.
However, due to the trait based nature of the signal_logger you can simply add a header file containing the additional traits.
Make sure to use the proper namespaces.

This include file contains the additional traits for the signal logger (see Circle example from above).<BR>

my_circle_trait.hpp:
\code{c}
namespace signal_logger_std {
  namespace traits {
    // Std trait from above example
  }
}

namespace signal_logger_ros {
  namespace traits {
    // Ros update and msg traits from above example
  }
}
\endcode

After including this header and "signal_logger.hpp" you can add your custom circle type to the signal logger.

*/
