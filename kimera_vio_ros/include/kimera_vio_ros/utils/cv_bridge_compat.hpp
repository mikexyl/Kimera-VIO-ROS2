#ifndef KIMERA_VIO_ROS__UTILS__CV_BRIDGE_COMPAT_HPP_
#define KIMERA_VIO_ROS__UTILS__CV_BRIDGE_COMPAT_HPP_

#if defined(__has_include)
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
#include <cv_bridge/cv_bridge.h>
#else
#error "cv_bridge headers were not found"
#endif
#else
#include <cv_bridge/cv_bridge.h>
#endif

#endif  // KIMERA_VIO_ROS__UTILS__CV_BRIDGE_COMPAT_HPP_
