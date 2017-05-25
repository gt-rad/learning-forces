#ifndef APP_CPPCOMMON_H
#define APP_CPPCOMMON_H

#include <iostream>
#include <string>
using std::cout;
using std::cerr;
using std::endl;

////////////////////////////////////////////////////////////
// Boost Libraries
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>

using namespace boost::assign;
#define FOREACH         BOOST_FOREACH
#define REVERSE_FOREACH BOOST_REVERSE_FOREACH
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Google Libraries
#include <glog/logging.h>
//using google::INFO;
//using google::WARNING;
//using google::ERROR;
//using google::FATAL;
// #include <gflags/gflags.h>
#define CHECK_NOTREACHED() do { LOG(FATAL) << "Should not be reached"; } while (false)
// ////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// Simple Macros
#ifdef __linux
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
#elif defined(__APPLE__)
#define FUNCTION_NAME() (__PRETTY_FUNCTION__)
//#error "Define your own FUNCTION_NAME() macro on Apple"
#elif defined(_WIN32)
// #define FUNCTION_NAME() (__FUNCSIG__)
#define FUNCTION_NAME() (__FUNCTION__)
#else
#error "What's your operating system?"
#endif
////////////////////////////////////////////////////////////

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define CONFINE(a, lo, hi) (((a)<(lo))?(lo):(  (((a)>(hi))?(hi):(a))           ))

#define DATA_DIR "../../data"

#define UTILS_PI 3.14159265359

// #include "Paths.h"

#endif // #ifndef APP_CPPCOMMON_H
