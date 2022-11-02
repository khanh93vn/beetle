#ifndef ACKERMANN_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
#define ACKERMANN_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACKERMANN_DRIVE_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define ACKERMANN_DRIVE_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define ACKERMANN_DRIVE_CONTROLLER_EXPORT __declspec(dllexport)
    #define ACKERMANN_DRIVE_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACKERMANN_DRIVE_CONTROLLER_BUILDING_DLL
    #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC ACKERMANN_DRIVE_CONTROLLER_EXPORT
  #else
    #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC ACKERMANN_DRIVE_CONTROLLER_IMPORT
  #endif
  #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC_TYPE ACKERMANN_DRIVE_CONTROLLER_PUBLIC
  #define ACKERMANN_DRIVE_CONTROLLER_LOCAL
#else
  #define ACKERMANN_DRIVE_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define ACKERMANN_DRIVE_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define ACKERMANN_DRIVE_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC
    #define ACKERMANN_DRIVE_CONTROLLER_LOCAL
  #endif
  #define ACKERMANN_DRIVE_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // ACKERMANN_DRIVE_CONTROLLER__VISIBILITY_CONTROL_H_
