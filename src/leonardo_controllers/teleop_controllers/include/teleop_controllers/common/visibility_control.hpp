// Copyright 2026
// Licensed under Apache 2.0

#ifndef TELEOP_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_
#define TELEOP_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_

// ==========================================
// Windows (MSVC & MinGW)
// ==========================================
#if defined(_WIN32) || defined(__CYGWIN__)

#ifdef TELEOP_CONTROLLERS_BUILDING_DLL
#ifdef __GNUC__
#define TELEOP_CONTROLLERS_PUBLIC __attribute__((dllexport))
#else
#define TELEOP_CONTROLLERS_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define TELEOP_CONTROLLERS_PUBLIC __attribute__((dllimport))
#else
#define TELEOP_CONTROLLERS_PUBLIC __declspec(dllimport)
#endif
#endif

#define TELEOP_CONTROLLERS_LOCAL

// ==========================================
// Linux / macOS (GCC / Clang)
// ==========================================
#else

#if defined(__has_attribute)
#if __has_attribute(visibility)
#define TELEOP_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define TELEOP_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define TELEOP_CONTROLLERS_PUBLIC
#define TELEOP_CONTROLLERS_LOCAL
#endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#define TELEOP_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define TELEOP_CONTROLLERS_LOCAL __attribute__((visibility("hidden")))
#else
#define TELEOP_CONTROLLERS_PUBLIC
#define TELEOP_CONTROLLERS_LOCAL
#endif

#endif

#endif // TELEOP_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_