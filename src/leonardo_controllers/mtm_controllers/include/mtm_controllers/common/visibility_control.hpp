// Copyright 2026
// Licensed under Apache 2.0

#ifndef MTM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_
#define MTM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_

// ==========================================
// Windows (MSVC & MinGW)
// ==========================================
#if defined(_WIN32) || defined(__CYGWIN__)

#ifdef MTM_CONTROLLERS_BUILDING_DLL
#ifdef __GNUC__
#define MTM_CONTROLLERS_PUBLIC __attribute__((dllexport))
#else
#define MTM_CONTROLLERS_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define MTM_CONTROLLERS_PUBLIC __attribute__((dllimport))
#else
#define MTM_CONTROLLERS_PUBLIC __declspec(dllimport)
#endif
#endif

#define MTM_CONTROLLERS_LOCAL

// ==========================================
// Linux / macOS (GCC / Clang)
// ==========================================
#else

#if defined(__has_attribute)
#if __has_attribute(visibility)
#define MTM_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define MTM_CONTROLLERS_LOCAL  __attribute__((visibility("hidden")))
#else
#define MTM_CONTROLLERS_PUBLIC
#define MTM_CONTROLLERS_LOCAL
#endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#define MTM_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define MTM_CONTROLLERS_LOCAL  __attribute__((visibility("hidden")))
#else
#define MTM_CONTROLLERS_PUBLIC
#define MTM_CONTROLLERS_LOCAL
#endif

#endif

#endif  // MTM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_
