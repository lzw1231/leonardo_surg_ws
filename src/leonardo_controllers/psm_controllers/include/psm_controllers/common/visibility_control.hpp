// Copyright 2026
// Licensed under Apache 2.0

#ifndef PSM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_
#define PSM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_

// ==========================================
// Windows (MSVC & MinGW)
// ==========================================
#if defined(_WIN32) || defined(__CYGWIN__)

#ifdef PSM_CONTROLLERS_BUILDING_DLL
#ifdef __GNUC__
#define PSM_CONTROLLERS_PUBLIC __attribute__((dllexport))
#else
#define PSM_CONTROLLERS_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define PSM_CONTROLLERS_PUBLIC __attribute__((dllimport))
#else
#define PSM_CONTROLLERS_PUBLIC __declspec(dllimport)
#endif
#endif

#define PSM_CONTROLLERS_LOCAL

// ==========================================
// Linux / macOS (GCC / Clang)
// ==========================================
#else

#if defined(__has_attribute)
#if __has_attribute(visibility)
#define PSM_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define PSM_CONTROLLERS_LOCAL  __attribute__((visibility("hidden")))
#else
#define PSM_CONTROLLERS_PUBLIC
#define PSM_CONTROLLERS_LOCAL
#endif
#elif defined(__GNUC__) && __GNUC__ >= 4
#define PSM_CONTROLLERS_PUBLIC __attribute__((visibility("default")))
#define PSM_CONTROLLERS_LOCAL  __attribute__((visibility("hidden")))
#else
#define PSM_CONTROLLERS_PUBLIC
#define PSM_CONTROLLERS_LOCAL
#endif

#endif

#endif  // PSM_CONTROLLERS__COMMON__VISIBILITY_CONTROL_HPP_
