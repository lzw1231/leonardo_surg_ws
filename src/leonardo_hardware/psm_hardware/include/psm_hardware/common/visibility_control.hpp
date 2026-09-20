// Copyright 2026
// Licensed under Apache 2.0
#ifndef PSM_HARDWARE__VISIBILITY_CONTROL_H_
#define PSM_HARDWARE__VISIBILITY_CONTROL_H_
#ifdef __cplusplus
extern "C"{
#endif
// ==========================================
// Windows (MSVC & MinGW)
// ==========================================
#if defined _WIN32 || defined __CYGWIN__
#ifdef PSM_HARDWARE_BUILDING_DLL
#ifdef __GNUC__
#define PSM_HARDWARE_PUBLIC __attribute__((dllexport))
#else
#define PSM_HARDWARE_PUBLIC __declspec(dllexport)
#endif
#else
#ifdef __GNUC__
#define PSM_HARDWARE_PUBLIC __attribute__((dllimport))
#else
#define PSM_HARDWARE_PUBLIC __declspec(dllimport)
#endif
#endif
#define PSM_HARDWARE_LOCAL
#else
// ==========================================
// Linux / macOS (GCC / Clang)
// ==========================================
#if defined(__has_attribute) && __has_attribute(visibility)
#define PSM_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define PSM_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
#elif defined(__GNUC__) && __GNUC__ >= 4
#define PSM_HARDWARE_PUBLIC __attribute__((visibility("default")))
#define PSM_HARDWARE_LOCAL  __attribute__((visibility("hidden")))
#else
#define PSM_HARDWARE_PUBLIC
#define PSM_HARDWARE_LOCAL
#endif
#endif
#ifdef __cplusplus
}
#endif
#endif  // PSM_HARDWARE__VISIBILITY_CONTROL_H_
