#pragma once

// NOTE(swenninger):
//   The FBX SDK provided by autodesk throws a lot of warnings in fbxsdk.h.
//   Since we don't want to change the FBX library, these warnings are
//   disabled when including fbxsdk.h


#if defined(__GNUC__) && (__GNUC__ >= 8)
  #define CLASS_MEMACCESS_WARNING _Pragma("GCC diagnostic ignored \"-Wclass-memaccess\"")
#else
  #define CLASS_MEMACCESS_WARNING
#endif

#if defined(_MSC_VER)
  #define FBX_DISABLE_WARNINGS_BEGIN //TODO
  #define FBX_DISABLE_WARNINGS_END //TODO
#elif defined(__GNUC__) || defined(__clang__)
  #define FBX_DISABLE_WARNINGS_BEGIN \
    _Pragma("GCC diagnostic push") \
    _Pragma("GCC diagnostic ignored \"-Wunused-parameter\"") \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-copy\"") \
    _Pragma("GCC diagnostic ignored \"-Wpedantic\"") \
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"") \
    _Pragma("GCC diagnostic ignored \"-Wshadow\"") \
    CLASS_MEMACCESS_WARNING
  #define FBX_DISABLE_WARNINGS_END \
    _Pragma("GCC diagnostic pop")
#else
  #define FBX_DISABLE_WARNINGS_BEGIN
  #define FBX_DISABLE_WARNINGS_END
#endif

#if 0
// TODO(swenninger): find actual warning codes for MSVC (and apple?)
#define FBX_DISABLE_WARNINGS_BEGIN \
  __pragma(warning(push, 0)) \
  __pragma(warning(disable: 4701)) \
  __pragma(warning(disable: 4702)) \
  __pragma(warning(default: 4701)) \
  __pragma(warning(default: 4702)) \
#define FBX_DISABLE_WARNINGS_END \
  __pragma(warning(pop ))
#endif
