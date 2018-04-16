//
//           .___.
//           {o,o}
//          ./)_)
//      owl --"-"---
//
//  Copyright (c) 2018 Sören König. All rights reserved.
//

#pragma once

#ifdef _WIN32
#pragma warning (disable:4251)
#ifdef owl_EXPORTS
#define OWL_API __declspec(dllexport)
#else
#ifdef OWL_DLL
#define OWL_API __declspec(dllimport)
#else
#define OWL_API
#endif
#endif

#else

#ifdef owl_EXPORTS
#define OWL_API __attribute__ ((visibility ("default")))
#else
#define OWL_API __attribute__ ((visibility ("default")))
#endif

#endif


/*
 // Generic helper definitions for shared library support
#if defined _WIN32 || defined __CYGWIN__
  #define FOX_HELPER_DLL_IMPORT __declspec(dllimport)
  #define FOX_HELPER_DLL_EXPORT __declspec(dllexport)
  #define FOX_HELPER_DLL_LOCAL
#else
  #if __GNUC__ >= 4
    #define FOX_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
    #define FOX_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
    #define FOX_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
  #else
    #define FOX_HELPER_DLL_IMPORT
    #define FOX_HELPER_DLL_EXPORT
    #define FOX_HELPER_DLL_LOCAL
  #endif
#endif

// Now we use the generic helper definitions above to define FOX_API and FOX_LOCAL.
// FOX_API is used for the public API symbols. It either DLL imports or DLL exports (or does nothing for static build)
// FOX_LOCAL is used for non-api symbols.

#ifdef FOX_DLL // defined if FOX is compiled as a DLL
  #ifdef owl_EXPORTS // defined if we are building the FOX DLL (instead of using it)
    #define FOX_API FOX_HELPER_DLL_EXPORT
  #else
    #define FOX_API FOX_HELPER_DLL_IMPORT
  #endif // owl_EXPORTS
  #define FOX_LOCAL FOX_HELPER_DLL_LOCAL
#else // FOX_DLL is not defined: this means FOX is a static lib.
  #define FOX_API
  #define FOX_LOCAL
#endif // FOX_DLL
 */



