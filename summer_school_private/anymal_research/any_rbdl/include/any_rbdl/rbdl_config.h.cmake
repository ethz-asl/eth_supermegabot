/*
 * RBDL - Rigid Body Dynamics Library
 * Copyright (c) 2011-2012 Martin Felis <martin.felis@iwr.uni-heidelberg.de>
 *
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef RBDL_CONFIG_H
#define RBDL_CONFIG_H

#define ANY_RBDL_API_VERSION (@ANY_RBDL_VERSION_MAJOR@ << 16) + (@ANY_RBDL_VERSION_MINOR@ << 8) + @ANY_RBDL_VERSION_PATCH@

#cmakedefine ANY_RBDL_USE_SIMPLE_MATH
#cmakedefine ANY_RBDL_ENABLE_LOGGING
#cmakedefine ANY_RBDL_BUILD_REVISION "@RBDL_BUILD_REVISION@"
#cmakedefine ANY_RBDL_BUILD_TYPE "@RBDL_BUILD_TYPE@"
#cmakedefine ANY_RBDL_BUILD_BRANCH "@RBDL_BUILD_BRANCH@"
#cmakedefine ANY_RBDL_BUILD_ADDON_LUAMODEL
#cmakedefine ANY_RBDL_BUILD_ADDON_URDFREADER
#cmakedefine ANY_RBDL_BUILD_STATIC

/* compatibility defines */
#ifdef _WIN32
	#define __func__ __FUNCTION__
	#define M_PI 3.1415926535897932384
#endif

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define ANY_RBDL_DLLIMPORT __declspec(dllimport)
#  define ANY_RBDL_DLLEXPORT __declspec(dllexport)
#  define ANY_RBDL_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define ANY_RBDL_DLLIMPORT __attribute__ ((visibility("default")))
#   define ANY_RBDL_DLLEXPORT __attribute__ ((visibility("default")))
#   define ANY_RBDL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define ANY_RBDL_DLLIMPORT
#   define ANY_RBDL_DLLEXPORT
#   define ANY_RBDL_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef ANY_RBDL_BUILD_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ANY_RBDL_DLLAPI
#  define ANY_RBDL_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef any_rbdl_EXPORTS
#   define ANY_RBDL_DLLAPI ANY_RBDL_DLLEXPORT
#  else
#   define ANY_RBDL_DLLAPI ANY_RBDL_DLLIMPORT
#  endif // ANY_RBDL_EXPORTS
#  define ANY_RBDL_LOCAL ANY_RBDL_DLLLOCAL
# endif // ANY_RBDL_BUILD_STATIC

#endif
