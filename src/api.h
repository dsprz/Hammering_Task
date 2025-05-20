#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define Hammering_FSM_Controller_DLLIMPORT __declspec(dllimport)
#  define Hammering_FSM_Controller_DLLEXPORT __declspec(dllexport)
#  define Hammering_FSM_Controller_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define Hammering_FSM_Controller_DLLIMPORT __attribute__((visibility("default")))
#    define Hammering_FSM_Controller_DLLEXPORT __attribute__((visibility("default")))
#    define Hammering_FSM_Controller_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define Hammering_FSM_Controller_DLLIMPORT
#    define Hammering_FSM_Controller_DLLEXPORT
#    define Hammering_FSM_Controller_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef Hammering_FSM_Controller_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define Hammering_FSM_Controller_DLLAPI
#  define Hammering_FSM_Controller_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef Hammering_FSM_Controller_EXPORTS
#    define Hammering_FSM_Controller_DLLAPI Hammering_FSM_Controller_DLLEXPORT
#  else
#    define Hammering_FSM_Controller_DLLAPI Hammering_FSM_Controller_DLLIMPORT
#  endif // Hammering_FSM_Controller_EXPORTS
#  define Hammering_FSM_Controller_LOCAL Hammering_FSM_Controller_DLLLOCAL
#endif // Hammering_FSM_Controller_STATIC