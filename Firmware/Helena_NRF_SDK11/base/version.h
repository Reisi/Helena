/**
  ******************************************************************************
  * @file    version.h
  * @author  Thomas Reisnecker
  * @brief   Helen version number
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef VERSION_H_INCLUDED
#define VERSION_H_INCLUDED

#define VERSION_MAJOR   3
#define VERSION_MINOR   0
#define VERSION_PATCH   1

#define VERSION_LEVEL   "alpha"

#ifndef VERSION_MAJOR
#error "at least VERSION_MAJOR is necessary"
#endif // VERSION_MAJOR

#define str(s)  #s
#define xstr(s) str(s)

#ifndef VERSION_MINOR
#define VERSION_NUMSTR  xstr(VERSION_MAJOR)
#else
#ifndef VERSION_PATCH
#define VERSION_NUMSTR  xstr(VERSION_MAJOR) "." xstr(VERSION_MINOR)
#else
#define VERSION_NUMSTR  xstr(VERSION_MAJOR) "." xstr(VERSION_MINOR) "." xstr(VERSION_PATCH)
#endif // VERSION_PATCH
#endif // VERSION_MINOR

#ifndef VERSION_LEVEL
#define VERSION_STRING  VERSION_NUMSTR
#else
#define VERSION_STRING  VERSION_NUMSTR "-" VERSION_LEVEL
#endif // VERSION_LEVEL

#endif /* VERSION_H_INCLUDED */

/**END OF FILE*****************************************************************/
