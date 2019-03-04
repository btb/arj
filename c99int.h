/*
 * $Id: c99int.h,v 1.1 2007/04/19 18:42:09 andrew_belov Exp $
 * ---------------------------------------------------------------------------
 * Definitions of C99 integer types with some surrogates for legacy compilers.
 *
 */

#ifndef C99INT_INCLUDED
#define C99INT_INCLUDED

#ifdef HAVE_STDINT_H

#include <stdint.h>

#else

typedef signed char     int8_t;
typedef unsigned char   uint8_t;
typedef signed short    int16_t;
typedef unsigned short  uint16_t;
#ifdef TILED
typedef signed long     int32_t;
typedef unsigned long   uint32_t;
#else
/* TODO: need special cases for systems with 64-bit integers and no stdint.h */
typedef signed int      int32_t;
typedef unsigned int    uint32_t;
#endif
/* Do not bother defining [u]int64_t for now */

#endif  /* !stdint.h */
#endif
