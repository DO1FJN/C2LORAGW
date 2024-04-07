/*
 * compiler.h
 *
 * Compiler and architecture / definitions / general typedefs.
 *
 *  Created on: 03.02.2012
 *      Author: Jan Alte, DO1FJN
 *
 * This file is part of the DV-RPTR application.
 * For general information about this program see "main.c".
 *
 * DV-RPTR app is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * The DV-RPTR app is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this package. If not, see <http://www.gnu.org/licenses/>.
 *
 */


#ifndef COMPILER_H_
#define COMPILER_H_

#ifndef __windows__		// define a simple "windows build" flag
#if __WIN32__ || _MSC_VER
#define __windows__
#endif
#endif

#ifndef TARGENT_NAME		// Generic Target Name Strings (should be defined in makefile)
#ifdef __windows__
#define TARGET_NAME		"MS Windows"
#endif

#ifdef __unix__
#define TARGET_NAME		"Linux"
#endif
#endif

#include <stdbool.h>

typedef signed char             S8 ;  //!< 8-bit signed integer.
typedef unsigned char           U8 ;  //!< 8-bit unsigned integer.
typedef signed short int        S16;  //!< 16-bit signed integer.
typedef unsigned short int      U16;  //!< 16-bit unsigned integer.
typedef signed long int    	    S32;  //!< 32-bit signed integer.
typedef unsigned long int      	U32;  //!< 32-bit unsigned integer.
typedef signed long long int   	S64;  //!< 64-bit signed integer.
typedef unsigned long long int  U64;  //!< 64-bit unsigned integer.
typedef float                   F32;  //!< 32-bit floating-point number.
typedef double                  F64;  //!< 64-bit floating-point number.

/*! \name Aliasing Aggregate Types
 */
//! @{

//! 16-bit union.
typedef union {
  S16 s16   ;
  U16 u16   ;
  S8  s8 [2];
  U8  u8 [2];
} Union16;

//! 32-bit union.
typedef union {
  S32 s32   ;
  U32 u32   ;
  S16 s16[2];
  U16 u16[2];
  S8  s8 [4];
  U8  u8 [4];
} Union32;

typedef union {
  S64 s64;
  U64 u64;
  S32 s32[2];
  U32 u32[2];
  S16 s16[4];
  U16 u16[4];
  S8  s8 [8];
  U8  u8 [8];
} Union64;


//! @}


/*! \name Compiler spezific attributes and types
 */
//! @{

#if __GNUC__

#define PACKED_DATA     __attribute__((__packed__))
#define ALIGNED_DATA	__attribute__ ((aligned(4)))
#define __weak		__attribute__((weak))

#elif __ICCAVR32__

#define PACKED_DATA
#define ALIGNED_DATA	_Pragma("data_alignment=4")

#endif


//! @}


typedef void (*tfunction)(void);	// void-void Functions generell

#if (LITTLE_ENDIAN_MCU==TRUE)
# include "littleendian.h"
#elif (BIG_ENDIAN_MCU==TRUE)
# include "bigendian.h"
#else
# error Unknown endianism.
#endif



#endif // COMPILER_H_
