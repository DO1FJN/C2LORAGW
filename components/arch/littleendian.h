/*
 * littleendian.h
 *
 * description.
 *
 *  Created on: 04.02.2012
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

#ifndef LITTLEENDIAN_H_
#define LITTLEENDIAN_H_

#if (LITTLE_ENDIAN_MCU==TRUE)

  #define LSB(u16)        (((U8  *)&(u16))[0])  //!< Least significant byte of \a u16.
  #define MSB(u16)        (((U8  *)&(u16))[1])  //!< Most significant byte of \a u16.

  #define LSH(u32)        (((U16 *)&(u32))[0])  //!< Least significant half-word of \a u32.
  #define MSH(u32)        (((U16 *)&(u32))[1])  //!< Most significant half-word of \a u32.
  #define LSB0W(u32)      (((U8  *)&(u32))[0])  //!< Least significant byte of 1st rank of \a u32.
  #define LSB1W(u32)      (((U8  *)&(u32))[1])  //!< Least significant byte of 2nd rank of \a u32.
  #define LSB2W(u32)      (((U8  *)&(u32))[2])  //!< Least significant byte of 3rd rank of \a u32.
  #define LSB3W(u32)      (((U8  *)&(u32))[3])  //!< Least significant byte of 4th rank of \a u32.
  #define MSB3W(u32)      LSB0W(u32)            //!< Most significant byte of 4th rank of \a u32.
  #define MSB2W(u32)      LSB1W(u32)            //!< Most significant byte of 3rd rank of \a u32.
  #define MSB1W(u32)      LSB2W(u32)            //!< Most significant byte of 2nd rank of \a u32.
  #define MSB0W(u32)      LSB3W(u32)            //!< Most significant byte of 1st rank of \a u32.

  #define LSW(u64)        (((U32 *)&(u64))[0])  //!< Least significant word of \a u64.
  #define MSW(u64)        (((U32 *)&(u64))[1])  //!< Most significant word of \a u64.
  #define LSH0(u64)       (((U16 *)&(u64))[0])  //!< Least significant half-word of 1st rank of \a u64.
  #define LSH1(u64)       (((U16 *)&(u64))[1])  //!< Least significant half-word of 2nd rank of \a u64.
  #define LSH2(u64)       (((U16 *)&(u64))[2])  //!< Least significant half-word of 3rd rank of \a u64.
  #define LSH3(u64)       (((U16 *)&(u64))[3])  //!< Least significant half-word of 4th rank of \a u64.
  #define MSH3(u64)       LSH0(u64)             //!< Most significant half-word of 4th rank of \a u64.
  #define MSH2(u64)       LSH1(u64)             //!< Most significant half-word of 3rd rank of \a u64.
  #define MSH1(u64)       LSH2(u64)             //!< Most significant half-word of 2nd rank of \a u64.
  #define MSH0(u64)       LSH3(u64)             //!< Most significant half-word of 1st rank of \a u64.
  #define LSB0D(u64)      (((U8  *)&(u64))[0])  //!< Least significant byte of 1st rank of \a u64.
  #define LSB1D(u64)      (((U8  *)&(u64))[1])  //!< Least significant byte of 2nd rank of \a u64.
  #define LSB2D(u64)      (((U8  *)&(u64))[2])  //!< Least significant byte of 3rd rank of \a u64.
  #define LSB3D(u64)      (((U8  *)&(u64))[3])  //!< Least significant byte of 4th rank of \a u64.
  #define LSB4D(u64)      (((U8  *)&(u64))[4])  //!< Least significant byte of 5th rank of \a u64.
  #define LSB5D(u64)      (((U8  *)&(u64))[5])  //!< Least significant byte of 6th rank of \a u64.
  #define LSB6D(u64)      (((U8  *)&(u64))[6])  //!< Least significant byte of 7th rank of \a u64.
  #define LSB7D(u64)      (((U8  *)&(u64))[7])  //!< Least significant byte of 8th rank of \a u64.
  #define MSB7D(u64)      LSB0D(u64)            //!< Most significant byte of 8th rank of \a u64.
  #define MSB6D(u64)      LSB1D(u64)            //!< Most significant byte of 7th rank of \a u64.
  #define MSB5D(u64)      LSB2D(u64)            //!< Most significant byte of 6th rank of \a u64.
  #define MSB4D(u64)      LSB3D(u64)            //!< Most significant byte of 5th rank of \a u64.
  #define MSB3D(u64)      LSB4D(u64)            //!< Most significant byte of 4th rank of \a u64.
  #define MSB2D(u64)      LSB5D(u64)            //!< Most significant byte of 3rd rank of \a u64.
  #define MSB1D(u64)      LSB6D(u64)            //!< Most significant byte of 2nd rank of \a u64.
  #define MSB0D(u64)      LSB7D(u64)            //!< Most significant byte of 1st rank of \a u64.

#endif

#endif // LITTLEENDIAN_H_
