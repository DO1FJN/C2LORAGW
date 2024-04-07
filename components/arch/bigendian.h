/*
 * bigendian.h
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

#ifndef BIGENDIAN_H_
#define BIGENDIAN_H_

#if (BIG_ENDIAN_MCU==TRUE)

  #define MSB(u16)        (((U8  *)&(u16))[0])  //!< Most significant byte of \a u16.
  #define LSB(u16)        (((U8  *)&(u16))[1])  //!< Least significant byte of \a u16.

  #define MSH(u32)        (((U16 *)&(u32))[0])  //!< Most significant half-word of \a u32.
  #define LSH(u32)        (((U16 *)&(u32))[1])  //!< Least significant half-word of \a u32.
  #define MSB0W(u32)      (((U8  *)&(u32))[0])  //!< Most significant byte of 1st rank of \a u32.
  #define MSB1W(u32)      (((U8  *)&(u32))[1])  //!< Most significant byte of 2nd rank of \a u32.
  #define MSB2W(u32)      (((U8  *)&(u32))[2])  //!< Most significant byte of 3rd rank of \a u32.
  #define MSB3W(u32)      (((U8  *)&(u32))[3])  //!< Most significant byte of 4th rank of \a u32.
  #define LSB3W(u32)      MSB0W(u32)            //!< Least significant byte of 4th rank of \a u32.
  #define LSB2W(u32)      MSB1W(u32)            //!< Least significant byte of 3rd rank of \a u32.
  #define LSB1W(u32)      MSB2W(u32)            //!< Least significant byte of 2nd rank of \a u32.
  #define LSB0W(u32)      MSB3W(u32)            //!< Least significant byte of 1st rank of \a u32.

  #define MSW(u64)        (((U32 *)&(u64))[0])  //!< Most significant word of \a u64.
  #define LSW(u64)        (((U32 *)&(u64))[1])  //!< Least significant word of \a u64.
  #define MSH0(u64)       (((U16 *)&(u64))[0])  //!< Most significant half-word of 1st rank of \a u64.
  #define MSH1(u64)       (((U16 *)&(u64))[1])  //!< Most significant half-word of 2nd rank of \a u64.
  #define MSH2(u64)       (((U16 *)&(u64))[2])  //!< Most significant half-word of 3rd rank of \a u64.
  #define MSH3(u64)       (((U16 *)&(u64))[3])  //!< Most significant half-word of 4th rank of \a u64.
  #define LSH3(u64)       MSH0(u64)             //!< Least significant half-word of 4th rank of \a u64.
  #define LSH2(u64)       MSH1(u64)             //!< Least significant half-word of 3rd rank of \a u64.
  #define LSH1(u64)       MSH2(u64)             //!< Least significant half-word of 2nd rank of \a u64.
  #define LSH0(u64)       MSH3(u64)             //!< Least significant half-word of 1st rank of \a u64.
  #define MSB0D(u64)      (((U8  *)&(u64))[0])  //!< Most significant byte of 1st rank of \a u64.
  #define MSB1D(u64)      (((U8  *)&(u64))[1])  //!< Most significant byte of 2nd rank of \a u64.
  #define MSB2D(u64)      (((U8  *)&(u64))[2])  //!< Most significant byte of 3rd rank of \a u64.
  #define MSB3D(u64)      (((U8  *)&(u64))[3])  //!< Most significant byte of 4th rank of \a u64.
  #define MSB4D(u64)      (((U8  *)&(u64))[4])  //!< Most significant byte of 5th rank of \a u64.
  #define MSB5D(u64)      (((U8  *)&(u64))[5])  //!< Most significant byte of 6th rank of \a u64.
  #define MSB6D(u64)      (((U8  *)&(u64))[6])  //!< Most significant byte of 7th rank of \a u64.
  #define MSB7D(u64)      (((U8  *)&(u64))[7])  //!< Most significant byte of 8th rank of \a u64.
  #define LSB7D(u64)      MSB0D(u64)            //!< Least significant byte of 8th rank of \a u64.
  #define LSB6D(u64)      MSB1D(u64)            //!< Least significant byte of 7th rank of \a u64.
  #define LSB5D(u64)      MSB2D(u64)            //!< Least significant byte of 6th rank of \a u64.
  #define LSB4D(u64)      MSB3D(u64)            //!< Least significant byte of 5th rank of \a u64.
  #define LSB3D(u64)      MSB4D(u64)            //!< Least significant byte of 4th rank of \a u64.
  #define LSB2D(u64)      MSB5D(u64)            //!< Least significant byte of 3rd rank of \a u64.
  #define LSB1D(u64)      MSB6D(u64)            //!< Least significant byte of 2nd rank of \a u64.
  #define LSB0D(u64)      MSB7D(u64)            //!< Least significant byte of 1st rank of \a u64.

#endif


#endif // BIGENDIAN_H_
