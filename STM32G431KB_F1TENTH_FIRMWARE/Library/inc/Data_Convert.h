/*
 * Data_Convert.h
 *
 *  Created on: Apr 24, 2024
 *      Author: 08809
 */

#ifndef INC_DATA_CONVERT_H_
#define INC_DATA_CONVERT_H_

typedef union {
	uint16_t u16;
	int16_t i16;
	uint8_t u8[2];
	int8_t i8[2];
}data16_t;

typedef union {
	uint32_t u32;
	int32_t i32;
	uint16_t u16[2];
	int16_t i16[2];
	uint8_t u8[4];
	int8_t i8[4];
	float f32;
}data32_t;

typedef union {
	uint64_t u64;
	int64_t i64;
	uint32_t u32[2];
	int32_t i32[2];
	uint16_t u16[4];
	int16_t i16[4];
	uint8_t u8[8];
	int8_t i8[8];
	double f64;
}data64_t;

#endif /* INC_DATA_CONVERT_H_ */
