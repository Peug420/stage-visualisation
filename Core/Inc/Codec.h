/*
 * my_Codec.h
 *
 *  Created on: 17.06.2022
 *      Author: Davidlohner
 */

#include <stdint.h>

#ifndef INC_MY_CODEC_H_
#define INC_MY_CODEC_H_


#define CODECI2CADDRESS ((uint16_t)0x34)

void write_register(uint16_t register_pointer, uint16_t register_value);
void Codec(void);

#endif /* INC_MY_CODEC_H_ */
