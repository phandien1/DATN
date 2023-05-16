/*
 * Flag.h
 *
 *  Created on: 14 thg 5, 2023
 *      Author: Administrator
 */

#ifndef INC_FLAG_H_
#define INC_FLAG_H_
#include "main.h"

#define CHECKFLAG(FlagGroup,FlagBit) ((((FlagGroup) & (FlagBit)) == (FlagBit)) ? 1 : 0)
#define SETFLAG(FlagGroup,FlagBit) ((FlagGroup) |= (FlagBit))
#define CLEARFLAG(FlagGroup,FlagBit) (FlagGroup &= ~(FlagBit))


typedef uint8_t FlagGroup_t;

#endif /* INC_FLAG_H_ */
