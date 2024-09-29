/*
 * L3G4200D.h
 *
 *  Created on: Aug 1, 2024
 *      Author: Minh Tuan
 */
#include "stm32f1xx_hal.h"


#ifndef INC_L3G4200D_H_
#define INC_L3G4200D_H_

#define L3G4200D_ADDRESS           (0xD2 >> 1)

#define L3G4200D_WHO_AM_I_ADDR              0x0F
#define L3G4200D_CTRL_REG1_ADDR             0x20
#define L3G4200D_CTRL_REG2_ADDR             0x21
#define L3G4200D_CTRL_REG3_ADDR             0x22
#define L3G4200D_CTRL_REG4_ADDR             0x23
#define L3G4200D_CTRL_REG5_ADDR             0x24
#define L3G4200D_REFERENCE_ADDR             0x25
#define L3G4200D_OUT_TEMP_ADDR              0x26
#define L3G4200D_STATUS_REG_ADDR            0x27
#define L3G4200D_OUT_X_L_ADDR               0x28
#define L3G4200D_OUT_X_H_ADDR               0x29
#define L3G4200D_OUT_Y_L_ADDR               0x2A
#define L3G4200D_OUT_Y_H_ADDR               0x2B
#define L3G4200D_OUT_Z_L_ADDR               0x2C
#define L3G4200D_OUT_Z_H_ADDR               0x2D
#define L3G4200D_FIFO_CTRL_REG_ADDR         0x2E
#define L3G4200D_FIFO_SRC_REG_ADDR          0x2F
#define L3G4200D_INT1_CFG_ADDR              0x30
#define L3G4200D_INT1_SRC_ADDR              0x31
#define L3G4200D_INT1_TSH_XH_ADDR           0x32
#define L3G4200D_INT1_TSH_XL_ADDR           0x33
#define L3G4200D_INT1_TSH_YH_ADDR           0x34
#define L3G4200D_INT1_TSH_YL_ADDR           0x35
#define L3G4200D_INT1_TSH_ZH_ADDR           0x36
#define L3G4200D_INT1_TSH_ZL_ADDR           0x37
#define L3G4200D_DURATION                   0x38


#endif /* INC_L3G4200D_H_ */
