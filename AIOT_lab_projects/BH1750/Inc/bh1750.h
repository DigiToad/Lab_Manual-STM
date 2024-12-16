
#ifndef INC_BH1750_H_
#define INC_BH1750_H_

#include "stdio.h"

// BH1750 I2C Address
#define BH1750_ADDR 0x23 // BH1750 I2C address


// Function prototypes
void BH1750_Init(I2C_HandleTypeDef *hi2c);
float BH1750_ReadLux(I2C_HandleTypeDef *hi2c);

// Error Status
HAL_StatusTypeDef Transmit_Err, Receive_Err;


// BH1750 initialization
void BH1750_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t cmd[] = {0x10}; // Power on
    Transmit_Err = HAL_I2C_Master_Transmit(hi2c, BH1750_ADDR << 1, cmd, sizeof(cmd), HAL_MAX_DELAY);
//    if(Transmit_Err != HAL_ERROR){
//    	  printf("\r\n");
//    	  printf("BH1750 has been initialized");
//    	  printf("\r\n");
//    }
}

// Reading Light Intensity from BH1750 sensor
float BH1750_ReadLux(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2];
    HAL_I2C_Master_Receive(hi2c, BH1750_ADDR << 1, data, sizeof(data), HAL_MAX_DELAY);

    uint16_t lux = (data[0] << 8) | data[1];
    return (float)lux / 1.2;
}
#endif
