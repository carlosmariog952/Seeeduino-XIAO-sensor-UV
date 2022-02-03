/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/


//#include "GUVA-C32SM.h"


#ifndef LIB_GUVA_C32SM_H
#define LIB_GUVA_C32SM_H
#ifdef _cplusplus
extern "C"  {
#endif

#include "stm32f4xx_hal.h"


//External Function
HAL_StatusTypeDef Start_GUVA_C32 (I2C_HandleTypeDef * hi2c);
HAL_StatusTypeDef Mode_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t MODE, uint8_t PMODE);
HAL_StatusTypeDef Resolution_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t RES);
HAL_StatusTypeDef Range_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t RANGE);
HAL_StatusTypeDef Sleep_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t TIME);
HAL_StatusTypeDef Reset_GUVA_C32 (I2C_HandleTypeDef * hi2c);
HAL_StatusTypeDef Receive_GUVA_C32 (I2C_HandleTypeDef * hi2c, float *UVA);

//Internal Function
float Converter (uint16_t UVA_RES);

#ifdef _cplusplus
}
#endif
#endif// LIB_GUVA_C32SM_H

static const uint8_t REG_GUVA = 0x62 << 1;
static const uint8_t REG_ID_GUVA = 0x00;
static const uint8_t MODE_GUVA = 0x01;
static const uint8_t RES_GUVA = 0x04;
static const uint8_t RANGE_GUVA = 0x05;
static const uint8_t MODE_CTL_GUVA = 0x0A;
static const uint8_t SOFT_RESET_GUVA = 0x0B;
static const uint8_t UVA_LSB_GUVA = 0x15;
static const uint8_t UVA_MSB_GUVA = 0x16;


// Sensor modes
#define No_op 0b00000000;
#define UVA_op 0b00010000;
#define Not_use 0b00100000;
#define Normal_mode 0b00000000;
#define Low_powermode 0b00000001;
#define Auto_shutdow_mode 0b0000010;
#define Shutdown_mode 0b00000011;

// Sensor Resolutions
#define Res_800 0b00000000
#define Res_400 0b00000001
#define Res_200 0b00000010
#define Res_100 0b00000011


//Sensor Range
#define Range_x1 0b00000000
#define Range_x2 0b00000001
#define Range_x4 0b00000010
#define Range_x8 0b00000011
#define Range_x16 0b00000100
#define Range_x32 0b00000101
#define Range_x64 0b00000110
#define Range_x128 0b00000111

//sleep duration
#define Time_2 0b00000000
#define Time_4 0b00000001
#define Time_8 0b00000010
#define Time_16 0b00000011
#define Time_32 0b00000100
#define Time_64 0b00000101
#define Time_128 0b00000110
#define Time_256 0b00000111

//reset
#define RESET 0xA5


//===========================================================================
//              Sensor Initialization
//===========================================================================

HAL_StatusTypeDef Start_GUVA_C32 (I2C_HandleTypeDef * hi2c){
  uint8_t buf[1];
  HAL_StatusTypeDef ret;

  buf[0] = REG_ID_GUVA;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 1, 10);
  if(ret != HAL_OK){
    return ret;
  }else{
    ret = HAL_I2C_Master_Receive(hi2c, REG_GUVA, buf, 1, 10);
    if(ret != HAL_OK || buf[0] != REG_ID_GUVA){
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

//===========================================================================
//                Sensor Mode
//===========================================================================

HAL_StatusTypeDef Mode_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t MODE, uint8_t PMODE){
  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf [0] = MODE_GUVA;
  buf [1] = MODE + PMODE;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 2, 10);
  if (ret != HAL_OK){
    return ret;
  }
  return HAL_OK;
}

//===========================================================================
//                Sensor Resolution
//===========================================================================

HAL_StatusTypeDef Resolution_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t RES){
  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf [0] = RES_GUVA;
  buf [1] = RES;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 2, 10);
  if (ret != HAL_OK){
    return ret;
  }
  return HAL_OK;
}


//===========================================================================
//                Sensor Range
//===========================================================================

HAL_StatusTypeDef Range_GUVA_C32 (I2C_HandleTypeDef * hi2c, uint8_t RANGE){

  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf [0] = RANGE_GUVA;
  buf [1] = RANGE;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 2, 10);
  if (ret != HAL_OK){
    return ret;
  }
  return HAL_OK;
}


//===========================================================================
//            Sensor Sleep Duration
//===========================================================================

HAL_StatusTypeDef Sleep_GUVA_C32(I2C_HandleTypeDef * hi2c, uint8_t TIME){

  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf [0] = MODE_CTL_GUVA;
  buf [1] = TIME;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 2, 10);
  if (ret != HAL_OK){
    return ret;
  }
  return HAL_OK;
}

//===========================================================================
//            Sensor Reset
//===========================================================================

HAL_StatusTypeDef Reset_GUVA_C32 (I2C_HandleTypeDef * hi2c){

  uint8_t buf[2];
  HAL_StatusTypeDef ret;

  buf [0] = SOFT_RESET_GUVA;
  buf [1] = RESET;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 2, 10);
  if (ret != HAL_OK){
    return ret;
  }
  return HAL_OK;
}


//===========================================================================
//              Sensor Receive
//===========================================================================

HAL_StatusTypeDef Receive_GUVA_C32 (I2C_HandleTypeDef * hi2c, float* UVA){

  uint8_t buf[1];
  uint8_t save_buf;
  uint16_t UVA_RES;
  HAL_StatusTypeDef ret;

  buf[0] = UVA_LSB_GUVA;

  ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 1, 10);
  if(ret != HAL_OK){
    return ret;
  }
    else{
      ret = HAL_I2C_Master_Receive(hi2c, REG_GUVA, buf, 1, 10);
      if(ret != HAL_OK){
        return ret;
      }
      else{
        save_buf = buf[0];
        buf[0] = UVA_MSB_GUVA;

        ret = HAL_I2C_Master_Transmit(hi2c, REG_GUVA, buf, 1, 10);
        if(ret != HAL_OK){
          return ret;
        }
        else{
          ret = HAL_I2C_Master_Receive(hi2c, REG_GUVA, buf, 1, 10);
          if(ret != HAL_OK){
            return ret;
          }
          else{
            UVA_RES = buf[0] + save_buf;    // Da uma olhada pra ver se isso aqui funciona
            *UVA = Converter (UVA_RES);
          }
        }
      }
    }
  return HAL_OK;
}

//===========================================================================
//            Converter
//===========================================================================

float Converter (uint16_t UVA_RES){

  float res;
  res = ((UVA_RES*100)/255);
  return res;

}




// the setup function runs once when you press reset or power the board

int buttonState = 200;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(100);                       // wait for a second
 Serial.println(buttonState);
  int green = 10;
  Serial.print(green, HEX);
  Serial.print(green, HEX);
  Serial.print(green, HEX);
}
