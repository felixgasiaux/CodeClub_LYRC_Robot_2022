/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeCamera
 * \brief   Driver for MeCamera module.
 * @file    MeCamera.cpp
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2017/09/26
 * @brief   Driver for MeCamera module.
 *
 * \par Copyright
 * This software is Copyright (C), 2012-2016, MakeBlock. Use is subject to license \n
 * conditions. The main licensing options available are GPL V2 or Commercial: \n
 *
 * \par Open Source Licensing GPL V2
 * This is the appropriate option if you want to share the source code of your \n
 * application with everyone you distribute it to, and you also want to give them \n
 * the right to share who uses it. If you wish to use this software under Open \n
 * Source Licensing, you must contribute all your source code to the open source \n
 * community in accordance with the GPL Version 2 when your application is \n
 * distributed. See http://www.gnu.org/copyleft/gpl.html
 *
 * \par Description
 * This file is a drive for MeCamera module, It supports MeCamera V1.0 device provided
 * by MakeBlock.
 *
 * \par Method List:
 *
 *    1.void MeCamera::setpin(uint8_t AD0, uint8_t INT)
 *    2.void MeCamera::begin(void)
 *
 * \par History:
 * <pre>
 * "<Author>"         "<Time>"        "<Version>"        "<Descr>"
 *  Payton            2017/09/26        1.0.0         rebuild the old lib.
 * </pre>
 *
 * @example CameraTest.ino
 */

/* Includes ------------------------------------------------------------------*/
#include "MeCamera.h"

#include <Arduino.h>

/* Private functions ---------------------------------------------------------*/
#ifdef ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the MeCamera to arduino port
 * no pins are used or initialized here, but PWM frequency set to 976 Hz
 * \param[in]
 *   port - RJ25 port from PORT_1 to PORT_4
 */
MeCamera::MeCamera(uint8_t port) : MePort(port)
{
  //address0-11, address1-10, address2-01, address3-00
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  if(port == PORT_1)
  {
    digitalWrite(s1,LOW);
    digitalWrite(s2,LOW);
    Device_Address = CAMERA_DEFAULT_ADDRESS + 0;
  }
  else if(port == PORT_2)
  {
    digitalWrite(s1,HIGH);
    digitalWrite(s2,LOW);
    Device_Address = CAMERA_DEFAULT_ADDRESS + 1;
  }
  else if(port == PORT_3)
  {
    digitalWrite(s1,LOW);
    digitalWrite(s2,HIGH);
    Device_Address = CAMERA_DEFAULT_ADDRESS + 2;
  }
  else if(port == PORT_4)
  {
    digitalWrite(s1,HIGH);
    digitalWrite(s2,HIGH);
    Device_Address = CAMERA_DEFAULT_ADDRESS + 3;
  }
  else
  { 
    digitalWrite(s1,LOW);
    digitalWrite(s2,LOW);
    Device_Address = CAMERA_DEFAULT_ADDRESS + 0;
  }
}
#else  // ME_PORT_DEFINED
/**
 * Alternate Constructor which can call your own function to map the AD0 and INT to arduino port,
 * no pins are used or initialized here
 * \param[in]
 *   AD0 - arduino gpio number
 * \param[in]
 *   INT - arduino gpio number
 */
MeCamera::MeCamera(uint8_t AD0, uint8_t INT)
{
  pinMode(AD0,OUTPUT);
  pinMode(INT,OUTPUT);

  digitalWrite(AD0,LOW);
  digitalWrite(INT,LOW);
  Device_Address = CAMERA_DEFAULT_ADDRESS + 0;
}

#endif // ME_PORT_DEFINED

/**
 * \par Function
 *   setpin
 * \par Description
 *   Set the PIN of the button module.
 * \param[in]
 *   AD0 - pin mapping for arduino
 * \param[in]
 *   INT - pin mapping for arduino
 * \par Output
 *   None
 * \return
 *   None.
 * \par Others
 *   Set global variable AD0, INT, s1 and s2
 */
void MeCamera::setpin(uint8_t AD0, uint8_t INT)
{
  pinMode(AD0,OUTPUT);
  pinMode(INT,OUTPUT);

#ifdef ME_PORT_DEFINED
  s1 = AD0;
  s2 = INT;
#endif // ME_PORT_DEFINED
}

/**
 * \par Function
 *   begin
 * \par Description
 *   Initialize the MeCamera.
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 *   
 */
void MeCamera::begin(void)
{
  motor_diff_speed = 0;
  kp = 0.5;
  pixy_work_mode = IDLE_MODE;
  led_change_flag = 1;
  
  Wire.begin();
  delay(50);
}

/**
 * \par Function
 *   setCameraMode
 * \par Description
 *   Set pixy2 work in mode: IDLE_MODE / CCC_MODE / LINE_MODE / VIDEO_MODE
 *   Setting mode delay time 500ms ~ 1500ms.
 * \param[in]
 *   mode: IDLE_MODE / CCC_MODE / LINE_MODE / VIDEO_MODE
 * \par Output
 *   None
 * \return
 *   0: set mode sucess
 *   -1:set mode fail
 * \par Others
 */
int8_t MeCamera::setCameraMode(uint8_t mode)
{
  int8_t return_value = 0;
  int8_t count = 3;

  if(mode != pixy_work_mode)
  {
    led_change_flag = 1;
  }

  while(count--)
  {
    //Serial.println("SET_PIXY_MODE");
    writeData(SET_PIXY_MODE, &mode, 1);
    delay(500);

    return_value = readData(SET_PIXY_MODE, &i2cData[0], 1);
    if((i2cData[0] == mode) && (return_value == I2C_OK))
    {
      pixy_work_mode = mode;
      motor_diff_speed = 0;
      return 0;
    }
  }
  return -1;
}

/**
 * \par Function
 *   setLineMode
 * \par Description
 *   Set line follow mode
 * \param[in]
 *   mode: 
 *   0 - find dark lines on a light background (black lines)
  *  1 - find light lines on a dark background (white lines)
 * \par Output
 *   None
 * \return
 *   0: set mode sucess
 *   -1:set mode fail
 * \par Others
 */
int8_t MeCamera::setLineMode(uint8_t mode)
{
  int8_t return_value = 0;
  int8_t count = 3;

  while(count--)
  {
    return_value = writeData(SET_LINE_FOLLOW_MODE, &mode, 1);
    delay(2);
    if(return_value == I2C_OK)
    {
      readData(SET_LINE_FOLLOW_MODE, &i2cData[0], 1);
      if(i2cData[0] == mode)
      {
        return 0;
      }
    }
  }
  return -1;
}

/**
 * \par Function
 *   setLED
 * \par Description
 *   Set Supplementary lamp status
 * \param[in]
 *   onOff: 
 *   0 - Turn off the supplementary light
  *  1 - Turn on the supplementary light
 * \par Output
 *   None
 * \return
 *   0: set sucess
 *   -1:set fail
 * \par Others
 */
int8_t MeCamera::setLED(uint8_t onOff)
{
  int8_t return_value = 0;
  int8_t count = 3;
  uint8_t data = 1;

  if(led_change_flag == 1)
  {
    return_value = writeData(SET_LED, &data, 1);
    led_change_flag = 0;
  }
  while(count--)
  {
    return_value = writeData(SET_LED, &onOff, 1);
    delay(2);
    if(return_value == I2C_OK)
    {
      readData(SET_LED, &i2cData[0], 1);
      if(i2cData[0] == onOff)
      {
        return 0;
      }
    }
  }
  return -1;
}

/**
 * \par Function
 *   startStudyCCC
 * \par Description
 *   Start enter study CCC
 * \param[in]
 *    sig - color signatures, numbered 1-7
 *    times - study timeout ,unit 1s
 * \par Output
 *   None
 * \return
 *   0: study sucess
 *   -1:study fail
 * \par Others
 */
int8_t MeCamera::startStudyCCC(uint8_t sig, uint8_t times)
{
  int8_t return_value = 0;
  uint8_t buff[2];

  buff[0] = sig - 1;
  buff[1] = times;

  return_value = writeData(SET_START_STUDY, &buff[0], 2);
  delay(100);

  while(1)
  {
    return_value = readData(GET_STUDY_STATE_ADDR, &i2cData[0], 1);
    if(return_value == I2C_OK)
    {
      if(i2cData[0] == 0)
      {
        //Serial.println("startStudyCCC");
        delay(2000);
        return 0;
      }
    }
    else
    {
      break;
    }
    delay(100);
  }
  return -1;
}

/**
 * \par Function
 *   getCCCValue
 * \par Description
 *   get CCC value
 * \param[in]
 *   sig - color signatures, numbered 1-7
 *   type - data type, CCC_X/CCC_Y/CCC_W/CCC_H
 * \par Output
 *   None
 * \return
 *   Return the value of CCC:
 *    - x-axis value ranges between 0 and 319
 *    - y-axis value ranges between 0 and 239
 *    - width value ranges between 0 and 320
 *    - height value ranges between 0 and 240
 * \par Others
 */
uint16_t MeCamera::getCCCValue(uint8_t sig, uint8_t type)
{
  int8_t return_value;
  uint32_t data = 0;

  if(type == CCC_NUM)
  {
    return_value = readPixy2Data(CCC_NUM_ADDR, &i2cData[0], 1);
  }
  else
  {
    return_value = readPixy2Data(CCC1_X_LOW_ADDR + (sig-1)*12 + type*2, &i2cData[0], 2);
  }

  if(return_value == I2C_OK)
  {
    data = *(uint16_t*)&i2cData[0];
  }

  if(type == CCC_X || type == CCC_W)
  {
    data = data * 320 / 316;
  }
  else if(type == CCC_Y || type == CCC_H)
  {
    data = data * 240 / 208;
  }
  return (uint16_t)data;
}

/**
 * \par Function
 *   getVectorValue
 * \par Description
 *   get the value of the Vector or line
 * \param[in]
 *   type - data type, VECTOR_X0/VECTOR_Y0/VECTOR_X1/VECTOR_Y1
 * \par Output
 *   None
 * \return
 *   Return the value of the vector:
 *   - x0 value ranges between 0 and 319
 *   - y0 value ranges between 0 and 239
 *   - x1 value ranges between 0 and 319
 *   - y1 value ranges between 0 and 239
 * \par Others
 */
uint16_t MeCamera::getVectorValue(uint8_t type)
{
  int8_t return_value;
  uint32_t data = 0;
  
  return_value = readPixy2Data(LINE_X0_ADDR + type, &i2cData[0], 1);
  if(return_value == I2C_OK)
  {
    data = *(uint8_t*)&i2cData[0];
  }

  if(type == VECTOR_X0 || type == VECTOR_X1)
  {
    data = data * 320 / 78;
  }
  else if(type == VECTOR_Y0 || type == VECTOR_Y1)
  {
    data = data * 240 / 52;
  }
  return (uint16_t)data;
}

/**
 * \par Function
 *   getIntersectionValue
 * \par Description
 *   get the value of the intersection
 * \param[in]
 *   type - data type, INTERSECTION_X/INTERSECTION_Y/INTERSECTION_NUM
 * \par Output
 *   None
 * \return
 *   Return the value of the intersection:
 *   - x-axis value ranges between 0 and 319
 *   - y-axis value ranges between 0 and 239
 *   - num the number of lines (branches), ranges between 0 and 6
 * \par Others
 */
uint16_t MeCamera::getIntersectionValue(uint8_t type)
{
  int8_t return_value;
  uint32_t data = 0;
  
  return_value = readPixy2Data(INTERSECTION_X_ADDR + type, &i2cData[0], 1);
  if(return_value == I2C_OK)
  {
    data = *(uint8_t*)&i2cData[0];
  }

  if(type == INTERSECTION_X)
  {
    data = data * 320 / 78;
  }
  else if(type == INTERSECTION_Y)
  {
    data = data * 240 / 52;
  }
  return (uint16_t)data;
}

/**
 * \par Function
 *   getIntersectionAngle
 * \par Description
 *   get the angle in degrees of the line
 * \param[in]
 *   index: the tracking index of the line, 1 ~ 6
 * \par Output
 *   None
 * \return
 *   Return the angle in degrees of the line, The value ranges between -180 and 180
 * \par Others
 */
int16_t MeCamera::getIntersectionAngle(uint8_t index)
{
  int8_t return_value;
  int16_t data = 0;

  if(index < 1 || index > 6)
  {
    return 0;
  }
  
  return_value = readPixy2Data(INTERSECTION_LINE1_ANGLE_LOW_ADDR + (index-1)*2  , &i2cData[0], 2);
  if(return_value == I2C_OK)
  {
    data = *(int16_t*)&i2cData[0];
  }
  return data;
}

/**
 * \par Function
 *   getBarCodeValue
 * \par Description
 *   get the value location of the barcode.
 * \param[in]
 *   code - value of the code, The value ranges between 0 and 15
 *   type - data type, BARCODE_X/BARCODE_Y
 * \par Output
 *   None
 * \return
 *   the value of the barcode:
 *   - x-axis value ranges between 0 and 319
 *   - y-axis value ranges between 0 and 239
 * \par Others
 */
uint16_t MeCamera::getBarCodeValue(uint8_t code, uint8_t type)
{
  int8_t return_value;
  uint32_t data = 0;

  if(type == BARCODE_NUM)
  {
    return_value = readPixy2Data(BARCODE_NUM_ADDR, &i2cData[0], 1);
  }
  else
  {
    return_value = readPixy2Data(BARCODE0_X_ADDR + code*2 + type, &i2cData[0], 1);
  }

  if(return_value == I2C_OK)
  {
    data = *(uint8_t*)&i2cData[0];
  }

  if(type == BARCODE_X)
  {
    data = data * 320 / 78;
  }
  else if(type == BARCODE_Y)
  {
    data = data * 240 / 52;
  }
  return (uint16_t)data;
}

/**
 * \par Function
 *   setMotorDiffSpeedKp
 * \par Description
 *   Set left/right Motor differential speed sensitivity
 * \param[in]
 *   kp: 0 ~ 1
 * \par Output
 *   None
 * \return
 *   None
 * \par Others
 */
void MeCamera::setMotorDiffSpeedKp(float _kp)
{
  if(_kp < 0)
  {
    kp = 0;
  }
  else  if(_kp > 1)
  {
    kp = 1;
  }
  else
  {
    kp = _kp;
  }
}

/**
 * \par Function
 *   getFollowColorValue
 * \par Description
 *   When automatically following the color block: Get left/right Motor differential speed value
 * \param[in]
 *    sig - color signatures, numbered 1-7
 *   x - x axis location in the image, 0 ~ 319, -1 invalid
 *   y - y axis location in the image, 0 ~ 239, -1 invalid
 * \par Output
 *   None
 * \return
 *   Motor differential speed value: -100 ~ 100
 * \par Others
 */
int16_t MeCamera::getFollowColorValue(uint8_t sig, int16_t x, int16_t y)
{
  int8_t return_value = 0;
  int32_t cur_x = 0, cur_y = 0;
  int16_t error = 0;

  if(pixy_work_mode != CCC_MODE)
  {
    return 0;
  }

  return_value = readPixy2Data(CCC1_X_LOW_ADDR + (sig-1)*12, &i2cData[0], 4);
  cur_x = *(int16_t*)&i2cData[0];
  cur_y = *(int16_t*)&i2cData[2];
  if(return_value != I2C_OK)
  {
    return 0;
  }
  if(cur_x == 0)
  {
    return 0;
  }

  cur_x = cur_x * 320 / 316;
  cur_y = cur_y * 240 / 208;
  if(x == -1)
  {
    error = (int16_t)(y - cur_y) * 2;
  }
  else if(y == -1)
  {
    error = (int16_t)(cur_x - x) * 1;
  }
  else
  {
    return 0;
  }

  error = constrain(error, -100, 100);
  error = error * kp;
  //LIMIT(error, 20);

  motor_diff_speed = error;
  return motor_diff_speed;
}

/**
 * \par Function
 *   getFollowBarcodeValue
 * \par Description
 *   When automatically following the barcode: Get left/right Motor differential speed value
 * \param[in]
 *   code - value of the barcode, The value ranges between 0 and 15
 *   x - x axis location in the image, 0 ~ 319, -1 invalid
 *   y - y axis location in the image, 0 ~ 239, -1 invalid
 * \par Output
 *   None
 * \return
 *   Motor differential speed value: -100 ~ 100
 * \par Others
 */
int16_t MeCamera::getFollowBarcodeValue(uint8_t code, int16_t x, int16_t y)
{
  int8_t return_value = 0;
  int32_t cur_x = 0, cur_y = 0;
  int16_t error = 0;

  if(pixy_work_mode != LINE_MODE)
  {
    return 0;
  }

  return_value = readPixy2Data(BARCODE0_X_ADDR + code*2, &i2cData[0], 2);
  cur_x = *(uint8_t*)&i2cData[0];
  cur_y = *(uint8_t*)&i2cData[1];
  if(return_value != I2C_OK)
  {
    return 0;
  }
  if(cur_x == 0)
  {
    return 0;
  }

  cur_x = cur_x * 320 / 78;
  cur_y = cur_y * 240 / 52;
  if(x == -1)
  {
    error = (int16_t)(y - cur_y) * 2;
  }
  else if(y == -1)
  {
    error = (int16_t)(cur_x - x) * 1;
  }
  else
  {
    return 0;
  }

  error = constrain(error, -100, 100);
  error = error * kp;
  //LIMIT(error, 20);

  motor_diff_speed = error;
  return motor_diff_speed;
}

/**
 * \par Function
 *   getFollowVectorValue
 * \par Description
 *   When automatically following the vector: Get left/right Motor differential speed value
 * \param[in]
 *   None
 * \par Output
 *   None
 * \return
 *   Motor differential speed value: -100 ~ 100
 * \par Others
 */
int16_t MeCamera::getFollowVectorValue(void)
{
  int8_t return_value = 0;
  int32_t x0, y0, x1, y1;
  int16_t error = 0;

  if(pixy_work_mode != LINE_MODE)
  {
    return 0;
  }

  return_value = readPixy2Data(LINE_X0_ADDR + VECTOR_X0, &i2cData[0], 4);
  x0 = *(uint8_t*)&i2cData[0];
  y0 = *(uint8_t*)&i2cData[1];
  x1 = *(uint8_t*)&i2cData[2];
  y1 = *(uint8_t*)&i2cData[3];
  if(return_value != I2C_OK)
  {
    return 0;
  }
  if(x0 == 0 && x1 == 0)
  {
    return 0;
  }

  x0 = x0 * 320 / 78;
  y0 = y0 * 240 / 52;
  x1 = x1 * 320 / 78;
  y1 = y1 * 240 / 52;

  error = (int16_t)(x0 - 160)*0.7 + (int16_t)(x1 - 160)*0.3;
  error = constrain(error, -100, 100);
  error = error * kp;
  //LIMIT(error, 20);

  motor_diff_speed = error;
  return motor_diff_speed;
}

/**
 * \par Function
 *   isLockedCCC
 * \par Description
 *   is locked the color block?
 * \param[in]
 *   sig - color signatures, numbered 1-7
 *   x - x axis location in the image, 0 ~ 319, -1 invalid
 *   y - y axis location in the image, 0 ~ 239, -1 invalid
 * \par Output
 *   None
 * \return
 *   true - Lock the target position
 *   false - No Lock the target position
 * \par Others
 */
bool MeCamera::isLockedCCC(uint8_t sig, int16_t x, int16_t y)
{
  int8_t return_value = 0;
  int32_t cur_x = 0, cur_y = 0;

  if(pixy_work_mode != CCC_MODE)
  {
    return false;
  }

  return_value = readPixy2Data(CCC1_X_LOW_ADDR + (sig-1)*12, &i2cData[0], 4);
  cur_x = *(int16_t*)&i2cData[0];
  cur_y = *(int16_t*)&i2cData[2];
  if((return_value != I2C_OK) || (cur_x == 0))
  {
    return false;
  }

  cur_x = cur_x * 320 / 316;
  cur_y = cur_y * 240 / 208;
  if(x == -1)
  {
    if(abs(cur_y - y) < 20)
    {
      return true;
    }
  }
  else if(y == -1)
  {
    if(abs(cur_x - x) < 20)
    {
      return true;
    }
  }
  else
  {
    if((abs(cur_y - y) < 20) && (abs(cur_x - x) < 20))
    {
      return true;
    }
  }
  return false;
}

/**
 * \par Function
 *   isLockedBarcode
 * \par Description
 *   is locked the Barcode?
 * \param[in]
 *   code - value of the barcode, The value ranges between 0 and 15
 *   x - x axis location in the image, 0 ~ 319, -1 invalid
 *   y - y axis location in the image, 0 ~ 239, -1 invalid
 * \par Output
 *   None
 * \return
 *   true - Lock the target position
 *   false - No Lock the target position
 * \par Others
 */
bool MeCamera::isLockedBarcode(uint8_t code, int16_t x, int16_t y)
{
  int8_t return_value = 0;
  int32_t cur_x = 0, cur_y = 0;

  if(pixy_work_mode != LINE_MODE)
  {
    return false;
  }

  return_value = readPixy2Data(BARCODE0_X_ADDR + code*2, &i2cData[0], 2);
  cur_x = *(uint8_t*)&i2cData[0];
  cur_y = *(uint8_t*)&i2cData[1];
  if((return_value != I2C_OK) || (cur_x == 0))
  {
    return false;
  }

  cur_x = cur_x * 320 / 78;
  cur_y = cur_y * 240 / 52;
  if(x == -1)
  {
    if(abs(cur_y - y) < 20)
    {
      return true;
    }
  }
  else if(y == -1)
  {
    if(abs(cur_x - x) < 20)
    {
      return true;
    }
  }
  else
  {
    if((abs(cur_y - y) < 20) && (abs(cur_x - x) < 20))
    {
      return true;
    }
  }

  return false;
}

/**
 * \par Function
 *   writeReg
 * \par Description
 *   Write the registor of i2c device.
 * \param[in]
 *   reg - the address of registor.
 * \param[in]
 *   data - the data that will be written to the registor.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   To set the registor for initializing.
 */
int8_t MeCamera::writeReg(uint8_t reg, uint8_t data)
{
  int8_t return_value = 0;
  return_value = writeData(reg, &data, 1);
  return(return_value);
}

/**
 * \par Function
 *   readData
 * \par Description
 *   Write the data to i2c device.
 * \param[in]
 *   start - the address which will write the data to.
 * \param[in]
 *   pData - the head address of data array.
 * \param[in]
 *   size - set the number of data will be written to the devide.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   Calling the official i2c library to read data.
 */
int8_t MeCamera::readData(uint8_t start, uint8_t *buffer, uint8_t size)
{
  int16_t i = 0;
  int8_t return_value = 0;

  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  return_value = Wire.endTransmission(false);
  if(return_value != 0)
  {
    return(return_value);
  }
  delayMicroseconds(1);
  /* Third parameter is true: relase I2C-bus after data is read. */
  Wire.requestFrom(Device_Address, size, (uint8_t)true);
  while(Wire.available() && i < size)
  {
    buffer[i++] = Wire.read();
  }
  return_value = Wire.endTransmission(true);
  delayMicroseconds(1);
  if(i != size)
  {
    return(I2C_ERROR);
  }
  return(0); //return: no error 
}

/**
 * \par Function
 *   readPixy2Data
 * \par Description
 *   Write the data to i2c device.
 * \param[in]
 *   start - the address which will write the data to.
 * \param[in]
 *   pData - the head address of data array.
 * \param[in]
 *   size - set the number of data will be written to the devide.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   Calling the official i2c library to read data.
 */
int8_t MeCamera::readPixy2Data(uint8_t start, uint8_t *buffer, uint8_t size)
{
  int8_t return_value;

  return_value = writeReg(start, 0x00);
  if(return_value == I2C_OK)
  {
    delay(2);
    return_value = readData(start, &buffer[0], size);
    if(return_value == I2C_OK)
    {
      return return_value;
    }
  }
  return(I2C_ERROR);
}

/**
 * \par Function
 *   writeData
 * \par Description
 *   Write the data to i2c device.
 * \param[in]
 *   start - the address which will write the data to.
 * \param[in]
 *   pData - the head address of data array.
 * \param[in]
 *   size - set the number of data will be written to the devide.
 * \par Output
 *   None
 * \return
 *   Return the error code.
 *   the definition of the value of variable return_value:
 *   0:success
 *   1:BUFFER_LENGTH is shorter than size
 *   2:address send, nack received
 *   3:data send, nack received
 *   4:other twi error
 *   refer to the arduino official library twi.c
 * \par Others
 *   Calling the official i2c library to write data.
 */
int8_t MeCamera::writeData(uint8_t start, const uint8_t *pData, uint8_t size)
{
  int8_t return_value = 0;
  Wire.beginTransmission(Device_Address);
  return_value = Wire.write(start);
  if(return_value != 1)
  {
    return(I2C_ERROR);
  }
  Wire.write(pData, size);
  return_value = Wire.endTransmission(true);
  return(return_value); //return: no error                     
}