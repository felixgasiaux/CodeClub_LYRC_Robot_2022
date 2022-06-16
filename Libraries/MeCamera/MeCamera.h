/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * \class   MeCamera
 * \brief   Driver for MeCamera module.
 * @file   MeCamera.h
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2017/09/26
 * @brief   Header for MeCamera.cpp module.
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
 *  Payton            2017/09/26          1.0.0         rebuild the old lib.
 * </pre>
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MeCamera_H
#define MeCamera_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include "MeConfig.h"
#ifdef ME_PORT_DEFINED
#include "MePort.h"
#endif // ME_PORT_DEFINED

/* Exported macro ------------------------------------------------------------*/
#define I2C_ERROR                  						(-1)
#define I2C_OK                    						(0)
#define CAMERA_DEFAULT_ADDRESS        (0x25)
#define CAMERA_DEVICE_ID							(0xC1)

#define CAMERA_NONE_DATA              (0x1F)
#define CAMERA_NONE_BIG_DATA          (0x1F1F)

// pixy work mode
#define IDLE_MODE  0
#define CCC_MODE    1
#define LINE_MODE   2
#define VIDEO_MODE  3

// SIG INDEX
#define SIG1    1
#define SIG2    2
#define SIG3    3
#define SIG4    4
#define SIG5    5
#define SIG6    6
#define SIG7    7

//CCC value type
#define  CCC_X   0
#define  CCC_Y   1
#define  CCC_W   2
#define  CCC_H   3
#define  CCC_NUM   4

//Vector value type
#define  VECTOR_X0   0
#define  VECTOR_Y0   1
#define  VECTOR_X1   2
#define  VECTOR_Y1   3

//intersection value type
#define  INTERSECTION_X       0
#define  INTERSECTION_Y       1
#define  INTERSECTION_NUM     2

//barcode value type
#define  BARCODE_X         0
#define  BARCODE_Y         1
#define  BARCODE_NUM       2

//video value type
#define  VIDEO_R         0
#define  VIDEO_G         1
#define  VIDEO_B         2


#define LIMIT(x, a) if( (((x) >= 0) && ((x) < (a))) ) (x) = a;\
                    else if( (((x) < 0) && ((x) >= -(a))) ) (x) = -a;


//Camera IIC Register Address
#define CAMERA_DEVICE_ID_ADDR   (0x01)
#define  CCC1_X_LOW_ADDR        0X01
#define  CCC1_X_HIGH_ADDR       0X02
#define  CCC1_Y_LOW_ADDR        0X03
#define  CCC1_Y_HIGH_ADDR       0X04
#define  CCC1_W_LOW_ADDR        0X05
#define  CCC1_W_HIGH_ADDR       0X06
#define  CCC1_H_LOW_ADDR        0X07
#define  CCC1_H_HIGH_ADDR       0X08
#define  CCC1_ANGLE_LOW_ADDR    0X09
#define  CCC1_ANGLE_HIGH_ADDR   0X0A
#define  CCC1_INDEX_LOW_ADDR    0X0B
#define  CCC1_INDEX_HIGH_ADDR   0X0C
#define  CCC2_X_LOW_ADDR        0X0D
#define  CCC2_X_HIGH_ADDR       0X0E
#define  CCC2_Y_LOW_ADDR        0X0F
#define  CCC2_Y_HIGH_ADDR       0X10
#define  CCC2_W_LOW_ADDR        0X11
#define  CCC2_W_HIGH_ADDR       0X12
#define  CCC2_H_LOW_ADDR        0X13
#define  CCC2_H_HIGH_ADDR       0X14
#define  CCC2_ANGLE_LOW_ADDR    0X15
#define  CCC2_ANGLE_HIGH_ADDR   0X16
#define  CCC2_INDEX_LOW_ADDR    0X17
#define  CCC2_INDEX_HIGH_ADDR   0X18
#define  CCC3_X_LOW_ADDR        0X19
#define  CCC3_X_HIGH_ADDR       0X1A
#define  CCC3_Y_LOW_ADDR        0X1B
#define  CCC3_Y_HIGH_ADDR       0X1C
#define  CCC3_W_LOW_ADDR        0X1D
#define  CCC3_W_HIGH_ADDR       0X1E
#define  CCC3_H_LOW_ADDR        0X1F
#define  CCC3_H_HIGH_ADDR       0X20
#define  CCC3_ANGLE_LOW_ADDR    0X21
#define  CCC3_ANGLE_HIGH_ADDR   0X22
#define  CCC3_INDEX_LOW_ADDR    0X23
#define  CCC3_INDEX_HIGH_ADDR   0X24
#define  CCC4_X_LOW_ADDR        0X25
#define  CCC4_X_HIGH_ADDR       0X26
#define  CCC4_Y_LOW_ADDR        0X27
#define  CCC4_Y_HIGH_ADDR       0X28
#define  CCC4_W_LOW_ADDR        0X29
#define  CCC4_W_HIGH_ADDR       0X2A
#define  CCC4_H_LOW_ADDR        0X2B
#define  CCC4_H_HIGH_ADDR       0X2C
#define  CCC4_ANGLE_LOW_ADDR    0X2D
#define  CCC4_ANGLE_HIGH_ADDR   0X2E
#define  CCC4_INDEX_LOW_ADDR    0X2F
#define  CCC4_INDEX_HIGH_ADDR   0X30
#define  CCC5_X_LOW_ADDR        0X31
#define  CCC5_X_HIGH_ADDR       0X32
#define  CCC5_Y_LOW_ADDR        0X33
#define  CCC5_Y_HIGH_ADDR       0X34
#define  CCC5_W_LOW_ADDR        0X35
#define  CCC5_W_HIGH_ADDR       0X36
#define  CCC5_H_LOW_ADDR        0X37
#define  CCC5_H_HIGH_ADDR       0X38
#define  CCC5_ANGLE_LOW_ADDR    0X39
#define  CCC5_ANGLE_HIGH_ADDR   0X3A
#define  CCC5_INDEX_LOW_ADDR    0X3B
#define  CCC5_INDEX_HIGH_ADDR   0X3C
#define  CCC6_X_LOW_ADDR        0X3D
#define  CCC6_X_HIGH_ADDR       0X3E
#define  CCC6_Y_LOW_ADDR        0X3F
#define  CCC6_Y_HIGH_ADDR       0X40
#define  CCC6_W_LOW_ADDR        0X41
#define  CCC6_W_HIGH_ADDR       0X42
#define  CCC6_H_LOW_ADDR        0X43
#define  CCC6_H_HIGH_ADDR       0X44
#define  CCC6_ANGLE_LOW_ADDR    0X45
#define  CCC6_ANGLE_HIGH_ADDR   0X46
#define  CCC6_INDEX_LOW_ADDR    0X47
#define  CCC6_INDEX_HIGH_ADDR   0X48
#define  CCC7_X_LOW_ADDR        0X49
#define  CCC7_X_HIGH_ADDR       0X4A
#define  CCC7_Y_LOW_ADDR        0X4B
#define  CCC7_Y_HIGH_ADDR       0X4C
#define  CCC7_W_LOW_ADDR        0X4D
#define  CCC7_W_HIGH_ADDR       0X4E
#define  CCC7_H_LOW_ADDR        0X4F
#define  CCC7_H_HIGH_ADDR       0X50
#define  CCC7_ANGLE_LOW_ADDR    0X51
#define  CCC7_ANGLE_HIGH_ADDR   0X52
#define  CCC7_INDEX_LOW_ADDR    0X53
#define  CCC7_INDEX_HIGH_ADDR   0X54
#define  CCC_NUM_ADDR           0X55

#define LINE_X0_ADDR            0X56
#define LINE_Y0_ADDR            0X57
#define LINE_X1_ADDR            0X58
#define LINE_Y1_ADDR            0X59

#define INTERSECTION_X_ADDR     0X5A
#define INTERSECTION_Y_ADDR     0X5B
#define INTERSECTION_NUM_ADDR   0X5C
#define INTERSECTION_LINE1_ANGLE_LOW_ADDR   0X5D
#define INTERSECTION_LINE1_ANGLE_HIGH_ADDR  0X5E
#define INTERSECTION_LINE2_ANGLE_LOW_ADDR   0X5F
#define INTERSECTION_LINE2_ANGLE_HIGH_ADDR  0X60
#define INTERSECTION_LINE3_ANGLE_LOW_ADDR   0X61
#define INTERSECTION_LINE3_ANGLE_HIGH_ADDR  0X62
#define INTERSECTION_LINE4_ANGLE_LOW_ADDR   0X63
#define INTERSECTION_LINE4_ANGLE_HIGH_ADDR  0X64
#define INTERSECTION_LINE5_ANGLE_LOW_ADDR   0X65
#define INTERSECTION_LINE5_ANGLE_HIGH_ADDR  0X66
#define INTERSECTION_LINE6_ANGLE_LOW_ADDR   0X67
#define INTERSECTION_LINE6_ANGLE_HIGH_ADDR  0X68
#define INTERSECTION_INDEX_LOW_ADDR   0X69
#define INTERSECTION_INDEX_HIGH_ADDR  0X6A

#define BARCODE_NUM_ADDR        0X6F
#define BARCODE0_X_ADDR         0X70
#define BARCODE0_Y_ADDR         0X71
#define BARCODE1_X_ADDR         0X72
#define BARCODE1_Y_ADDR         0X73
#define BARCODE2_X_ADDR         0X74
#define BARCODE2_Y_ADDR         0X75
#define BARCODE3_X_ADDR         0X76
#define BARCODE3_Y_ADDR         0X77
#define BARCODE4_X_ADDR         0X78
#define BARCODE4_Y_ADDR         0X79
#define BARCODE5_X_ADDR         0X7A
#define BARCODE5_Y_ADDR         0X7B
#define BARCODE6_X_ADDR         0X7C
#define BARCODE6_Y_ADDR         0X7D
#define BARCODE7_X_ADDR         0X7E
#define BARCODE7_Y_ADDR         0X7F
#define BARCODE8_X_ADDR         0X80
#define BARCODE8_Y_ADDR         0X81
#define BARCODE9_X_ADDR         0X82
#define BARCODE9_Y_ADDR         0X83
#define BARCODE10_X_ADDR        0X84
#define BARCODE10_Y_ADDR        0X85
#define BARCODE11_X_ADDR        0X86
#define BARCODE11_Y_ADDR        0X87
#define BARCODE12_X_ADDR        0X88
#define BARCODE12_Y_ADDR        0X89
#define BARCODE13_X_ADDR        0X8A
#define BARCODE13_Y_ADDR        0X8B
#define BARCODE14_X_ADDR        0X8C
#define BARCODE14_Y_ADDR        0X8D
#define BARCODE15_X_ADDR        0X8E
#define BARCODE15_Y_ADDR        0X8F

#define RGB_R_ADDR              0X90
#define RGB_G_ADDR              0X91
#define RGB_B_ADDR              0X92
#define RGB_SATURATION_ADDR     0X93
#define RGB_GRAY_ADDR           0X94
#define RGB_X_ADDR              0X95
#define RGB_Y_ADDR              0X97

#define GET_KEY_STATE_ADDR      0X9A
#define GET_STUDY_STATE_ADDR    0X9B

#define SET_LINE_FOLLOW_MODE    0XA0
#define SET_VECTOR_MODE         0XA1
#define SET_DEFAULT_TURN_ANGLE  0XA2
#define SET_NEXT_TURN_ANGLE     0XA4
#define SET_CAMERA_BRIGHNESS    0XA6
#define SET_RGB                 0XA7
#define SET_LED                 0XAA
#define SET_PIXY_MODE           0XAB
#define SET_START_STUDY         0XAC
#define SET_STUDY_TIME          0XAD

/**
 * Class: MeCamera
 * \par Description
 * Declaration of Class MeCamera
 */
#ifndef ME_PORT_DEFINED
class MeCamera
#else // !ME_PORT_DEFINED
class MeCamera : public MePort
#endif // !ME_PORT_DEFINED
{
public:
#ifdef ME_PORT_DEFINED

/**
 * Alternate Constructor which can call your own function to map the MeCompass to arduino port
 * no pins are used or initialized here, but PWM frequency set to 976 Hz
 * \param[in]
 *   port - RJ25 port from PORT_1 to PORT_4
 */
  MeCamera(uint8_t port);
#else
/**
 * Alternate Constructor which can call your own function to map the AD0 and INT to arduino port,
 * no pins are used or initialized here
 * \param[in]
 *   AD0 - arduino gpio number
 * \param[in]
 *   INT - arduino gpio number
 */
 MeCamera(uint8_t AD0, uint8_t INT);

#endif  //  ME_PORT_DEFINED
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
 *   Set global variable _AD0, _INT, s1 and s2
 */
  void setpin(uint8_t AD0, uint8_t INT);

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
 *   None
 */
  void begin();

/**
 * \par Function
 *   setCameraMode
 * \par Description
 *   Set camera in mode: IDLE_MODE / CCC_MODE / LINE_MODE / VIDEO_MODE
 *   Setting mode delay time 500ms ~ 1500ms.
 *   mode: IDLE_MODE / CCC_MODE / LINE_MODE / VIDEO_MODE
 * \par Output
 *   None
 * \return
 *   0:  set sucess
 *   -1: set fail
 * \par Others
 */
  int8_t setCameraMode(uint8_t mode);

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
 *   0: set sucess
 *   -1:set fail
 * \par Others
 */
  int8_t setLineMode(uint8_t mode);

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
  int8_t setLED(uint8_t onOff);

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
 *   0: study  sucess
 *   -1:study  fail
 * \par Others
 */
  int8_t startStudyCCC(uint8_t sig, uint8_t times);

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
  uint16_t getCCCValue(uint8_t sig, uint8_t type);

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
  uint16_t getVectorValue(uint8_t type);

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
  uint16_t getIntersectionValue(uint8_t type);

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
  int16_t getIntersectionAngle(uint8_t index);

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
  uint16_t getBarCodeValue(uint8_t code, uint8_t type);

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
  void setMotorDiffSpeedKp(float _kp);

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
  int16_t getFollowColorValue(uint8_t sig, int16_t x, int16_t y);

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
  int16_t getFollowBarcodeValue(uint8_t code, int16_t x, int16_t y);

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
  int16_t getFollowVectorValue(void);

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
  bool isLockedCCC(uint8_t sig, int16_t x, int16_t y);

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
  bool isLockedBarcode(uint8_t code, int16_t x, int16_t y);

private:
  uint8_t i2cData[8];
  uint8_t Device_Address;
  uint8_t pixy_work_mode;
  int16_t motor_diff_speed;
  uint8_t led_change_flag;
  float kp;
  
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
  int8_t writeReg(uint8_t reg, uint8_t data);

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
  int8_t readData(uint8_t start, uint8_t *buffer, uint8_t size);

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
  int8_t readPixy2Data(uint8_t start, uint8_t *buffer, uint8_t size);


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
  int8_t writeData(uint8_t start, const uint8_t *pData, uint8_t size);
};
#endif //  MeCamera_H
