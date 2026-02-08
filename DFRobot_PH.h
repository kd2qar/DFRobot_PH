/*!
 * @file DFRobot_PH.h
 * @brief Arduino library for Gravity: Analog pH Sensor / Meter Kit V2, SKU: SEN0161-V2
 *
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license     The MIT License (MIT)
 * @author [Jiawei Zhang](jiawei.zhang@dfrobot.com)
 * @version  V1.0
 * @date  2018-11-06
 * @url https://github.com/DFRobot/DFRobot_PH
 */

#ifndef _DFROBOT_PH_H_
#define _DFROBOT_PH_H_

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ReceivedBufferLength 10  //length of the Serial CMD buffer

class DFRobot_PH
{
public:
  DFRobot_PH();
  ~DFRobot_PH();
  /**
   * @fn calibration
   * @brief Calibrate the calibration data
   *
   * @param voltage     : Voltage value
   * @param temperature : Ambient temperature
   * @param cmd         : enterph -> enter the PH calibration mode
   * @n                   calph   -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
   * @n                   exitph  -> save the calibrated parameters and exit from PH calibration mode
   */
  void    calibration(float voltage, float temperature,char* cmd);  //calibration by Serial CMD
  void    calibration(float voltage, float temperature);
  /**
   * @fn calibration
   * @brief Calibrate the calibration data
   *
   * @param voltage     : Voltage value
   * @param temperature : Ambient temperature
   * @param mode         : 1 -> enter the PH calibration mode
   * @n                    2 -> calibrate with the standard buffer solution, two buffer solutions(4.0 and 7.0) will be automaticlly recognized
   * @n                    3 -> save the calibrated parameters and exit from PH calibration mode
   */ 
  void    calibration(float voltage, float temperature, byte mode);
  /**
   * @fn readPH
   * @brief Convert voltage to PH with temperature compensation
   * @note voltage to pH value, with temperature compensation
   *
   * @param voltage     : Voltage value
   * @param temperature : Ambient temperature
   * @return The PH value
   */
  float   readPH(float voltage, float temperature); 
  /**
   * @fn begin
   * @brief Initialization The Analog pH Sensor
   */
  void begin();
  void begin(unsigned int phValueAddress);

private:
    float  _phValue;
    float  _acidVoltage;
    float  _neutralVoltage;
    float  _voltage;
    float  _temperature;

    char   _cmdReceivedBuffer[ReceivedBufferLength];  //store the Serial CMD
    byte   _cmdReceivedBufferIndex;

private:
    boolean cmdSerialDataAvailable();
    void    phCalibration(byte mode); // calibration process, wirte key parameters to EEPROM
    byte    cmdParse(const char* cmd);
    byte    cmdParse();
	char* strupr(char* str);
private:
	byte _phValueAddress = 0x0;
};

#endif
