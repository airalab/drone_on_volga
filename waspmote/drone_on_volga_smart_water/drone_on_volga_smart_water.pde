/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *  Water parameters monitoring sensor firmware
 *  For Libelium Plug&Sense Smart Water
 *  2018, Alisher Khassanov <alisher@aira.life>
 *  BSD 3-Clause License
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * 
 *  Wiring
 *  - PTSM     : Temperature Sensor (Pt-1000)
 *  - SOCKET_A : pH
 *  - SOCKET_B : Dissolved Oxygen (DO)
 *  - SOCKET_C : Conductivity
 *  - SOCKET_D : NC
 *  - SOCKET_E : NC
 *  - SOCKET_F : NC
 *  
 *  Calibration
 *  - Sensors works since July 2018
 *  - Latest calibration 03.08.2018
 *  - Latest check 03.08.2018
 *  
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include <WaspSensorSW.h>
#include <WaspFrame.h>

char node_ID[] = "DOV01SW";

// pH Sensor calibration values
#define cal_point_10 1.954 // at 27.980 toC
#define cal_point_7 2.073  // at 28.167 toC
#define cal_point_4 2.240  // at 28.135 toC
#define cal_temp 28.094    // Temperature at which calibration was carried out

// Dissolved Oxygen (DO) Sensor calibration values
#define air_calibration 4.092   //  In normal air
#define zero_calibration 0.00067 // Calibration of the sensor under 0% solution

// Conductivity Sensor calibration values
#define point1_cond 12880 // Value 1 used to calibrate the sensor
//#define point2_cond 80000 // Value 2 used to calibrate the sensor
#define point2_cond 84 // Value 2 used to calibrate the sensor
#define point1_cal 103.519 // Point 1 of the calibration
//#define point2_cal 33.18 // Point 2 of the calibration
#define point2_cal 8255.698 // Point 2 of the calibration

pHClass pHSensor;
DOClass DOSensor;
conductivityClass ConductivitySensor;
pt1000Class TemperatureSensor;

float value_temp;
float value_pH;
float value_pH_calculated;
float value_do;
float value_do_calculated;
float value_cond;
float value_cond_calculated;

void setup()
{
  USB.ON();
  frame.setID(node_ID);
  
  // Configure the calibration values
  pHSensor.setCalibrationPoints(cal_point_10, cal_point_7, cal_point_4, cal_temp);
  DOSensor.setCalibrationPoints(air_calibration, zero_calibration);
  ConductivitySensor.setCalibrationPoints(point1_cond, point1_cal, point2_cond, point2_cal);  
}

void loop()
{
  Water.ON();
  delay(2000);

  value_temp = TemperatureSensor.readTemperature();
  value_pH = pHSensor.readpH();
  value_pH_calculated = pHSensor.pHConversion(value_pH, value_temp); // temperature correction
  value_do = DOSensor.readDO();
  value_do_calculated = DOSensor.DOConversion(value_do); // from volts into dissolved oxygen percentage
  value_cond = ConductivitySensor.readConductivity();
  value_cond_calculated = ConductivitySensor.conductivityConversion(value_cond); // from resistance into ms/cm

  Water.OFF();

  frame.createFrame(ASCII);
  frame.addSensor(SENSOR_WATER_WT, value_temp); // toC
  frame.addSensor(SENSOR_WATER_PH, value_pH_calculated); // 0-14
  frame.addSensor(SENSOR_WATER_DO, value_do_calculated); // 0-100%
  frame.addSensor(SENSOR_WATER_COND, value_cond_calculated); // muS/cm

  USB.println((char*) frame.buffer);

  delay(2000);
}
