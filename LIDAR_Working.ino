#include <VL53L1X.h>

/*
This example takes range measurements with the VL53L1X and displays additional 
details (status and signal/ambient rates) for each measurement, which can help
you determine whether the sensor is operating normally and the reported range is
valid. The range is in units of mm, and the rates are in units of MCPS (mega 
counts per second).
*/

#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;
//VL53L1X sensor2;
//VL53L1X sensor3;
long readDistance;

void setup()
{  
//  Serial.begin(9600);
//  Wire.begin();
//  Wire.setClock(400000); // use 400 kHz I2C

//  sensor.setTimeout(500);
//  if (!sensor.init())
//  {
//    Serial.println("Failed to detect and initialize sensor!");
//    while (1);
//  }
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
//  pinMode(10, OUTPUT);
//  pinMode(11, OUTPUT);
  digitalWrite(9, LOW);
//  digitalWrite(10, LOW);
//  digitalWrite(11, LOW);

  delay(500);
  Wire.begin();


  Serial.begin (9600);

  sensor.setTimeout(500);
//  sensor2.setTimeout(500);
//  sensor3.setTimeout(500);
  
  //SENSOR
  digitalWrite(9, HIGH);
  delay(150);
  //Serial.println("00");
  sensor.init(true);
  //Serial.println("01");
  delay(100);
  sensor.setAddress((uint8_t)22);
  //Serial.println("02");

//  //SENSOR 2
//  digitalWrite(10, HIGH);
//  delay(150);
//  sensor2.init(true);
//  //Serial.println("03");
//  delay(100);
//  sensor2.setAddress((uint8_t)25);
//  //Serial.println("04");
//
//  //SENSOR 3
//  digitalWrite(11, HIGH);
//  delay(150);
//  sensor3.init(true);
//  //Serial.println("05");
//  delay(100);
//  sensor3.setAddress((uint8_t)28);
//  //Serial.println("06");

  
  // Use long distance mode and allow up to 50000 us (50 ms) for a measurement.
  // You can change these settings to adjust the performance of the sensor, but
  // the minimum timing budget is 20 ms for short distance mode and 33 ms for
  // medium and long distance modes. See the VL53L1X datasheet for more
  // information on range and timing limits.
  sensor.setDistanceMode(VL53L1X::Long);
  sensor.setMeasurementTimingBudget(50000);

  // Start continuous readings at a rate of one measurement every 50 ms (the
  // inter-measurement period). This period should be at least as long as the
  // timing budget.
  sensor.startContinuous(50);


//  sensor2.setDistanceMode(VL53L1X::Long);
//  sensor2.setMeasurementTimingBudget(15000);
//
//  sensor2.startContinuous(15);
//
//
//  sensor3.setDistanceMode(VL53L1X::Long);
//  sensor3.setMeasurementTimingBudget(15000);
//
//  sensor3.startContinuous(15);
}

void loop()
{
//  static long int lastTime = 0;
//  long int curTime = millis();
//  Serial.print(",");
//  Serial.print(curTime - lastTime);
//  Serial.print(",");
//  lastTime = curTime;

//  long int startTime = millis();
  sensor.read();
  readDistance = sensor.ranging_data.range_mm;
//  long int stopTime = millis();
//  Serial.print(",");
//  Serial.print(stopTime - startTime);
//  Serial.print(",");
  
  
  Serial.print("ReadDistance: ");
  Serial.println(readDistance);

  Serial.print("SensorDistance: ");
  Serial.println(sensor.ranging_data.range_mm);
//Serial.println(sensor.read());
//  Serial.println("E");

if(readDistance <= 210) {
  digitalWrite(5, HIGH);
}
else {
  digitalWrite(5, LOW);
}
  

//  sensor2.read();
//  Serial.print("b");
//  Serial.print(sensor2.ranging_data.range_mm);
//  Serial.println("e");
//
//
//  sensor3.read();
//  Serial.print("X");
//  Serial.print(sensor3.ranging_data.range_mm);
//  Serial.print("Z");

  
    
  Serial.println();
}
