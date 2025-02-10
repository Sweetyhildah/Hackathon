#    "IoT based Real Time Automobile Dashboard using I2C Protocol"

## Introduction

<p align="justify">
  
The IoT-based Real-Time Automobile Dashboard, powered by the innovative VSD Squadron, transforms vehicle monitoring by integrating advanced technology and IoT capabilities. The VSD Squadron acts as the central microcontroller, managing communication between sensors, displays, and the cloud using the efficient I2C protocol. This system provides real-time updates on key metrics like speed, fuel levels, engine temperature, and location. Data is processed locally and transmitted to the cloud, allowing for remote access and monitoring. The dashboard not only displays live data but also sends alerts for anomalies like low fuel or overheating. With GPS integration, drivers can track their vehicle’s location in real-time. The IoT connectivity enhances safety, maintenance, and performance optimization. The VSD Squadron makes driving smarter by providing actionable insights and timely notifications. This dashboard combines cutting-edge technology with convenience, making it the ultimate tool for modern vehicle management. It’s not just a dashboard; it’s an intelligent, connected co-pilot for today’s drivers.

## Overview
<p align="justify">
  
The IoT-based Real-Time Automobile Dashboard powered by the VSD Squadron integrates advanced technology to monitor key vehicle parameters like speed, fuel levels, engine temperature, and location. Using the I2C protocol, the system ensures efficient communication between sensors, displays, and IoT modules. The VSD Squadron processes the data and displays it in real-time on a dashboard, while also sending it to the cloud for remote monitoring. With GPS integration, alerts for anomalies, and cloud connectivity, the system enhances vehicle safety, performance, and management, offering a smarter, more connected driving experience.
<img width="658" alt="1" src="https://github.com/user-attachments/assets/8451520d-7c3d-4769-857c-0a64bb774ab6" />

## Key Features and Benifits of the Product
<p align="justify">
The selected text outlines the proposed solution for a real-time Tire Pressure Monitoring System (TPMS). ​ This system aims to control various performance metrics of an automobile, specifically focusing on air pressure in tires, wheel alignment, and wheel balancing. ​ The key features of this solution include:

##### Real-time monitoring: Continuously tracks tire pressure and other related metrics in real-time. ​
##### Simplified communication: Utilizes a streamlined communication protocol to reduce complexity. ​
##### User-friendly interface: Ensures that the system is easy to use and interact with for the end-users.

## Technical Approach
<p align="justify">
  
A RISC-V architecture is used for complete functioning of the system.

**Hardware:** Sensor for measuring pressure (BMP180), VSD Squadron Mini, LCD Display, ESP32 (Wi-Fi module)

**Software:** VS Code, Platform IO, CH32V platform 

**Communication:** I2C for data transmission

**Interface:** I2C interface to connect to display, Thingspeak IoT cloud platform to aggregate, visualize and analyze live data stream in the cloud.



## Components required with Bill of Materials
|Item                                        | Quantity             | Description                          | Links to Products                  |
|--------------------------------------------|---------------------|--------------------------------------|------------------------------------|
|MPU6050                                     | 1                     |  Gyroscope and Accelerometer sensor   |https://sulkurl.com/kR7            |
|VSDSquadron Mini RISC-V | 1|  Microcontroller          |                  https://sulkurl.com/kR9|
|BMP180 |1|Digital Barometric Pressure Sensor Module  |https://sulkurl.com/kSg|

## Table for Pin Connections
### VSD Squadron (Microcontroller):
     
|VSD Squadron Pin                            | I2C Pin Component               |Function            |
|--------------------------------------------|-----------------------|----------------------------------------|
|SDA (A4)                                    | I2C SDA                    |Serial Data Line (Data transfer)        |
|SCL (A5)                                    | I2C SCL                    |Serial Clock Line (Clock signal)        |
|VCC                                         | I2C VCC                    |Power supply for I2C device (3.3V or 5V)|
|GND                                       |  I2C GND                    |Ground (Common reference)               |

### MPU6050 Gyroscope:

|VSD Squadron Pin                            | MPU6050 Pin Component       |Function            |
|--------------------------------------------|---------------------------|----------------------------------------|
|SDA (A4)                                   |MPU6050 Pin SDA               |Serial Data Line (Data transfer)       |
|SCL (A5))                                  | MPU6050 Pin SCL              |Serial Clock Line (Clock signal        |
|VCC                                        | MPU6050 Pin VCC              |Power supply for I2C device (3.3V or 5V)|
|GND                                        | MPU6050 Pin GND              |Ground (Common reference)               |

### 	BMP180:

|VSD Squadron Pin                            | Pressure Sensor Pin      |Function            |
|--------------------------------------------|---------------------------|----------------------------------------|
|SDA (A4)                                   |MPU6050 Pin SDA               |Serial Data Line (Data transfer)       |
|SCL (A5))                                  | MPU6050 Pin SCL              |Serial Clock Line (Clock signal        |
|VCC                                        | MPU6050 Pin VCC              |Power supply for I2C device (3.3V or 5V)|
|GND                                        | MPU6050 Pin GND              |Ground (Common reference)               |

### ESP8266 WiFi Module:

|ESP8266 Pin                                        | VSD Squadron Pin    |Function                                     |
|--------------------------------------------------|---------------------------|----------------------------------------|
|VCC                                                 |VCC              |Serial Data Line (Data transfer)                  |
|GND                                                  |GND               |Serial Data Line (Data transfer)                 |
|TX (Transmit)                                        |TX             |Serial Data Line (Data transfer)                    |
|RX (Receive)                                         |RX              |Serial Data Line (Data transfer)                  |

    
## Pinout Diagram


    
![2](https://github.com/user-attachments/assets/8a75f0cd-7507-4f94-a562-39140cf85bfa)



## Source code with Arduino IDE 

```
#include <math.h>
#include <LiquidCrystal_I2C.h> // Driver Library for the LCD Module
#include <Wire.h>
#include <Adafruit_BMP085.h>
#define seaLevelPressure_hPa 1013.25
Adafruit_BMP085 bmp;

const int x_out = A1; /* connect x_out of module to A1 of UNO board */
const int y_out = A2; /* connect y_out of module to A2 of UNO board */
const int z_out = A3; /* connect z_out of module to A3 of UNO board */


LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x3F,16,2);


float Pressure,temperature_C,temperature_F;


void lcd_s(String lcd_status ,float lcd_pressure){

  lcd.setCursor(0,0);
  lcd.print("Status:");
  lcd.print(lcd_status);
  lcd.setCursor(0,1);
  lcd.print("Press:");
  lcd.print(lcd_pressure);
  lcd.print("psi");
}

void gyro() {
  int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  double roll, pitch, yaw;
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 
  Serial.print("x = ");
  Serial.print(x_adc_value);
  Serial.print("\t\t");
  Serial.print("y = ");
  Serial.print(y_adc_value);
  Serial.print("\t\t");
  Serial.print("z = ");
  Serial.print(z_adc_value);
  Serial.print("\t\t");
  //delay(100);
  
  x_g_value = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 

  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 ); /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 ); /* Formula for pitch */
  yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 ); /* Formula for yaw */
  /* Not possible to measure yaw using accelerometer. Gyroscope must be used if yaw is also required */

  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print("Yaw = ");
  Serial.print(yaw);
  Serial.print("\n\n");
  delay(1000);
}
void pressure() {
    //Serial.print("Temperature = ");
   // float t = bmp.readTemperature();
    //Serial.print(t);
    //Serial.print(bmp.readTemperature()); 
    //Serial.println(" *C");

    Serial.print("Pressure = ");
    float pressure = bmp.readPressure();
    float  p = pressure / 6894.76;
    Serial.print(p);
    Serial.println(" Psi");
      String s;
 
  if((p > 14))
  {
    s = "Safe";
    //Serial.println("Risk");
  }
  else 
  {
     s = "Risk";
    //Serial.println("Safe");
  }
  Serial.println("Status : "+ s);

    Serial.println();
    delay(500);
    lcd_s(s,p);
}

void setup(){
  Serial.begin(115200);
 // wifi();
  lcd.init();          // Initiate the LCD module
  lcd.backlight();     // Turn on the backlight
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
}
 
void loop()
{
 // dht_s();
    gyro();
    pressure();
  //thing();l
}
```


## Demonstation Video with Arduino IDE

https://drive.google.com/file/d/10zW7UOl_aDTQp0_erywTDyBhxi6431aL/view?usp=sharing

## Demonstation Video with VSDSquadron Mini RISC-V Microcontroller Board

https://drive.google.com/file/d/113H-x_Sz_JgadCqJOI0I7lY1-Dg_z7pm/view?usp=sharing



## Conclusion
<p align="justify">
The system designed and developed as part of this project provides a comprehensive solution for monitoring and maintaining optimal tire pressure which leads to wheel balancing and wheel alignment. This project demonstrates the potential for technology to improve vehicle safety, efficiency, and performance. Its successful implementation can have a significant impact on the automotive industry.
    
    
 
