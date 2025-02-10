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

## Source code with VSDSquadron Mini RISC-V Microcontroller

```
// Defining the SDA and SCL Pins for I2C Communication
#include <debug.h>
#include <ch32v00x.h>
#include <ch32v00x_gpio.h>

// Defining the SDA and SCL Pins for I2C Communication
#define SDA_PIN GPIO_Pin_1
#define SCL_PIN GPIO_Pin_2


// Defining the LCD Address
#define LCD_Address 0x27

// BMP180 I2C Address
#define BMP180_Address 0x77

void lcd_send_cmd(unsigned char cmd);
void lcd_send_data(unsigned char data);
void lcd_send_str(unsigned char *str);
void lcd_init(void);
void delay_ms(unsigned int ms);
void i2c_start(void);
void i2c_stop(void);
void i2c_write(unsigned char dat);
void i2c_ACK(void);

int32_t bmp180_read_pressure(void);
void bmp180_write_register(unsigned char reg, unsigned char value);
unsigned char bmp180_read_register(unsigned char reg);
uint16_t bmp180_read_16bit_register(unsigned char reg);

// Function to produce a delay
void delay_ms(unsigned int ms) {
    for (unsigned int i = 0; i < ms; i++) {
        for (unsigned int j = 0; j < 8000; j++) {
            __NOP();
        }
    }
}

// Function to initialize GPIO pins
void GPIO_INIT(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    // Initialize SDA and SCL pins for I2C
    GPIO_InitStructure.GPIO_Pin = SDA_PIN | SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// Function to write a byte of data to the I2C bus
void i2c_write(unsigned char dat) {
    for (unsigned char i = 0; i < 8; i++) {
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
        if (dat & (0x80 >> i)) {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
        } else {
            GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
        }
        GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    }
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to start I2C communication
void i2c_start(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to stop I2C communication
void i2c_stop(void) {
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    delay_ms(1);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
}

// Function to wait for an acknowledgment bit
void i2c_ACK(void) {
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
    GPIO_WriteBit(GPIOC, SDA_PIN, Bit_SET);
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_SET);
    while(GPIO_ReadInputDataBit(GPIOC, SDA_PIN));
    GPIO_WriteBit(GPIOC, SCL_PIN, Bit_RESET);
}

// Function to send a command to the LCD
void lcd_send_cmd(unsigned char cmd) {
    unsigned char cmd_l = (cmd << 4) & 0xf0;
    unsigned char cmd_u = cmd & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(cmd_u | 0x0C);
    i2c_ACK();
    i2c_write(cmd_u | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_write(cmd_l | 0x0C);
    i2c_ACK();
    i2c_write(cmd_l | 0x08);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send data to the LCD
void lcd_send_data(unsigned char data) {
    unsigned char data_l = (data << 4) & 0xf0;
    unsigned char data_u = data & 0xf0;

    i2c_start();
    i2c_write(LCD_Address << 1);
    i2c_ACK();
    i2c_write(data_u | 0x0D);
    i2c_ACK();
    i2c_write(data_u | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_write(data_l | 0x0D);
    i2c_ACK();
    i2c_write(data_l | 0x09);
    i2c_ACK();
    delay_ms(1);
    i2c_stop();
}

// Function to send a string to the LCD
void lcd_send_str(unsigned char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Function to initialize the LCD
void lcd_init(void) {
    lcd_send_cmd(0x02); // Return home
    lcd_send_cmd(0x28); // 4-bit mode, 2 lines, 5x7 dots
    lcd_send_cmd(0x0C); // Display On, cursor off
    lcd_send_cmd(0x06); // Increment cursor (shift cursor to right)
    lcd_send_cmd(0x01); // Clear display
    delay_ms(20);       // Wait for the LCD to process the clear command
}

// Function to write a value to a BMP180 register
void bmp180_write_register(unsigned char reg, unsigned char value) {
    i2c_start();
    i2c_write(BMP180_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_write(value);
    i2c_ACK();
    i2c_stop();
}

// Function to read a value from a BMP180 register
unsigned char bmp180_read_register(unsigned char reg) {
    unsigned char value;
    i2c_start();
    i2c_write(BMP180_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_start();
    i2c_write((BMP180_Address << 1) | 0x01);
    i2c_ACK();
    value = GPIO_ReadInputDataBit(GPIOC, SDA_PIN);
    i2c_stop();
    return value;
}

// Function to read a 16-bit value from a BMP180 register
uint16_t bmp180_read_16bit_register(unsigned char reg) {
    uint16_t value;
    i2c_start();
    i2c_write(BMP180_Address << 1);
    i2c_ACK();
    i2c_write(reg);
    i2c_ACK();
    i2c_start();
    i2c_write((BMP180_Address << 1) | 0x01);
    i2c_ACK();
    value = (bmp180_read_register(reg) << 8) | bmp180_read_register(reg + 1);
    i2c_stop();
    return value;
}
    
// Function to read pressure from BMP180
int32_t bmp180_read_pressure(void) {
    int32_t up;
    bmp180_write_register(0xF4, 0x34 + (3 << 6));
    delay_ms(5);
    up = bmp180_read_16bit_register(0xF6);
    return up;
}

int main(void) {
    GPIO_INIT(); // Initialize GPIO pins
    lcd_init();  // Initialize the LCD Display
    delay_ms(20);
    
    
    
    // Print "Pressure" on the LCD 
    while (1) {
        
        int32_t pressure = bmp180_read_pressure();
        
        lcd_send_cmd(0x80); // Move the cursor to first row, first column
        delay_ms(20);
        lcd_send_str((unsigned char*)"Press: ");
        
       // lcd_send_data((pressure) + '0');
       lcd_send_data((pressure / 100) + '0');
       lcd_send_data(((pressure % 100) / 10) + '0');
       lcd_send_data((pressure % 10) + '0');
        lcd_send_str((unsigned char*)" hPa");
        lcd_send_cmd(0xC0); // Move the cursor to second row, first column
        lcd_send_str((unsigned char*)"Status:");

                // Define pressure thresholds in hPa
        
        float safe_max_PSI = 256.0; 
    
        // Check pressure conditions
        if (pressure > safe_max_PSI) {  
            //lcd_send_cmd(0x80); 
            lcd_send_str((unsigned char*)"Safe");
    
        } else if (pressure < safe_max_PSI) {  
           // lcd_send_cmd(0x80); 
            lcd_send_str((unsigned char*)"Risk");

        
        delay_ms(1000);
        lcd_send_cmd(0x01); // Clear the display
        delay_ms(1000);
    }
}
}
        
```


## Demonstation Video with Arduino IDE

https://drive.google.com/file/d/10zW7UOl_aDTQp0_erywTDyBhxi6431aL/view?usp=sharing

## Demonstation Video with VSDSquadron Mini RISC-V Microcontroller Board

https://drive.google.com/file/d/113H-x_Sz_JgadCqJOI0I7lY1-Dg_z7pm/view?usp=sharing



## Conclusion
<p align="justify">
The system designed and developed as part of this project provides a comprehensive solution for monitoring and maintaining optimal tire pressure which leads to wheel balancing and wheel alignment. This project demonstrates the potential for technology to improve vehicle safety, efficiency, and performance. Its successful implementation can have a significant impact on the automotive industry.
    
    
 
