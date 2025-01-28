 
Title
“IoT based Real Time Automobile Dashboard using I2C Protocol”

INTRODUCTION:
The IoT-based Real-Time Automobile Dashboard, powered by the innovative VSD Squadron, transforms vehicle monitoring by integrating advanced technology and IoT capabilities. The VSD Squadron acts as the central microcontroller, managing communication between sensors, displays, and the cloud using the efficient I2C protocol. This system provides real-time updates on key metrics like speed, fuel levels, engine temperature, and location. Data is processed locally and transmitted to the cloud, allowing for remote access and monitoring. The dashboard not only displays live data but also sends alerts for anomalies like low fuel or overheating. With GPS integration, drivers can track their vehicle’s location in real-time. The IoT connectivity enhances safety, maintenance, and performance optimization. The VSD Squadron makes driving smarter by providing actionable insights and timely notifications. This dashboard combines cutting-edge technology with convenience, making it the ultimate tool for modern vehicle management. It’s not just a dashboard; it’s an intelligent, connected co-pilot for today’s drivers.
OVERVIEW:
The IoT-based Real-Time Automobile Dashboard powered by the VSD Squadron integrates advanced technology to monitor key vehicle parameters like speed, fuel levels, engine temperature, and location. Using the I2C protocol, the system ensures efficient communication between sensors, displays, and IoT modules. The VSD Squadron processes the data and displays it in real-time on a dashboard, while also sending it to the cloud for remote monitoring. With GPS integration, alerts for anomalies, and cloud connectivity, the system enhances vehicle safety, performance, and management, offering a smarter, more connected driving experience.
 
Block Diagram

Table for Pin Connections

•	VSD Squadron (Microcontroller):
VSD Squadron Pin	I2C Pin	Function
SDA (A4)	SDA	Serial Data Line (Data transfer)
SCL (A5)	SCL	Serial Clock Line (Clock signal)
VCC	VCC	Power supply for I2C device (3.3V or 5V)
GND	GND	Ground (Common reference)

•	MPU6050 Gyroscope:
VSD Squadron Pin	MPU6050 Pin	Function
SDA (A4)	SDA	Serial Data Line (Data transfer)
SCL (A5)	SCL	Serial Clock Line (Clock signal)
VCC	VCC	Power supply (3.3V or 5V)
GND	GND	Ground (Common reference)

•	BMP180:
VSD Squadron   Pin		
	

Pressure Sensor Pin	Function
SDA (A4)		SDA
	Serial Data Line (Data transfer)
SCL (A5)	SCL
	Serial Clock Line (Clock signal)
VCC	VCC	Power supply (typically 3.3V or 5V)
GND	GND	Ground (Common reference)





•	ESP8266 WiFi Module
      
          
ESP8266 Pin	VSD Squadron Pin	Function
VCC	VCC	Power supply (3.3V required for ESP8266)
GND	GND	Ground connection
TX (Transmit)	TX	Data transmission from ESP8266 to VSD
RX (Receive)	RX	Data reception from VSD to ESP8266




Components required with Bill of Materials
             Item	Quantity	
Links to Products

MPU6050 Gyroscope and Accelerometer sensor
	
1	
https://sulkurl.com/kR7

VSDSquadron Mini RISC-V Microcontroller
	
1	
https://sulkurl.com/kR9

BMP180 Digital Barometric Pressure Sensor Module
	
1	
https://sulkurl.com/kSg


Pinout Diagram

 

Working Code







Demo Vedio


Acknowledgements
