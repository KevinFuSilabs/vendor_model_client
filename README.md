Tested SDK: Bluetooth Mesh 1.3.0

Clone: git clone --recursive https://github.com/KevinFuSilabs/vendor_model_client.git
Vendor ID: 0x1111

Models
 - Server ID: 0x1111
 - Client ID: 0x2222
 
States
 - Temprature
 - Unit
 	- 0x1 - Celsius
 	- 0x2 - Fahrenheit

Messages
 - 0x1 - Temperature Get
 - 0x2 - Temperature Status
 - 0x3 - Unit Get
 - 0x4 - Unit Set
 - 0x5 - Unit Set Unacknowledged
 - 0x6 - Unit Status
 - 0x7 - Update Period Get
 - 0x8 - Update Period Set
 - 0x9 - Update Period Set Unacknowledged
 - 0xA - Update Period Status
 
Operations
 - Server
  - Press PB0 to send "Temperature Status" update
  - Press PB1 to send "Unit Status" update
 - Client
  - Press PB0 to send "Temperature Get"
  - Long press PB0 to send "Update Period Set Unacknowledged" with sequent parameters as below
	- 300ms
	- Off
	- 2s
	- Off
	- 10s
	- Off
	- 2min
	- Off
	- 10min
	- Off
  - Short press PB1 to send "Unit Get"
  - Long press PB1 to send "Unit Set(Celsius)" and "Unit Set Unacknowledged(Fahrenheit)" sequently.