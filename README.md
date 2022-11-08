# AxisController_20220609_Last
6 axis robotic arm controller - last mbed compiler version of software

![image](https://user-images.githubusercontent.com/5246863/200633940-92506a6f-ee66-45a6-847e-1aa8b9bc6fb3.png)

The Axis Class has 3 dependencies (MotCon, LS7366LIB, and PID). The class encapsulates the required functionality of controlling a DC motor with encoder feedback through pin assignments, an SPI bus, and a pointer for the limit switch source.
The LS7366 encoder interface IC off-loads the critical time and counting requirements from the processor using an SPI bus interface for the class. The Axis class then uses a state machine to perform trapezoidal movement profiles with a Ticker class. Parameters can be adjusted through the serial interface using a FT232RL USB to serial interface IC for computer communication.
The MotCon class is a basic class that defines a PWM output pin and a single direction signal intended to control an H-Bridge motor driver IC. I used an MC33926 motor driver for each motor which are rated at 5.0-28V and 5.0 amp peak, with an RDSon max resistance of 225 milli-ohms. This part also has 3.0V to 5V TTL/CMOS inputs logic levels and various protection circuitry on board. I also liked this particular motor driver chip because you can use a PWM frequency of up to 20KHz, getting the frequency out of the audio range.

![image](https://user-images.githubusercontent.com/5246863/200633998-58830dd1-282f-4270-b623-2726cbdc9967.png)

Above is the prototype for the controller. Originally, a PCF8574 I/O expander was used to read the limit switches by the I2C bus. This has now been re-written to use 6 external interrupts directly for the limit/homing switches. Six motor driver breakout boards using the MC33926 motor driver chip were used to drive the motors.
I use the mbed online compiler to generate the .bin file, use bin2hex to convert it and upload the hex file using Flash Magic to the processor with the serial bootloader. I prefer to use the FT232RL usb to serial converter IC for PC comms due to the high level of reliability and USB driver support (typically already built in Windows 7+). I've started putting this on a PCB and hope to finish by the end of the month (Dec 2015).
Well 3 months later, I've completed the first PCB prototype. A few minor errors but it's working!!

![image](https://user-images.githubusercontent.com/5246863/200634262-e737a6b2-e5f7-4295-b7d1-4ec3c3a2d5ac.png)
  Express PCB Artwork
  
![image](https://user-images.githubusercontent.com/5246863/200634317-d9d42a1b-e4b2-47e8-aef1-fd86eb6d9f73.png)
Inner Power Layer Breakup for motor current

![image](https://user-images.githubusercontent.com/5246863/200634383-8622dfd8-76c8-48e0-9489-b2f1677573c5.png)
First Prototype

![image](https://user-images.githubusercontent.com/5246863/200634451-6cd1e550-2bcf-4ff7-b02b-75ef55790651.png)

![image](https://user-images.githubusercontent.com/5246863/200634482-e1790dc1-835f-47e3-99a5-4c0c75fd051b.png)

![image](https://user-images.githubusercontent.com/5246863/200634503-e30f44d9-508a-4c0f-bc61-e137d8165977.png)

![image](https://user-images.githubusercontent.com/5246863/200634534-a72418ab-3384-49bd-ba52-4b5cef88e42a.png)

![image](https://user-images.githubusercontent.com/5246863/200634624-f58ecdb6-e2a7-4f6e-8139-7728b068ce64.png)

![image](https://user-images.githubusercontent.com/5246863/200634667-2b43c32a-928c-44ce-99fa-670e3943ada2.png)

Python script for converting mbed .bin output to intel hex format (no bin2hex 64K limit) https://pypi.python.org/pypi/IntelHex
Example batch script for speeding up conversion process for FlashMagic (http://www.flashmagictool.com/) programming of board

Serial Interface at 115200 (8N1) baud using a virtual COM port (USB to serial converter).  Uses FTDI driver for FT232RL chipset (built into Windows 7+).
Command Set can be obtained by sending a question mark ‘?’.
