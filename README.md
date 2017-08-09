# Controller-C-Code
These files represent a small portion (~20%) of 'C' code for a Solar Thermal controller board.  

These files represent communications with:

    1. Two     - Dallas Semiconductor DS-2482-800 chips   (multi-channel I/O 1-Wire)

    2. Sixteen - Dallas Semiconductor DS18B20 sensors     (temperature sensor 1-Wire)
    
    3. One     - Linear Technology LTC2309                (multi-channel ADC)

These all utilize the I2C protocol for communications.

The sensor input from these chips feed into downstream systems that run pumps and control
system movement.

Finally, the Driver Setup code provides insight into the the Drivers used (Serial, I2C, PWM, Timers)
the Logging and Error checking involved to validate a clean setup.

The overall controller code is fully operational.

http://www.ti.com/lit/ds/symlink/tm4c129xnczad.pdf

https://datasheets.maximintegrated.com/en/ds/DS2482-800.pdf

https://datasheets.maximintegrated.com/en/ds/DS18B20-PAR.pdf

http://cds.linear.com/docs/en/datasheet/2309fd.pdf
