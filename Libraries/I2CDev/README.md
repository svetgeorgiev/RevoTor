# I2CMaster_4_I2CDev
The I2Cdev.h for I2CMaster library from DSSCircuits

This repository contains a working version of I2CDev library that fully supports the I2CMaster library, because the original I2CDev library does not really work with I2CMaster, even though the define `I2CDEV_I2CMASTER_LIBRARY` is present in the header files.

I created this version because I needed to make the [MPU6050 DMP example](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050) work with a [PJON](https://github.com/gioblu/PJON/) bus.

For more information, have a look at the respective projects:

a) I2CDev: https://github.com/jrowberg/i2cdevlib/

b) I2CMaster: http://dsscircuits.com/articles/86-articles/66-arduino-i2c-master-library

c) PJON: https://github.com/gioblu/PJON/