# Arduino Library for MCP4725 DAC

The MCP4725 Digital to Analog Converters is one of the most popular 12-bit DACs, with breakout boards made by both Adafruit and Sparkfun. Unfortunately, the Adafruit library and other libraries I could find for this chip were very incomplete, exposing only the most basic features of the device, and were coded rather poorly. But the device has a variety of interesting features that are neither exposed nor supported by the existing libraries. Examples include putting the device into one of its three sleep modes, or writing to it in Fast-Mode. Since I needed to use some of those features for a project, I ended up wiring my own library, which I am now sharing with with world. 

If you would like to see something changed or added to the library, or if you have any questions, you can file an Issue here on GitHub, and I will try to take care of the request. Note that the documentation for the library is in the actual code files.

If you haven't yet read the datasheet for the device, I would highly encourage you do to this first (https://www.sparkfun.com/datasheets/BreakoutBoards/MCP4725.pdf). To fully understand how the library works, you need to first understand  how the I2C protocol works. A good place to start is this tutorial https://learn.sparkfun.com/tutorials/i2c. To understand the protocol fully and especially some of the advanced concepts, I would also recommend reading the official I2C specification which is only 62 pages long http://www.nxp.com/documents/user_manual/UM10204.pdf.