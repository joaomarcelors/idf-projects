# Module mcp23017 example
This example demonstrates how to use an mcp23017 module with an ESP device over an I2C interface.

This is based on the component developed by Espressif:
https://github.com/espressif/esp-iot-solution/tree/8c7e1dfe1fdbc1a83e642921938c22949420373f/components/expander/io_expander/mcp23017.

With the addition of new functions for writing/reading individual GPIO [0-15] of the mcp23017 module.

## Pin assignments
The GPIO pin numbers used to connect an mcp23017 module can be customized. This can be done this way:
1. In the source code: See the initialization of ``I2C_MASTER_SCL_IO`` and ``I2C_MASTER_SDA_IO`` structures in the example code.
The table below shows the default pin assignments.

mcp23017 pin | SP32 pin  
-------------|---------
 SDA         | 21    
 SCL         | 22      