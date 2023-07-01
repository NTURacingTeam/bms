# bms
battery management system

## CAN development
Using [mcp2515 library](https://github.com/autowp/arduino-mcp2515).

There are the same variable names in mcp2515.h(311) and LTC681x.h(111). 

Change the variable name in mcp2515.h.
``` C++=311
enum /*class*/ STATE : uint8_t {
    STATE_RX0IF = (1<<0),
    STATE_RX1IF = (1<<1)
};
```
