# bms
battery management system

## LTC6811 library
Default board is Arduino Uno. Need to change the code in **LTC681x.h** (114) for our needs.

```
#define CS_PIN 53
```

## CAN development
Using [mcp2515 library](https://github.com/autowp/arduino-mcp2515).

1. They used the same variable name \(**STAT**\) in **mcp2515.h** (311) and **LTC681x.h** (111).

    Change the variable name in mcp2515.h.
```C++
    enum /*class*/ STATE : uint8_t {
        STATE_RX0IF = (1<<0),
        STATE_RX1IF = (1<<1)
    };
```
