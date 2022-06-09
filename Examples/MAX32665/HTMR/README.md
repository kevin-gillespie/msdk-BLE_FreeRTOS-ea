## Description

This example demonstrates features of HTimer. It sets the short interval alarm to trigger every ~2^22 short interval counts (0.5sec).

-   Pressing PB0 (P1.06) will print the current count to the console
-   Pressing PB1 (P1.07) will set the Long Interval alarm to light LED1 (P1.14) in 10000 counts

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** High Speed Timer Example *****

This example enables the HTMR and sets the short interval
alarm to trigger every ~2^22 short interval counts (0.5sec)
Pressing PB0 (P1.06) will print the current count to the console
Pressing PB1 (P1.07) will set the Long Interval alarm to light LED1 in 10000 counts


Timer started.

Current Count 2.749268
```
