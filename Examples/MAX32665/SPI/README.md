## Description

This example configures the SPI to send data between the MISO (P0.11) and MOSI (P0.10) pins.  Connect these two pins together.

Multiple word sizes (2 through 16 bits) are demonstrated.

By default, the example performs blocking SPI transactions.  To switch to non-blocking (asynchronous) transactions, undefine the MASTERSYNC macro and define the MASTERASYNC macro.  To use DMA transactions, define the MASTERDMA macro instead.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   You must connect AIN1/P0.17 (MOSI) to AIN2/P0.18 (MOSI).

## Expected Output

The Console UART of the device will output these messages:

```
************** SPI Loopback Demo ****************
This example configures the SPI to send data between the MISO (AIN2/P0.18) and
MOSI (AIN1/P0.17) pins.  Connect these two pins together.  This demo shows SPI
sending different bit sizes each run through. If successful, the green LED will
illuminate.

This demo shows Asynchronous, Synchronous and DMA transaction for SPI1

--> 2 Bits Transaction Successful

--> 3 Bits Transaction Successful

--> 4 Bits Transaction Successful

--> 5 Bits Transaction Successful

--> 6 Bits Transaction Successful

--> 7 Bits Transaction Successful

--> 8 Bits Transaction Successful

--> 9 Bits Transaction Successful

-->10 Bits Transaction Successful

-->11 Bits Transaction Successful

-->12 Bits Transaction Successful

-->13 Bits Transaction Successful

-->14 Bits Transaction Successful

-->15 Bits Transaction Successful

-->16 Bits Transaction Successful
```

The green LED turns on after a successful DMA transaction, but the transaction status won't be printed on the terminal because the SPI and UART pins are shared.
