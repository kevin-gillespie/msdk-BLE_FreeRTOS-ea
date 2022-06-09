## Description

The example demonstartes the use of Semaphore. 

A semaphore is shared between task A and task A and task B. Use PB1 to start task A by acquiring the semaphore. Task B cannot start unless A drops the semaphore. Use PB2 to start task B.

## Required Connections

-   Connect a USB cable between the PC and the CN2 (USB/PWR) connector.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages:

```
***** Semaphore Example *****
SW1 (P1.06) = A tries to write
SW2 (P1.07) = B tries to write

Semaphore acquired.
Semaphore locked.
Semaphore free.

Example running.
A acquired semaphore!
A started writing...
var = 100

A stopped writing.
A dropped the semaphore...

```
