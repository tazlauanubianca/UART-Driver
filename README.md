# UART Driver


The project is a kernel module which serves as a driver for the serial port
UART16550. The driver supports the two standard ports: COM1, COM2. 
Additionally, the driver's parameters can also be changed using a ioctl
operation.

### Usage

The driver can be added as a module with the name of uart16550.ko.
