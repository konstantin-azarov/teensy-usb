# teensy-usb
Proof-of-concept code to drive the USBHS port on teensy 3.6. The firmware is a [platformio](http://platformio.org/) project. The main header file for the USB driver is [usbhs.hpp](firmware/src/usbhs.hpp) - see comments there for details. An example that sets up a bulk IN endpoint and sends some dummy data through it is in [main.cpp](firmware/src/main.cpp). A corresponding client is [here](usb_client/main.cc). The client uses usb-1.0.
