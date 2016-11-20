#include "Arduino.h"

#define DEBUG_PRINT 1

#include "debug_print.hpp"
#include "usbhs.hpp"
#include "usb_types.hpp"

using namespace usbhs;

UsbDeviceDescriptor device_descriptor = {
  sizeof(UsbDeviceDescriptor),
  USB_DESC_DEVICE,
  0x0200,
  0xFF,
  0,
  0,
  64,
  0xFEED,
  0xBEEF,
  0x0001,
  0,
  0,
  0,
  1
};

struct Configuration {
  UsbConfigurationDescriptor config;
  UsbInterfaceDescriptor interface;
  UsbEndpointDescriptor endpoint;
} __attribute__((packed)); 

Configuration config = {
  // configuration
  {
    sizeof(UsbConfigurationDescriptor), // bLength
    USB_DESC_CONFIGURATION, // bDescriptorType
    sizeof(UsbConfigurationDescriptor) + 
      sizeof(UsbInterfaceDescriptor) + sizeof(UsbEndpointDescriptor), // wTotalLength
    1, // bNumInterfaces
    1, // bConfigurationValue
    0, // iConfiguration
    0xC0, // bmAttributes
    100 // bMaxPower
  },
  // interface
  {
    sizeof(UsbInterfaceDescriptor), // bLength
    USB_DESC_INTERFACE, // bDescriptorType
    0, // bInterfaceNumber
    0, // bAlternateSetting
    1, // bNumEndpoints
    0xFF, // bInterfaceClass
    0, // bInterfaceSubclass
    0, // bInterfaceProtocol
    0  // iInterface
  },
  // endpoint
  {
  sizeof(UsbEndpointDescriptor), // bLength
  USB_DESC_ENDPOINT, // bDescriptorType
  0x81, // bEndpointAddress (1-IN)
  0x2, // bmAttributes (Bulk endpoint)
  512, // wMaxPacketSize
  0, // bInterval
  }
};

uint32_t bytes_to_send = 0;

const int kBuffers = 31;
const int kBufferSize = 4096;

uint8_t buffers[kBuffers][kBufferSize];
int current_buffer = 0;
uint8_t current_byte = 0;

extern UsbHs usbhs_driver;

uint32_t fillBuffer(uint8_t* buffer) {
  int i=0;
  for (; i < kBufferSize && bytes_to_send > 0; ++i, --bytes_to_send) {
    buffer[i] = current_byte++;
  }
  return i;
}

void sendBuffer(int buffer_idx) {
  uint8_t* buffer = buffers[buffer_idx];
  uint32_t sz = min(kBufferSize, bytes_to_send);
  if (sz > 0) {
    bytes_to_send -= sz;
    if (!usbhs_driver.enqueueTransfer(0x81, buffer, sz)) {
      DBG_LOG(Main) << "Failed to send" << endl;
    }
  }
}

void onBufferDone() {
  sendBuffer(current_buffer);
  current_buffer = (current_buffer + 1) % kBuffers;
}

void onBytesToSend() {
  DBG_LOG(Main) << "Sending " << bytes_to_send << endl;

  current_byte = 0;
  for (int i=0; i < kBuffers; ++i) {
    for (int j=0; j < kBufferSize; ++j) {
      buffers[i][j] = current_byte++;
    }
  }

  current_buffer = 0;
  for (int i=0; i < kBuffers; ++i) {
    sendBuffer(i);
  }
}

class UsbHsClient : public usbhs::Callbacks {
  virtual bool onControlSetup(const UsbSetupData* setup) {
    if (setup->bRequest == 42) {
      usbhs_driver.enqueueTransfer(0, (uint8_t*)&bytes_to_send, sizeof(bytes_to_send));
      return true;
    }

    return false;
  }

  virtual void onEndpointTransferComplete(int endpoint_address, uint32_t) {
    switch (endpoint_address) {
      case 0:
        onBytesToSend();
        break;
      case 0x81:
        onBufferDone();
        break;
    }
  }
};


UsbHsClient usbhs_client;
UsbHs usbhs_driver(&device_descriptor, &config, &usbhs_client);


uint32_t t0 = millis();

void setup() {
  /* SIM_SOPT1CFG |= SIM_SOPT1CFG_URWE; */
  /* SIM_SOPT1 &= ~SIM_SOPT1_USBREGEN; */

  while (!Serial);

  usbhs_driver.init();
 
  pinMode(13, OUTPUT);
}


void loop() {
  uint32_t t = millis();

  if (t - t0 > 1000) {
    digitalWrite(13, !digitalRead(13));

    t0 = t;
  }
}

