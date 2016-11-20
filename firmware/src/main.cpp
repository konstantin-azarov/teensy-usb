#include "Arduino.h"

#define DEBUG_PRINT 1

#include "debug_print.hpp"
#include "usbhs.hpp"
#include "usb_types.hpp"

using namespace usbhs;

UsbDeviceDescriptor device_descriptor = {
  sizeof(UsbDeviceDescriptor), // bLength
  USB_DESC_DEVICE,  // bDescriptorType
  0x0200,  // bcdUSB
  0xFF,  // bDeviceClass
  0, // bDeviceSubclass
  0, // bDeviceProtocol
  64, // bMaxPacketSize0
  0xFEED, // idVendor
  0xBEEF, // idProduct
  0x0001, // bcdDevice
  0,  // iManufacturer
  0,  // iProduct
  0,  // iSerialNumber
  1   // bNumConfigurations
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
      sizeof(UsbInterfaceDescriptor) + 
      sizeof(UsbEndpointDescriptor), // wTotalLength
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


/* More buffers require more max outstanding transfers, which can for now be
 * adjusted in usbhs.cpp */
const int kBuffers = 31;
const int kBufferSize = 4096;


class UsbHsClient : public usbhs::Callbacks {
  public:
    UsbHsClient() : usbhs_driver_(&device_descriptor, &config, this) {
    }

    void setup() {
      usbhs_driver_.init();
    }

    virtual bool onControlSetup(const UsbSetupData* setup) {
      if (setup->bRequest == 42) {
        usbhs_driver_.enqueueTransfer(
            0, (uint8_t*)&bytes_to_send_, sizeof(bytes_to_send_));
        return true;
      }

      return false;
    }

    virtual void onEndpointTransferComplete(int endpoint_address, uint32_t) {
      if (endpoint_address == 0) {
        DBG_LOG(Main) << "Sending " << bytes_to_send_ << endl;

        int current_byte = 0;
        for (int i=0; i < kBuffers; ++i) {
          for (int j=0; j < kBufferSize; ++j) {
            buffers_[i][j] = current_byte++;
          }
        }

        current_buffer_ = 0;
        for (int i=0; i < kBuffers; ++i) {
          sendBuffer_(i);
        }
      } else if (endpoint_address == 0x81) {
        sendBuffer_(current_buffer_);
        current_buffer_ = (current_buffer_ + 1) % kBuffers;
      }
    }

  private:
    void sendBuffer_(int buffer_idx) {
      uint8_t* buffer = buffers_[buffer_idx];
      uint32_t sz = min(kBufferSize, bytes_to_send_);
      if (sz > 0) {
        bytes_to_send_ -= sz;
        if (!usbhs_driver_.enqueueTransfer(0x81, buffer, sz)) {
          DBG_LOG(Main) << "Failed to send" << endl;
        }
      }
    }

  private:
    UsbHs usbhs_driver_;

    int current_buffer_ = 0;
    uint32_t bytes_to_send_ = 0;
    uint8_t buffers_[kBuffers][kBufferSize];
};

UsbHsClient usbhs_client;

uint32_t t0 = millis();

void setup() {
  /* SIM_SOPT1CFG |= SIM_SOPT1CFG_URWE; */
  /* SIM_SOPT1 &= ~SIM_SOPT1_USBREGEN; */

  while (!Serial);

  usbhs_client.setup();
 
  pinMode(13, OUTPUT);
}


void loop() {
  uint32_t t = millis();

  if (t - t0 > 1000) {
    digitalWrite(13, !digitalRead(13));

    t0 = t;
  }
}

