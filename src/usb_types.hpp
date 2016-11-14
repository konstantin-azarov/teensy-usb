#ifndef __USB_TYPES__HPP__
#define __USB_TYPES__HPP__

enum UsbRequestCode {
  USB_REQ_GET_STATUS = 0,
  USB_REQ_CLEAR_FEATURE = 1,
  USB_REQ_SET_FEATURE = 3,
  USB_REQ_SET_ADDRESS = 5,
  USB_REQ_GET_DESCRIPTOR = 6,
  USB_REQ_SET_DESCRIPTOR  = 7,
  USB_REQ_GET_CONFIGURATION = 8,
  USB_REQ_SET_CONFIGURATION = 9,
  USB_REQ_GET_INTERFACE = 10,
  USB_REQ_SET_INTERFACE = 11,
  USB_REQ_SYNCH_FRAME = 12
};

enum UsbDescriptorType {
  USB_DESC_DEVICE = 1,
  USB_DESC_CONFIGURATION = 2,
  USB_DESC_STRING = 3,
  USB_DESC_INTERFACE = 4,
  USB_DESC_ENDPOINT = 5,
  USB_DESC_DEVICE_QUALIFIER = 6,
  USB_DESC_OTHER_SPEED_CONFIGURATION = 7,
  USB_DESC_INTERFACE_POWER = 8
};

struct UsbSetupData {
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} __attribute__((packed));

struct UsbDeviceDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;
} __attribute__((packed));

struct UsbConfigurationDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t wTotalLength;
  uint8_t bNumInterfaces;
  uint8_t bConfigurationValue;
  uint8_t iConfiguration;
  uint8_t bmAttributes;
  uint8_t bMaxPower;
} __attribute__((packed));

struct UsbInterfaceDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} __attribute__((packed));

struct UsbEndpointDescriptor {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
};

#endif
