#include <iostream>

#include <libusb-1.0/libusb.h>

#define libUsbSafeCall(x) {  \
  int __call_res = x; \
  if (__call_res < 0) { \
    std::cerr << "Failed call: "#x << " = " \
    << libusb_error_name(__call_res) << std::endl; \
  } \
}

const uint16_t kVendorId = 0xFEED;
const uint16_t kProductId = 0xBEEF;

int main() {
  libusb_context* context;

  libUsbSafeCall(libusb_init(&context));

  auto handle = libusb_open_device_with_vid_pid(context, kVendorId, kProductId);

  if (handle == nullptr) {
    std::cerr << "Device not found" << std::endl;
    return 1;
  }

  libUsbSafeCall(libusb_control_transfer(
        handle,
        (2 << 5), // vendor request
        42,
        0,
        0,
        nullptr,
        0,
        1000));

  uint8_t data[256];
  int transferred;

  libUsbSafeCall(libusb_bulk_transfer(
        handle,
        0x81,
        data,
        256,
        &transferred,
        1000));

  std::cout << "Transferred: " << transferred << std::endl;

  bool ok = transferred == 256;
  for (int i=0; i < 256 && ok; ++i) {
    if (data[i] != i) {
      std::cout << "Data mismatch" << std::endl;
      ok = false;
    }
  }

  if (ok) {
    std::cout << "OK" << std::endl;
  }

  libusb_close(handle);

  libusb_exit(context);

  return ok ? 0 : 2;
}
