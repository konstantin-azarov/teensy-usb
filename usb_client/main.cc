#include <iostream>

#include <libusb-1.0/libusb.h>
#include <chrono>

#define libUsbSafeCall(x) {  \
  int __call_res = x; \
  if (__call_res < 0) { \
    std::cerr << "Failed call: "#x << " = " \
    << libusb_error_name(__call_res) << std::endl; \
  } \
}

const uint16_t kVendorId = 0xFEED;
const uint16_t kProductId = 0xBEEF;

using namespace std::chrono;

int main() {
  libusb_context* context;

  libUsbSafeCall(libusb_init(&context));

  auto handle = libusb_open_device_with_vid_pid(context, kVendorId, kProductId);

  if (handle == nullptr) {
    std::cerr << "Device not found" << std::endl;
    return 1;
  }

  const uint32_t kBytes = 5*1024*1024;

  bool ok = true;

  for (int t = 0; t < 5; ++t) {
  auto t0 = high_resolution_clock::now();

  libUsbSafeCall(libusb_control_transfer(
        handle,
        (2 << 5), // vendor request
        42,
        0,
        0,
        (uint8_t*)&kBytes,
        sizeof(kBytes),
        1000));

  uint8_t data[kBytes];
  int transferred;

  libUsbSafeCall(libusb_bulk_transfer(
        handle,
        0x81,
        data,
        kBytes,
        &transferred,
        1000));

  auto t1 = high_resolution_clock::now();

  float dt = duration_cast<milliseconds>(t1 - t0).count()/1000.0;

  std::cout << "Transferred: " << transferred << " bytes in " 
    << dt  << " seconds (" << transferred/dt/1024/1204 << "MB/sec)" << std::endl;

  bool ok = transferred == kBytes;
  for (int i=0; i < kBytes && ok; ++i) {
    if (data[i] != (uint8_t)i) {
      std::cout << "Data mismatch:" << i << " != " << (int)data[i] << std::endl;
      ok = false;
    }
  }

  if (ok) {
    std::cout << "OK" << std::endl;
  }

  } 

  libusb_close(handle);

  libusb_exit(context);

  return ok ? 0 : 2;
}
