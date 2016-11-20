#ifndef __USBHS_STRUCTS__HPP__
#define __USBHS_STRUCTS__HPP__

#include <stdint.h>

namespace usbhs {

enum class EndpointType {
  CONTROL = 0,
  ISOCHRONOUS = 1,
  BULK = 2,
  INTERRUPT = 3
};

struct EndpointTransferDescriptor {
  EndpointTransferDescriptor* next_dtd;
  volatile uint32_t token;
  void* buffer_ptr;
  uint32_t buffer_pointers[4];
  uint32_t length;
} __attribute__ ((aligned(32)));

struct EndpointQueueHead {
  uint32_t cap;
  volatile EndpointTransferDescriptor* current_dtd;
  EndpointTransferDescriptor* next_dtd;
  volatile uint32_t status;
  uint32_t overlay[6];
  uint32_t setup_buffer[4];
  EndpointTransferDescriptor *head_dtd, *tail_dtd;
} __attribute__ ((aligned(64)));

}

#endif
