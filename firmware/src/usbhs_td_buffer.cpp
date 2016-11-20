#include "kinetis.h"

#include "usbhs_td_buffer.hpp"

namespace usbhs {

TransferDescriptorBuffer::TransferDescriptorBuffer(
    EndpointTransferDescriptor* descriptors, int cnt) 
    : freelist_head_(descriptors) {
  for (int i=0; i < cnt - 1; ++i) {
    descriptors[i].next_dtd = &descriptors[i+1];
  }
  descriptors[cnt-1].next_dtd = 0;
}

EndpointTransferDescriptor* TransferDescriptorBuffer::allocate(
    void* buffer, uint32_t length) {
  __disable_irq();
  if (freelist_head_ == 0) {
    __enable_irq();
    return 0;
  }

  EndpointTransferDescriptor* res = freelist_head_;
  freelist_head_ = freelist_head_->next_dtd;
  __enable_irq();

  res->next_dtd = (EndpointTransferDescriptor*)1;
  res->token = (length << 16) | (1 << 15) | (1 << 7);
  res->buffer_ptr = buffer;
  res->length = length;

  uint32_t page = ((uint32_t)res->buffer_ptr) & 0xFFFFF000;
  for (int i = 0; i < 4; ++i) {
    page += 0x1000;
    res->buffer_pointers[i] = page;
  }

  return res;
}

void TransferDescriptorBuffer::release(EndpointTransferDescriptor* desc) {
  // This only happens inside the IRQ handler, so it cannot be preempted
  desc->next_dtd = freelist_head_;
  freelist_head_ = desc;
}

}
