#ifndef __USBHS_TD_BUFFER__HPP__
#define __USBHS_TD_BUFFER__HPP__

#include "usbhs_structs.hpp"

namespace usbhs {

class TransferDescriptorBuffer {
  public:
    TransferDescriptorBuffer(
        EndpointTransferDescriptor* descriptors,
        int cnt);

    EndpointTransferDescriptor* allocate(void* buffer, uint32_t length);
    void release(EndpointTransferDescriptor* desc);

  private:
    EndpointTransferDescriptor* freelist_head_;
};

}

#endif
