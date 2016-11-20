#ifndef __USBHS__HPP__
#define __USBHS__HPP__

#include "usb_types.hpp"
#include "usbhs_structs.hpp"
#include "usbhs_td_buffer.hpp"

namespace usbhs {

const int kMaxOutstandingTransfers = 32;
const int kNumEndpoints = 8;

class Callbacks {
  public:
    virtual bool onControlSetup(const UsbSetupData* setup) = 0;

    virtual void onEndpointTransferComplete(
        int endpoint_address, uint32_t bytes_transferred);
};

class UsbHs {
  public:
    UsbHs(
        const UsbDeviceDescriptor* device_descriptor,
        const void* config_descriptors, 
        Callbacks* callbacks);

    void init();
  
    bool enqueueTransfer(
        int endpoint, 
        const void* buffer, 
        uint32_t length, 
        bool callback = true);

  private:
    uint32_t epBit_(int endpoint);

    // Interrupt handler & hardware management

    void isr_();
    
    void busReset_();

    // Endpoint handling
  
    void configureEndpoint_(
        int endpoint, bool zlc, uint16_t max_packet_length, EndpointType type);
    void resetEndpoint_(int endpoint);
    void onComplete_(int endpoint);
    void onSetup_(int endpoint);

    // Standard USB messages

    void handleSetup_(UsbSetupData* data);
    bool handleGetDescriptor_(UsbSetupData* data);
    void handleSetAddress_(uint8_t address);
    bool handleSetConfiguration_(uint8_t index);

    const UsbConfigurationDescriptor* configurationDescriptorByIndex_(int idx);
    const UsbConfigurationDescriptor* configurationDescriptorById_(int id);

  private:
    EndpointQueueHead queue_heads_[kNumEndpoints];
    EndpointTransferDescriptor transfer_descriptors_[kMaxOutstandingTransfers];

    TransferDescriptorBuffer td_buffer_;

    const UsbDeviceDescriptor* device_descriptor_;
    const void* config_descriptors_;
    Callbacks* callbacks_;

    uint8_t usb_speed_;

    friend void ::usbhs_isr();

} __attribute__((aligned(4096)));

}

#endif
