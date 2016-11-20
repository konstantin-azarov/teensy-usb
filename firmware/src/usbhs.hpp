#ifndef __USBHS__HPP__
#define __USBHS__HPP__

#include "usb_types.hpp"
#include "usbhs_structs.hpp"
#include "usbhs_td_buffer.hpp"

namespace usbhs {

/**
 * Contains callback functions called by the UsbHs driver.
 */ 
class Callbacks {
  public:
    /** 
     *  Called when a setup packet arrives (currently on the endpoint 0).
     *  Standard setup packets are handled by the driver, so the 
     *  request_type of this setup packet is guaranteed to be 2 (vendor).
     *
     *  This function is called in the USBHS interrupt context.
     */
    virtual bool onControlSetup(const UsbSetupData* setup) = 0;

    /** 
     *  Called when a transfer on the endpoint with address endpoint_address
     *  is complete. The `bytes_transferred` parameter indicates how many bytes
     *  were actually transfered (useful for host to device variable length
     *  transfers). The `endpoint_address` field has the same format as the
     *  bEndpointAddress field in the endpoint descriptor.
     *
     *  This function is called in the USBHS interrupt context.
     */
    virtual void onEndpointTransferComplete(
        int endpoint_address, uint32_t bytes_transferred);
};

/**
 * This class drives the USBHS port and handles standard USB protocol messages.
 * Only one instance should be created (since there is only one USBHS port on
 * Teensy 3.6).
 */
class UsbHs {
  public:
    /**
     * The user is responsible for constructing proper USB descriptors:
     *   o device_descriptor should point to a valid USB device descriptor 
     *     according to Section 9.6.1
     *   o config_descriptor should point to configuration, interface and
     *     endpoint descriptors in the order suitable for the GET_DESCRIPTOR
     *     operation for configuration descriptors described in section 9.4.3
     */
    UsbHs(
        const UsbDeviceDescriptor* device_descriptor,
        const void* config_descriptors, 
        Callbacks* callbacks);

    /**
     * Initializes the underlying hardware
     */
    void init();
  
    /**
     * Enqueues a transfer on a given endpoint. This function is asynchronous -
     * the actual transfer takes place when host initiates an IN or OUT 
     * transaction on the bus. If the `callback` parameter is set to true,
     * then Callbacks::onEndpointTransferComplete will be called when the
     * transfer actually completes. It is guaranteed that sequential transfers
     * on a given endpoint will finish in the order they were enqueued. This 
     * guarantee does not exist for transfers on different endpoints.
     *
     * The maximum number of outstanding transfers is defined by the 
     * kMaxOutstandingTransfers constant in usbhs.cpp.
     *
     * The `endpoint` parameter should contain the endpoint address (in the
     * format of the bEndpointAddress field from the endpoint descriptor).
     * The `buffer` and `length` parameters specify the data to be send.
     * Maximum length of the data is 4 Kb.
     */ 
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
    TransferDescriptorBuffer td_buffer_;

    const UsbDeviceDescriptor* device_descriptor_;
    const void* config_descriptors_;
    Callbacks* callbacks_;

    uint8_t usb_speed_;

    friend void ::usbhs_isr();

} __attribute__((aligned(4096)));

}

#endif
