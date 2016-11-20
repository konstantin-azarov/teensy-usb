#include "Arduino.h"

#include "usbhs.hpp"
#include "debug_print.hpp"

namespace usbhs {

const int kMaxOutstandingTransfers = 32;
const int kNumEndpoints = 8;

const uint32_t kDtdCallbackBit = 1 << 16;

static UsbHs* g_instance;
    
static EndpointQueueHead queue_heads_[kNumEndpoints] 
  __attribute__((aligned(4096)));
static EndpointTransferDescriptor 
      transfer_descriptors_[kMaxOutstandingTransfers];

UsbHs::UsbHs(
    const UsbDeviceDescriptor* device_descriptor,
    const void* config_descriptors, 
    Callbacks* callbacks)
    : td_buffer_(transfer_descriptors_, kMaxOutstandingTransfers),
      device_descriptor_(device_descriptor),
      config_descriptors_(config_descriptors),
      callbacks_(callbacks) {
  g_instance = this;

}

void UsbHs::init() {
  DBG_LOG(USBHS) << "Initializing USBHS" << endl;

  // Enable USBHS access to MPU
  MPU_RGDAAC0 |= 0x30000000;

  // Clocks
  MCG_C1 |= MCG_C1_IRCLKEN;
  OSC0_CR |= OSC_ERCLKEN;

  SIM_SOPT2 |= SIM_SOPT2_USBREGEN;
  SIM_SOPT2 &= ~SIM_SOPT2_USBSLSRC;
  SIM_SCGC3 |= SIM_SCGC3_USBHSPHY | SIM_SCGC3_USBHS | SIM_SCGC3_USBHSDCD;
  USBHSDCD_CLOCK = 33 << 2;
 
  // PHY PLL
  USBPHY_CTRL = 0;
  
  USBPHY_TRIM_OVERRIDE_EN_SET = 1;    // PLL divider override

  USBPHY_PLL_SIC = 
    USBPHY_PLL_SIC_PLL_DIV_SEL(1) | 
    USBPHY_PLL_SIC_PLL_POWER | 
    USBPHY_PLL_SIC_PLL_EN_USB_CLKS | 
    USBPHY_PLL_SIC_PLL_ENABLE;
  
  int cnt = 0;
  while(!(USBPHY_PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK)) {
    cnt++;
  }

  DBG_LOG(USBHS) << "PLL locked: " << cnt << endl;

  // Disable DP/DM pulldown resistors
  USBPHY_ANACTRL &= ~(1<<10);

  // Power up PHY
  USBPHY_PWD = 0;

  // Reset USB controller
  USBHS_USBCMD |= USBHS_USBCMD_RST;
  cnt = 0;
  while (USBHS_USBCMD & USBHS_USBCMD_RST) {
    cnt++;
  }
  
  DBG_LOG(USBHS) << "USB reset: " << cnt << endl;

  // Enable interrupts
  NVIC_SET_PRIORITY(IRQ_USBHS, 113);
  NVIC_ENABLE_IRQ(IRQ_USBHS);

  // Configure default control endpoints
  configureEndpoint_(0, false, 64, EndpointType::CONTROL);
  configureEndpoint_(1, false, 64, EndpointType::CONTROL);

  USBHS_EPLISTADDR = (uint32_t)queue_heads_;
  USBHS_USBINTR = USBHS_USBINTR_URE 
    | USBHS_USBINTR_SEE
    | USBHS_USBINTR_UEE 
    | USBHS_USBINTR_PCE 
    | USBHS_USBINTR_UE;

  USBHS_USBMODE = 
    (USBHS_USBMODE & ~USBHS_USBMODE_CM(3)) | USBHS_USBMODE_CM(2) |
    USBHS_USBMODE_SLOM;

  // Start the controller
  USBHS_USBCMD |= USBHS_USBCMD_RS;
}

bool UsbHs::enqueueTransfer(
    int endpoint_address, 
    const void* buffer, 
    uint32_t length, 
    bool callback) {
  auto dtd = td_buffer_.allocate((uint8_t*)buffer, length);

  if (dtd == nullptr) {
    return false;
  }

  if (callback) {
    dtd->length |= kDtdCallbackBit;
  }

  int endpoint = (endpoint_address & 0x7)*2 + (endpoint_address >> 7);

  auto* qh = &queue_heads_[endpoint];

  __disable_irq();
  bool was_empty = qh->tail_dtd == nullptr;

  if (qh->tail_dtd != nullptr) {
    qh->tail_dtd->next_dtd = dtd;
    qh->tail_dtd = dtd;
  } else {
    qh->head_dtd = qh->tail_dtd = dtd;
  }
  __enable_irq();

  // Execute the transfer descriptor (54.5.3.5.3)
  uint32_t prime_bit = epBit_(endpoint);

  if (!was_empty) {
    // A tranport was already in progress, so maybe we don't have to do
    // anything (case 2 in the reference).
    if (USBHS_EPPRIME & prime_bit) {
      return true;
    } else {
      uint32_t status;
      do {
        USBHS_USBCMD |= USBHS_USBCMD_ATDTW;
        status = USBHS_EPSR & prime_bit;
      } while ((USBHS_USBCMD & USBHS_USBCMD_ATDTW) == 0);
      USBHS_USBCMD &= ~USBHS_USBCMD_ATDTW;
      if (status) {
        return true;
      }
      // USBHS already released the last dtd, so we are back to case 1.
    }
  }

  // Case 1 - the list is empty, need to prime from scratch
  qh->next_dtd = dtd;
  qh->status = 0;
  
  USBHS_EPPRIME |= prime_bit; 
  while (USBHS_EPPRIME & prime_bit);
  if ((USBHS_EPSR & prime_bit) == 0) {
    DBG_LOG(USBHS) << "Error priming" << endl;
    return false;
  }

  return true;
}

uint32_t UsbHs::epBit_(int endpoint) {
  return 1 << (endpoint / 2 + (endpoint % 2) * 16);
}

void UsbHs::isr_() {
  int status = USBHS_USBSTS;
  int setup = USBHS_EPSETUPSR;
  int complete = USBHS_EPCOMPLETE;

  DBG_LOG(USBHS) << "ISR: " << hex(status) << hex(USBHS_PORTSC1) << endl;

  USBHS_USBSTS = status;
  USBHS_EPSETUPSR = setup;
  USBHS_EPCOMPLETE = complete;

  if (status & USBHS_USBSTS_UI) {
    if (setup & 1) {
      onSetup_(0);
    }

    if (complete) {
      for (int i=0; i < 4; ++i) {
        if (complete & (1 << i)) {
          onComplete_(i*2);
        }
        if (complete & (1 << (i + 16))) {
          onComplete_(i*2 + 1);
        }
      }
    }
  }

  
  if (status & USBHS_USBSTS_SEI) {
    DBG_LOG(USBHS) << "System error, reset" << endl;
    USBHS_USBCMD |= USBHS_USBCMD_RST;
  }

  if (status & USBHS_USBSTS_URI) {
    busReset_();
  }

  if (status & USBHS_USBSTS_PCI) {
    usb_speed_ = (USBHS_PORTSC1 >> 26) & 0x3;

    DBG_LOG(USBHS) << "USBHS connected, speed = " << usb_speed_ << endl;
  }
}

void UsbHs::busReset_() {
  DBG_LOG(USBHS) << "USBHS reset" << endl;

  USBHS_EPSETUPSR = USBHS_EPSETUPSR;
  USBHS_EPCOMPLETE = USBHS_EPCOMPLETE;
  int cnt = 0;
  while (USBHS_EPPRIME) { cnt++; }
  USBHS_EPFLUSH = 0x000F000F;

  USBHS_DEVICEADDR = 0;

  if (!(USBHS_PORTSC1 & USBHS_PORTSC_PR)) {
    DBG_LOG(USBHS) << "Reset failed" << endl;
    USBHS_USBCMD |= USBHS_USBCMD_RST;
    return;
  }
}

void UsbHs::configureEndpoint_(
    int endpoint, bool zlc, uint16_t max_packet_length, EndpointType type) {
  if (endpoint <= 1 && type != EndpointType::CONTROL) {
    DBG_LOG(USBHS) << "Invalid endpoint type" << endl;
    return;
  }

  auto* qh = &queue_heads_[endpoint];
  bool ios = type == EndpointType::CONTROL && (endpoint % 2) == 0;

  qh->cap = (zlc << 29) | (max_packet_length << 16) | (ios << 15);
 
  if (endpoint > 1) {
    uint8_t shift = endpoint % 2 ? 16 : 0;
    volatile uint32_t* reg = (volatile uint32_t*)(0x400A11C0 + (endpoint/2)*4);
     *reg = ((*reg) & ~(0xFF << shift)) | 
      (((static_cast<int>(type) << 2) | (1 << 7)) << shift);
  }
}

void UsbHs::resetEndpoint_(int endpoint) {
  auto* qh = &queue_heads_[endpoint];
  if (qh->head_dtd != nullptr) {
    uint8_t bit = epBit_(endpoint);
    USBHS_EPFLUSH |= bit;
    while (USBHS_EPFLUSH & bit);

    while (qh->head_dtd != nullptr) {
      auto tmp = qh->head_dtd->next_dtd;
      td_buffer_.release(qh->head_dtd);
      qh->head_dtd = (EndpointTransferDescriptor*)((uint32_t)tmp & 0xFFFFFFE0);
    }

    qh->tail_dtd = nullptr;

    qh->next_dtd = (EndpointTransferDescriptor*)1;
    qh->status = 0;
  }
}

void UsbHs::onComplete_(int endpoint) {
  auto* qh = &queue_heads_[endpoint];

  while (qh->head_dtd != nullptr && 
      (qh->head_dtd->token & 0x80) == 0) {
    bool callback = qh->head_dtd->length & kDtdCallbackBit;
    uint32_t rem = ((qh->head_dtd->token >> 16) & 0x7FFF) ;
    uint32_t len = (qh->head_dtd->length & ~kDtdCallbackBit) - rem;
    auto* next = qh->head_dtd->next_dtd;

    td_buffer_.release(qh->head_dtd);
    qh->head_dtd = (EndpointTransferDescriptor*)((uint32_t)next & 0xFFFFFFE0);

    if (callback) {
      callbacks_->onEndpointTransferComplete(
          endpoint / 2 + ((endpoint % 2) << 7), len);
    }
  }

  if (qh->head_dtd == nullptr) {
    qh->tail_dtd = nullptr;
  }
}

void UsbHs::onSetup_(int endpoint) {
  uint32_t setup_buf_copy[4];
  auto* qh = &queue_heads_[endpoint];

  USBHS_EPSETUPSR = 1;

  uint32_t* setup_buf = qh->setup_buffer;

  do {
    USBHS_USBCMD |= USBHS_USBCMD_SUTW;
    setup_buf_copy[0] = setup_buf[0]; 
    setup_buf_copy[1] = setup_buf[1];  
    setup_buf_copy[2] = setup_buf[2];  
    setup_buf_copy[3] = setup_buf[3];  
  } while (!(USBHS_USBCMD | USBHS_USBCMD_SUTW));

  USBHS_USBCMD &= ~USBHS_USBCMD_SUTW; 

  while (USBHS_EPSETUPSR & 1);

  resetEndpoint_(endpoint);
  resetEndpoint_(endpoint+1);

  handleSetup_((UsbSetupData*)setup_buf_copy);
}

void UsbHs::handleSetup_(UsbSetupData* data) {
  bool processed = true;
  uint8_t request_type = (data->bmRequestType >> 5) & 3;
  switch (request_type) {
    case USB_REQ_TYPE_STANDARD:
      switch (data->bRequest) {
        case USB_REQ_GET_DESCRIPTOR:
          processed = handleGetDescriptor_(data);
          break;
        case USB_REQ_SET_ADDRESS:
          handleSetAddress_(data->wValue);
          break;
        case USB_REQ_SET_CONFIGURATION:
          processed = handleSetConfiguration_(data->wValue);
          break;
        default:
          processed = false;
          break;
      }
      break;
    case USB_REQ_TYPE_VENDOR:
      if (!callbacks_->onControlSetup(data)) {
        processed = false;
      }
      break;
    default:
      processed = false;
  }

  if (processed) {
    if (data->bmRequestType & 0x80) {
      enqueueTransfer(0x0, nullptr, 0, false); 
    } else {
      enqueueTransfer(0x80, nullptr, 0, false);
    }
  } else {
    DBG_LOG(USBHS) <<  "Unknown request: " << data->bmRequestType << endl;
  }
}

bool UsbHs::handleGetDescriptor_(UsbSetupData* data) {
  uint8_t desc_type = data->wValue >> 8;
  uint8_t desc_index = data->wValue & 0xFF;

  const UsbConfigurationDescriptor* config;

  switch (desc_type) {
    case USB_DESC_DEVICE:
      enqueueTransfer(
          0x80, (uint8_t*)device_descriptor_, sizeof(UsbDeviceDescriptor), false);
      return true;
    case USB_DESC_CONFIGURATION:
      config = configurationDescriptorByIndex_(desc_index);
      if (config != nullptr) {
        enqueueTransfer(
            0x80, config, min(data->wLength, config->wTotalLength), false);
      }
      return true;
    /* case USB_DESC_STRING: */
    /*   Serial.print("String descriptor: "); */
    /*   Serial.println(desc_index); */
    /*   sendStringDescriptor(string_descs[desc_index]); */
    /*   return true; */
    default:
      DBG_LOG(USBHS) <<"Unsupported descriptor request: " << desc_type << endl;
      return false;
  }
}

void UsbHs::handleSetAddress_(uint8_t address) {
  USBHS_DEVICEADDR = (address << 25) + (1 << 24);
}

bool UsbHs::handleSetConfiguration_(uint8_t id) {
  DBG_LOG(USBHS) << "SetConfiguration " << id << endl;

  auto config = configurationDescriptorById_(id);
  if (config == nullptr) {
    return false;
  }

  auto iface = reinterpret_cast<const UsbInterfaceDescriptor*>(config + 1);
  auto ep = reinterpret_cast<const UsbEndpointDescriptor*>(iface + 1);

  for (int i=0; i < iface->bNumEndpoints; ++i, ++ep) {
    configureEndpoint_(
        (ep->bEndpointAddress & 0xF)*2 + (ep->bEndpointAddress >> 7),
        true,
        ep->wMaxPacketSize,
        static_cast<EndpointType>(ep->bmAttributes & 0x3));
  }

  return true;
}

const UsbConfigurationDescriptor* UsbHs::configurationDescriptorByIndex_(
    int idx) {
  if (idx >= device_descriptor_->bNumConfigurations) {
    return nullptr;
  }
  auto config = reinterpret_cast<const UsbConfigurationDescriptor*>(
      config_descriptors_);

  while (idx--) {
    config = reinterpret_cast<const UsbConfigurationDescriptor*>(
        reinterpret_cast<const uint8_t*>(config) + config->wTotalLength);
  }

  return config;
}

const UsbConfigurationDescriptor* UsbHs::configurationDescriptorById_(
    int id) {
  int n = device_descriptor_->bNumConfigurations;

  auto config = reinterpret_cast<const UsbConfigurationDescriptor*>(
      config_descriptors_);

  while (n--) {
    if (config->bConfigurationValue == id) {
      return config;
    }
    config = reinterpret_cast<const UsbConfigurationDescriptor*>(
        reinterpret_cast<const uint8_t*>(config) + config->wTotalLength);
  }

  return nullptr;
}

/* int wcslen(const char16_t* str) { */
/*   int len = 0; */
/*   while (*(str++)) { */
/*     len++; */
/*   } */
/*   return len; */
/* } */

/* void sendStringDescriptor(const char16_t* str) { */
/*   int l = wcslen(str) * sizeof(char16_t); */
/*   string_desc_buffer[0] = (l + 2) | (USB_DESC_STRING << 8); */
/*   char16_t* res = &string_desc_buffer[1]; */
/*   while (*str) { */
/*     *res = *str; */
/*     res++; */
/*     str++; */
/*   } */
/*   usbHsControlReadSendData(string_desc_buffer, l + 2); */
/* } */

}

extern "C" void usbhs_isr(void) {
  usbhs::g_instance->isr_();
}


