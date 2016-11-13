#include "Arduino.h"

#include "usb_types.hpp"

struct EndpointTransferDescriptor;

struct TransferRecord {
  uint32_t length;
  void (*completion_callback)(uint32_t length);
};

struct EndpointTransferDescriptor {
  EndpointTransferDescriptor* next_dtd;
  uint32_t token;
  void* buffer_ptr;
  uint32_t buffer_pointers[4];
  TransferRecord* record;  
} __attribute__ ((aligned(32)));

struct EndpointQueueHead {
  uint32_t cap;
  EndpointTransferDescriptor* current_dtd;
  EndpointTransferDescriptor* next_dtd;
  uint32_t status;
  uint32_t overlay[6];
  uint32_t setup_buffer[4];
} __attribute__ ((aligned(64)));

template <uint32_t ND>
class TransferDescriptorBuffer {
  public:
    TransferDescriptorBuffer() : freelist_head_(transfer_descriptors_) {
      for (int i=0; i < ND; ++i) {
        transfer_descriptors_[i].record = &transfer_records_[i];
      }
      for (int i=0; i < ND-1; ++i) {
        transfer_descriptors_[i].next_dtd = &transfer_descriptors_[i+1];
      }
      transfer_descriptors_[ND-1].next_dtd = 0;
    }

    EndpointTransferDescriptor* allocateTransferDescriptor(
        void* buffer, uint32_t length, void (*completion_callback)(uint32_t)) {
      if (freelist_head_ == 0) {
        return 0;
      }

      EndpointTransferDescriptor* res = freelist_head_;
      freelist_head_ = freelist_head_->next_dtd;

      res->next_dtd = (EndpointTransferDescriptor*)1;
      res->token = (length << 16) | (1 << 15) | (1 << 7);

      res->buffer_ptr = buffer;
      res->record->length = length;
      res->record->completion_callback = completion_callback;

      uint32_t page = ((uint32_t)res->buffer_ptr) & 0xFFFFF000;
      for (int i = 0; i < 4; ++i) {
        page += 0x1000;
        res->buffer_pointers[i] = page;
      }

      return res;
    }

    void releaseTransferDescriptor(EndpointTransferDescriptor* desc) {
      desc->next_dtd = freelist_head_;
      freelist_head_ = desc;
    }

  private:
    EndpointTransferDescriptor transfer_descriptors_[ND];
    TransferRecord transfer_records_[ND];

    EndpointTransferDescriptor* freelist_head_;
} __attribute__((aligned(4096)));

TransferDescriptorBuffer<10> descriptor_buffer;

class Endpoint {
  public:
    Endpoint(uint8_t id, EndpointQueueHead* head) : id_(id), head_(head) {
    }

    void setup(bool zlc, uint16_t max_packet_length, bool ios) {
      head_->cap = (zlc << 29) | (max_packet_length << 16) | (ios << 15);
      reset();
    }

    void reset() {
      if (head_dtd_ != nullptr) {
        uint8_t bit = primeBit_();
        USBHS_EPFLUSH |= bit;
        while (USBHS_EPFLUSH & bit);

        while (head_dtd_ != nullptr) {
          auto tmp = head_dtd_->next_dtd;
          descriptor_buffer.releaseTransferDescriptor(head_dtd_);
          head_dtd_ = (EndpointTransferDescriptor*)((uint32_t)tmp & 0xFFFFFFE0);
        }

        tail_dtd_ = nullptr;

        head_->next_dtd = (EndpointTransferDescriptor*)1;
        head_->status = 0;
      }
    }

    void onComplete() {
      while (head_dtd_ != 0 && 
          (head_dtd_->token & 0x80) == 0) {
        uint32_t rem = ((head_dtd_->token >> 16) & 0x7FFF) ;
        uint32_t len = head_dtd_->record->length - rem;
        auto func = head_dtd_->record->completion_callback;

        auto* tmp = head_dtd_->next_dtd;
        descriptor_buffer.releaseTransferDescriptor(head_dtd_);
        head_dtd_ = (EndpointTransferDescriptor*)((uint32_t)tmp & 0xFFFFFFE0);

        func(len);
      }
      
      if (head_dtd_ == nullptr) {
        tail_dtd_ = nullptr;
      }
    }

    void readSetupBuffer(uint32_t* buffer) {
      USBHS_EPSETUPSR = 1;
    
      uint32_t* setup_buf = head_->setup_buffer;

      do {
        USBHS_USBCMD |= USBHS_USBCMD_SUTW;
        buffer[0] = setup_buf[0]; 
        buffer[1] = setup_buf[1];  
        buffer[2] = setup_buf[2];  
        buffer[3] = setup_buf[3];  
      } while (!(USBHS_USBCMD | USBHS_USBCMD_SUTW));

      USBHS_USBCMD &= ~USBHS_USBCMD_SUTW; 

      while (USBHS_EPSETUPSR & 1);
    }

    bool enqueueTransfer(
        void* buffer, uint32_t length, void (*completion_callback)(uint32_t)) {

      auto dtd = descriptor_buffer.allocateTransferDescriptor(
        buffer, length, completion_callback);

      if (tail_dtd_ != nullptr) {
        tail_dtd_->next_dtd = dtd;
        tail_dtd_ = dtd;
      } else {
        head_dtd_ = tail_dtd_ = dtd;
      }

      head_->next_dtd = dtd;
      head_->status = 0;

      int prime_bit = primeBit_();
      Serial.println("Priming ");
      Serial.println(prime_bit, HEX);
      Serial.println(dtd->token, HEX);
      Serial.println((uint32_t)dtd->buffer_ptr, HEX);
      Serial.println((uint32_t)dtd->buffer_pointers[0], HEX);
      Serial.println((uint32_t)dtd->buffer_pointers[1], HEX);
      Serial.println((uint32_t)dtd->buffer_pointers[2], HEX);
      Serial.println((uint32_t)dtd->buffer_pointers[3], HEX);
      Serial.println((uint32_t)head_->next_dtd, HEX);
      Serial.println(USBHS_EPPRIME & prime_bit, HEX);
      USBHS_EPPRIME |= prime_bit; 
      while (USBHS_EPPRIME & prime_bit);
      if ((USBHS_EPSR & prime_bit) == 0) {
        Serial.println("Error priming");
        return false;
      }
      Serial.println("Priming successful");
      /* Serial.println((uint32_t)qh->current_dtd, HEX); */
      /* Serial.println((uint32_t)qh->next_dtd, HEX); */
      /* Serial.println((uint32_t)qh->status, HEX); */

      return true;
    }

  private:
    uint8_t primeBit_() {
      return 1 << (id_/2 + (id_%2) * 16);
    }

  private:
    uint8_t id_;
    EndpointQueueHead* head_;
    EndpointTransferDescriptor *head_dtd_, *tail_dtd_;
};

void usbHsHandleControlSetup(UsbSetupData* setup);

class ControlPipe {
  public: 
    ControlPipe(Endpoint* endpoints) : eps_(endpoints) {
    }

    Endpoint* rx() { 
      return &eps_[0]; 
    }

    Endpoint* tx() {
      return &eps_[1];
    }

    void setup() {
      rx()->setup(false, 64, true);
      tx()->setup(false, 64, false);
    }

    void onSetup() {
      Serial.println("ControlPipe::onSetup");

      rx()->reset();
      tx()->reset();

      uint32_t dst[4];
      rx()->readSetupBuffer(dst);

      usbHsHandleControlSetup((UsbSetupData*)dst);
    }

  private:
    Endpoint* eps_;
};

enum UsbHsState {
  USB_ERROR,
  USB_POWERED,
  USB_ATTACH,
  USB_DEFAULT,
  USB_ADDRESS,
  USB_CONFIGURED
};

EndpointQueueHead endpoint_queue_heads[8] __attribute__((aligned(4096)));
Endpoint endpoints[8] = {
  Endpoint(0, &endpoint_queue_heads[0]),
  Endpoint(1, &endpoint_queue_heads[1]),
  Endpoint(2, &endpoint_queue_heads[2]),
  Endpoint(3, &endpoint_queue_heads[3]),
  Endpoint(4, &endpoint_queue_heads[4]),
  Endpoint(5, &endpoint_queue_heads[5]),
  Endpoint(6, &endpoint_queue_heads[6]),
  Endpoint(7, &endpoint_queue_heads[7]),
};
ControlPipe default_control_pipe(endpoints);

UsbHsState usb_state = USB_POWERED;
int8_t usb_speed = -1;

void printBin(uint32_t v){
  char buf[32 + 3 + 4 + 1];
  buf[39] = 0;

  for (int i = 0, j = 38; i < 32; ++i) {
    if (i) {
      if ((i % 8) == 0) {
        buf[j--] = '"';
      } else if ((i % 4) == 0) {
        buf[j--] = '\'';
      }
    }

    buf[j--] = (v&1) + '0';
    v>>=1;
  }

  Serial.println(buf);
}



void usbHsEnable() {
  Serial.println("Initializing USBHS");

  // Enable USBHS access to MPU
  MPU_RGDAAC0 |= 0x30000000;

  // Clocks
  MCG_C1 |= MCG_C1_IRCLKEN;
  OSC0_CR |= OSC_ERCLKEN;

  SIM_SOPT2 |= SIM_SOPT2_USBREGEN;
  SIM_SOPT2 &= ~SIM_SOPT2_USBSLSRC;

//  PORTC_PCR3 = PORT_PCR_MUX(5); 
//  SIM_SOPT2 = (SIM_SOPT2 & ~SIM_SOPT2_CLKOUTSEL(7)) | SIM_SOPT2_CLKOUTSEL(6);

  SIM_SCGC3 |= SIM_SCGC3_USBHSPHY | SIM_SCGC3_USBHS | SIM_SCGC3_USBHSDCD;
  USBHSDCD_CLOCK = 33 << 2;
 
  USBPHY_CTRL = 0;
  
  USBPHY_TRIM_OVERRIDE_EN_SET = 1;

  USBPHY_PLL_SIC = 
    USBPHY_PLL_SIC_PLL_DIV_SEL(1) | 
    USBPHY_PLL_SIC_PLL_POWER | 
    USBPHY_PLL_SIC_PLL_EN_USB_CLKS | 
    USBPHY_PLL_SIC_PLL_ENABLE;
  
  int cnt = 0;
  while(!(USBPHY_PLL_SIC & USBPHY_PLL_SIC_PLL_LOCK)) {
    cnt++;
  }

  Serial.print("PLL locked: ");
  Serial.println(cnt);


  // Verify connected
  // Disable DP/DM pulldown resistors
  USBPHY_ANACTRL &= ~(1<<10);

  delay(10);

  /* USBPHY_CTRL_SET = USBPHY_CTRL_ENDEVPLUGINDET; */

  /* delay(10); */

  /* while (!(USBPHY_STATUS & USBPHY_STATUS_DEVPLUGIN_STATUS)) { */
  /* } */

  /* Serial.println("Cable connected"); */

  /* // Disable pullup resistors */
  /* USBPHY_CTRL_CLR = USBPHY_CTRL_ENDEVPLUGINDET; */
  
  USBPHY_PWD = 0;

  // Reset USB controller
  USBHS_USBCMD |= USBHS_USBCMD_RST;
  cnt = 0;
  while (USBHS_USBCMD & USBHS_USBCMD_RST) {
    cnt++;
  }
  
  Serial.print("USB reset: ");
  Serial.println(cnt);

  // Enable interrupts
  NVIC_SET_PRIORITY(IRQ_USBHS, 113);
  NVIC_ENABLE_IRQ(IRQ_USBHS);

  usb_state = USB_ATTACH;

  default_control_pipe.setup();

  USBHS_EPLISTADDR = (uint32_t)endpoint_queue_heads;
  USBHS_USBINTR = USBHS_USBINTR_URE 
    | USBHS_USBINTR_SEE
    | USBHS_USBINTR_UEE 
    | USBHS_USBINTR_PCE 
    | USBHS_USBINTR_UE;

//  USBHS_OTGSC |= USBHS_OTGSC_OT;
  USBHS_USBMODE = 
    (USBHS_USBMODE & ~USBHS_USBMODE_CM(3)) | USBHS_USBMODE_CM(2) |
    USBHS_USBMODE_SLOM;

  USBHS_USBCMD |= USBHS_USBCMD_RS;
}

void usbHsReset() {
  Serial.println("USBHS reset");

  USBHS_EPSETUPSR = USBHS_EPSETUPSR;
  USBHS_EPCOMPLETE = USBHS_EPCOMPLETE;
  while (USBHS_EPPRIME) {}
  USBHS_EPFLUSH = 0x000F000F;

  if (!(USBHS_PORTSC1 & USBHS_PORTSC_PR)) {
    Serial.println("Reset failed");
    usb_state = USB_ERROR;
    USBHS_USBCMD |= USBHS_USBCMD_RST;
    return;
  }
}

void usbHsControlReadStatus(uint32_t) {
  default_control_pipe.rx()->enqueueTransfer(nullptr, 0, nullptr);
}

void usbHsControlWriteStatus(uint32_t) {
  default_control_pipe.tx()->enqueueTransfer(nullptr, 0, nullptr);
}

void usbHsControlReadSendData(void* data, uint32_t length) {
  default_control_pipe.tx()->enqueueTransfer(
      data, length, usbHsControlReadStatus);
}

UsbDeviceDescriptor device_descriptor = {
  sizeof(UsbDeviceDescriptor),
  USB_DESC_DEVICE,
  0x0200,
  0xFF,
  0,
  0,
  64,
  0xFEED,
  0xBEEF,
  0x0001,
  0,
  1,
  2,
  1
};

void usbHsSendDeviceDescriptor() {
  usbHsControlReadSendData(&device_descriptor, sizeof(UsbDeviceDescriptor));
}

EndpointTransferDescriptor* usbHsHandleGetDescriptor(UsbSetupData* setup) {
  uint8_t desc_type = setup->wValue >> 8;
//  uint8_t desc_index = setup->wValue & 0xFF;

  switch (desc_type) {
    case USB_DESC_DEVICE:
      usbHsSendDeviceDescriptor();
      break;
    default:
      Serial.print("Unsupported descriptor request: ");
      Serial.println(desc_type);
      break;
  }

  return nullptr;
}

void usbHsHandleSetAddress(uint8_t address) {
  usbHsControlWriteStatus(0);
}

void usbHsHandleControlSetup(UsbSetupData* setup) {
  switch (setup->bRequest) {
    case USB_REQ_GET_DESCRIPTOR:
      usbHsHandleGetDescriptor(setup);
      break;
    case USB_REQ_SET_ADDRESS:
      usbHsHandleSetAddress(setup->wValue);
      break;
    default:
      Serial.print("Unsupported request: ");
      Serial.println(setup->bRequest);
      break;
  }
}

int usb_intr = 0;

extern "C" void usbhs_isr(void) {
  usb_intr = 1;

  int status = USBHS_USBSTS;
  USBHS_USBSTS = status;
  int setup = USBHS_EPSETUPSR;
  USBHS_EPSETUPSR = setup;
  int complete = USBHS_EPCOMPLETE;
  USBHS_EPCOMPLETE = complete;

  Serial.print("USB interrupt: ");
  printBin(status);
  /* Serial.print("EPSETUPSR: "); */
  /* printBin(USBHS_EPSETUPSR); */

  if (status & USBHS_USBSTS_UI) {
    /* Serial.print("EPSETUPSR: "); */
    /* printBin(setup); */
    /* Serial.print("EPCOMPLETE: "); */
    /* printBin(complete); */

    if (setup & 1) {
      default_control_pipe.onSetup();
    }

    if (complete) {
      for (int i=0; i < 4; ++i) {
        if (complete & (1 << i)) {
          endpoints[i*2].onComplete();
        }
        if (complete & (1 << (i + 16))) {
          endpoints[i*2 + 1].onComplete();
        }
      }
    }
  }

  
  if (status & USBHS_USBSTS_SEI) {
    Serial.println("System error, reset");
    USBHS_USBCMD |= USBHS_USBCMD_RST;
  }

  if (status & USBHS_USBSTS_URI) {
    usbHsReset();
  }

  if (status & USBHS_USBSTS_PCI) {
    usb_speed = (USBHS_PORTSC1 >> 26) & 0x3;
    usb_state = USB_DEFAULT;

    Serial.print("USBHS connected, speed = ");
    Serial.println(usb_speed);
  }


}


void setup() {
  /* SIM_SOPT1CFG |= SIM_SOPT1CFG_URWE; */
  /* SIM_SOPT1 &= ~SIM_SOPT1_USBREGEN; */

  while (!Serial);

  usbHsEnable();
 
  pinMode(13, OUTPUT);
}

void loop() {
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(800);

  Serial.println("EP cap:");
  Serial.println(endpoint_queue_heads[0].cap, HEX);

  /* Serial.print("HEAD status: "); */
  /* Serial.println(endpoints.headsPtr()[1].status, HEX); */
  /* Serial.println((uint32_t)endpoints.headsPtr()[1].current_dtd, HEX); */
  /* Serial.println((uint32_t)endpoints.headsPtr()[1].next_dtd, HEX); */
  /* Serial.print("EPSR: "); */
  /* Serial.println(USBHS_EPSR, HEX); */
  /* Serial.print("EPCOMPLETE: "); */
  /* Serial.println(USBHS_EPCOMPLETE, HEX); */
}

