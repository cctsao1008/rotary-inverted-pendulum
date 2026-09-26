#include <string.h>

#include "tusb.h"

#define USB_VID 0xCAFE
#define USB_PID 0x4010
#define USB_BCD 0x0200

static tusb_desc_device_t const desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x00,
    .bNumConfigurations = 0x01,
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*)&desc_device;
}

// 64-byte vendor-defined input/output report. Input carries binary runtime
// telemetry; output carries versioned host test commands.
static uint8_t const desc_hid_report[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor 0xFF00)
    0x09, 0x01,        // Usage 1
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        // Logical Minimum 0
    0x26, 0xFF, 0x00,  // Logical Maximum 255
    0x75, 0x08,        // Report Size 8
    0x95, 0x40,        // Report Count 64
    0x09, 0x01,
    0x81, 0x02,        // Input (Data,Var,Abs)
    0x95, 0x40,
    0x09, 0x01,
    0x91, 0x02,        // Output (Data,Var,Abs)
    0xC0,
};

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return desc_hid_report;
}

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL,
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT 0x02
#define EPNUM_CDC_IN 0x82
#define EPNUM_HID_IN 0x83
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

static uint8_t const desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 5, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report),
                       EPNUM_HID_IN, CFG_TUD_HID_EP_BUFSIZE, 1),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_configuration;
}

static char const* const string_desc_arr[] = {
    (const char[]){0x09, 0x04},
    "Rotary",
    "Rotary Inverted Pendulum RP2350A",
    "",
    "CDC Console",
    "Runtime Telemetry",
};

static uint16_t desc_str[48];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    if (index >= (sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) return NULL;

    size_t count;
    if (index == 0) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        count = 1;
    } else {
        const char* str = string_desc_arr[index];
        count = strlen(str);
        if (count > 47) count = 47;
        for (size_t i = 0; i < count; ++i) desc_str[1 + i] = (uint8_t)str[i];
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
    return desc_str;
}
