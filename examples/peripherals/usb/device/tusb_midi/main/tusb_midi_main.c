/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdlib.h>
#include "esp_log.h"
//#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
//#include "tusb_config.h"
//#include "tusb_cdc_acm.h"
//#include "sdkconfig.h"

static const char *TAG = "example";

/* A combination of interfaces must have a unique product id, since PC will save device driver after the first plug.
 * Same VID/PID with different interface e.g MSC (first), then CDC (later) will possibly cause system error on PC.
 *
 * Auto ProductID layout's Bitmap:
 *   [MSB]       MIDI | HID | MSC | CDC          [LSB]
 */
#define _PID_MAP(itf, n) ( (CFG_TUD_##itf) << (n) )
#define USB_PID (0x4000 | _PID_MAP(CDC, 0) | _PID_MAP(MSC, 1) | _PID_MAP(HID, 2) | \
                 _PID_MAP(MIDI, 3) | _PID_MAP(VENDOR, 4) )

static void midi_task_example(void* arg)
{
    // Variable that holds the current p`osition in the sequence.
    uint32_t note_pos = 0;

    // Store example melody as an array of note values
    uint8_t note_sequence[] =
    {
        74,78,81,86,90,93,98,102,57,61,66,69,73,78,81,85,88,92,97,100,97,92,88,85,81,78,
        74,69,66,62,57,62,66,69,74,78,81,86,90,93,97,102,97,93,90,85,81,78,73,68,64,61,
        56,61,64,68,74,78,81,86,90,93,98,102
    };

    for(;;) {
        vTaskDelay(10);
        static uint32_t start_ms = 0;

        uint8_t const cable_num = 0; // MIDI jack associated with USB endpoint
        uint8_t const channel   = 0; // 0 for channel 1

        // The MIDI interface always creates input and output port/jack descriptors
        // regardless of these being used or not. Therefore incoming traffic should be read
        // (possibly just discarded) to avoid the sender blocking in IO
        uint8_t packet[4];
        while ( tud_midi_available() )
        {
            tud_midi_packet_read(packet);
            ESP_LOGI(TAG, "usb-midi data: %d %d %d %d", packet[0], packet[1], packet[2], packet[3]);
        }

        // send note periodically
        if ((esp_timer_get_time() / 1000) - start_ms < 286)
        {
            continue; // not enough time
        }
        start_ms += 286;

        // Previous positions in the note sequence.
        int previous = note_pos - 1;

        // If we currently are at position 0, set the
        // previous position to the last note in the sequence.
        if (previous < 0) previous = sizeof(note_sequence) - 1;

        // Send Note On for current position at full velocity (127) on channel 1.
        uint8_t note_on[3] = { 0x90 | channel, note_sequence[note_pos], 127 };
        tud_midi_stream_write(cable_num, note_on, 3);

        // Send Note Off for previous note.
        uint8_t note_off[3] = { 0x80 | channel, note_sequence[previous], 0};
        tud_midi_stream_write(cable_num, note_off, 3);

        // Increment position
        note_pos++;

        // If we are at the end of the sequence, start over.
        if (note_pos >= sizeof(note_sequence)) note_pos = 0;
    }
}

#define MIDI_RX_BUFFER_SIZE 128
uint8_t midi_in[MIDI_RX_BUFFER_SIZE];

void tud_mount_cb(void)
{
  ESP_LOGI("tud_mount_cb", "MOUNTED");
}

void tud_midi_rx_cb(uint8_t itf)
{
    ESP_LOGI(TAG, "new midi message");
    uint32_t bytes_read = tud_midi_n_stream_read(itf, 0, midi_in, MIDI_RX_BUFFER_SIZE);
    for(int i=0; i< bytes_read; i++)
    {
        ESP_LOGI(TAG, "byte %d = %d", i, midi_in[i]);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");

    tusb_desc_device_t midi_descriptor = {
        .bLength = sizeof(midi_descriptor),
        .bDescriptorType = TUSB_DESC_DEVICE,
        .bcdUSB = 0x0200,
        .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
        .bDeviceSubClass = 0x00,
        .bDeviceProtocol = 0x00,
        .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

        .idVendor = 0x303A,
        .idProduct = USB_PID,
        .bcdDevice = 0x010,

        .iManufacturer = 0x01,
        .iProduct = 0x02,
        .iSerialNumber = 0x03,

        .bNumConfigurations = 0x01
    };

    tinyusb_config_t tusb_cfg = {
        .descriptor = &midi_descriptor,
        .string_descriptor = NULL,
        .external_phy = false // In the most cases you need to use a `false` value
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    xTaskCreate(midi_task_example, "midi_task_example", 4 * 1024, NULL, 5, NULL);
}
