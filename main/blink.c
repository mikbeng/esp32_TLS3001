/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/rmt.h"

static const char *TAG = "TLS3001";

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO 18

typedef union {
	struct __attribute__((packed)) {
		uint8_t r, g, b;
	};
	uint32_t num;
} rgbVal;



#define PIXELS (10)	//Number of pixels 

#define TLS3001_COLOR_BITS (13)
#define TLS3001_PIXEL_BITS (39)

#define RMT_RGB_DATA_ITEMS (TLS3001_PIXEL_BITS)	//In RMT items
#define RMT_ITEM_SIZE (4)		//In bytes
#define RMT_RGB_DATA_PACKAGE_LEN (RMT_RGB_DATA_ITEMS*RMT_ITEM_SIZE)	//In bytes

#define RMT_COLORDATA_DURATION (3)	

#define NUM(a) (sizeof(a) / sizeof(*a))
#define RMT_BLOCK_LEN	32
#define RMT_MAX_DELAY 32767
#define blank_187us 187
#define SYNC_DELAY_CONST (29)	//minimum delay of 28.34uS
#define data_one  {{{ 3, 1, 3, 0 }}}
#define data_zero {{{ 3, 0, 3, 1 }}}
#define data_1msblank {{{ 500, 0, 500, 0 }}}
#define data_postsyncdelay {{{ (PIXELS*SYNC_DELAY_CONST), 0, (PIXELS*SYNC_DELAY_CONST), 0 }}}

rmt_item32_t packet_resetdevice[] = {
	data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, 
	data_zero, 
	data_one, 
	data_zero,
	data_zero
};  //Reset device (19 bits, 15 x 0b1, 1 x 0b0, 1 x 0b1 & 2 x 0b0)

rmt_item32_t packet_syncdevice[] = {
	data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one,
	data_zero, data_zero, data_zero,
	data_one,
	data_zero, data_zero, data_zero, data_zero, data_zero, data_zero, data_zero, data_zero, data_zero, data_zero, data_zero
};  //Synchronize device (30 bits, 15 x 0b1, 3 x 0b0, 1 x 0b1 & 11 x 0b0)

rmt_item32_t packet_startdata[] = {
	data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one,
	data_zero, data_zero,
	data_one,
	data_zero
};  //Start of data (19 bits, 15 x 0b1, 2 x 0b0, 1 x 0b1 & 1 x 0b0)

rmt_item32_t packet_delayresetsync[] = {
	data_1msblank
};  //Delay (1mS) between Reset device and Synchronize device

rmt_item32_t packet_delaypostsync[] = {
	{{{ 4, 0, 4, 0 }}}
};   

rmt_item32_t packet_delay_prestart[] = {
	{{{ 75, 0, 75, 0 }}}
};   


typedef struct
{
	unsigned int red : 12;
	unsigned int green : 12;
	unsigned int blue : 12;
}pixel_data_s;

pixel_data_s pixel_data_array[PIXELS];

void int_to_RMT(rmt_item32_t *RMT_pixel_ret, pixel_data_s *intdata_pixel_RGD);
void Fill_RMT_rx_array(rmt_item32_t *rmt_pixel_array_index, pixel_data_s *pixel_data);

static void light_control(void *arg) {
	ESP_LOGI(TAG, "[APP] Init");
	
	// RMT Config
	rmt_config_t config;
	config.rmt_mode = RMT_MODE_TX;
	config.channel = RMT_TX_CHANNEL;
	config.gpio_num = RMT_TX_GPIO;
	config.mem_block_num = 1;
	config.tx_config.loop_en = 0;
	config.tx_config.carrier_en = 0;
	config.tx_config.idle_output_en = 1;
	config.tx_config.idle_level = 0;
	config.clk_div = 80;		//Gives: 1 duration = 1 us

	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));


	ESP_LOGI(TAG, "[APP] Init done");
	
	uint16_t size_start = NUM(packet_startdata);

	rmt_item32_t *RMT_pixel_array_pointer;

	//Allocate RMT TX memory
	rmt_item32_t *RMT_pixel_array_pointer_start = calloc((size_start + (PIXELS*RMT_RGB_DATA_ITEMS)), 4);
	
	RMT_pixel_array_pointer = RMT_pixel_array_pointer_start;
	
	//First, copy a startdata package to allocated RMT tx memory
	memcpy(RMT_pixel_array_pointer, packet_startdata, (size_start*4));
	
	//test colors. Fill 10 PIXELS with some RGB data
	for(uint8_t i = 0 ; i < PIXELS ; i++)
	{
		pixel_data_array[i].red = 2000+i;
		pixel_data_array[i].green = 500+i;
		pixel_data_array[i].blue = 1000+i;
				
		//Increment pointer to RMT pixel array, points to location of next pixel data
		RMT_pixel_array_pointer = RMT_pixel_array_pointer_start + size_start + (i * RMT_RGB_DATA_ITEMS);
		
		Fill_RMT_rx_array(RMT_pixel_array_pointer, &pixel_data_array[i]);
			
	}

	//Write reset, reset delay, sync and sync delay
	rmt_write_items(RMT_TX_CHANNEL, packet_resetdevice, NUM(packet_resetdevice), true);
	rmt_write_items(RMT_TX_CHANNEL, packet_delayresetsync, NUM(packet_delayresetsync), true);
	rmt_write_items(RMT_TX_CHANNEL, packet_syncdevice, NUM(packet_syncdevice), true);
	rmt_write_items(RMT_TX_CHANNEL, packet_delaypostsync, NUM(packet_delaypostsync), true);		//Not working correctly??
	
	//rmt_write_items(RMT_TX_CHANNEL, packet_delaypoststart, NUM(packet_delaypoststart), true); 	//125us delay after last data bit
	
	while (1) {
		ESP_LOGI(TAG, "[APP] Send packet");
		rmt_write_items(RMT_TX_CHANNEL, RMT_pixel_array_pointer_start, ((RMT_RGB_DATA_ITEMS*PIXELS) + size_start), true);       	//test color, including start frame

		/*Left to-do:
			-Implement correct delay of 125uS between last data bit and new start
			-Implement a correct delay after Sync: 28.34uS x number of pixels. See packet_delaypostsync
			-Continously send sync?
			
		*/	
		
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

void app_main()
{
	ESP_LOGI(TAG, "[APP] Startup");
	xTaskCreate(light_control, "light_control", 4096, NULL, 5, NULL);
}

void int_to_RMT(rmt_item32_t *RMT_pixel_ret, pixel_data_s *intdata_pixel_RGD)
{
	uint16_t k, i;
	int16_t c;
	rmt_item32_t RMT_pixel_local[RMT_RGB_DATA_ITEMS];
	
	uint16_t item_index;
	unsigned int colors[3];
	
	colors[0] = intdata_pixel_RGD->red;
	colors[1] = intdata_pixel_RGD->green;
	colors[2] = intdata_pixel_RGD->blue;
	

	item_index = 0;
	
	for (i = 0; i < 3; i++)	//RGB loop
		{
		
			RMT_pixel_local[i*TLS3001_COLOR_BITS].duration0 = RMT_COLORDATA_DURATION;
			RMT_pixel_local[i*TLS3001_COLOR_BITS].duration1 = RMT_COLORDATA_DURATION;
			// DATA 0 in first bit in each color
			RMT_pixel_local[i*TLS3001_COLOR_BITS].level0 = 0;
			RMT_pixel_local[i*TLS3001_COLOR_BITS].level1 = 1;
			item_index++;
		
			for (c = 11; c >= 0; c--)
			{
				k = (colors[i] >> c) & 0b1;  
			
				RMT_pixel_local[item_index].duration0 = RMT_COLORDATA_DURATION;
				RMT_pixel_local[item_index].duration1 = RMT_COLORDATA_DURATION;
			
				if (k == 1)
				{
					// DATA 1
					RMT_pixel_local[item_index].level0 = 1;
					RMT_pixel_local[item_index].level1 = 0;
				}

				else
				{
					// DATA 0
					RMT_pixel_local[item_index].level0 = 0;
					RMT_pixel_local[item_index].level1 = 1;
				}
				item_index++;
			
			}
		
		}
	
	memcpy(RMT_pixel_ret, &RMT_pixel_local, RMT_RGB_DATA_PACKAGE_LEN);
	
}

void Fill_RMT_rx_array(rmt_item32_t *rmt_pixel_array_index, pixel_data_s *pixel_data)
{
	int_to_RMT(rmt_pixel_array_index, pixel_data);
	
	//Do some other stuff here in the future? 
}
