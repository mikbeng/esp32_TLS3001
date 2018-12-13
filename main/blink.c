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


#define PIXELS (126)	//Number of pixels 

//#define PIXELS (10)	//Number of pixels 

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
#define SYNC_DELAY_CONST (28.34)	//minimum delay of 28.34uS
//#define SYNC_DELAY_CONST (1)	//minimum delay of 28.34uS
#define data_one  {{{ 3, 1, 3, 0 }}}
#define data_zero {{{ 3, 0, 3, 1 }}}
#define data_1msblank {{{ 500, 0, 500, 0 }}}

#define postsyncduration ((PIXELS*SYNC_DELAY_CONST)/2)
#define extra_delay 86	//Due to some overhead

//#define data_postsyncdelay {{{ ((PIXELS*SYNC_DELAY_CONST)/2), 0, ((PIXELS*SYNC_DELAY_CONST)/2), 0 }}}

rmt_item32_t packet_resetdevice[] = {
	data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, data_one, 
	data_zero, 
	data_one, 
	data_zero,
	data_zero
};  //Reset device (19 bits, 15 x 0b1, 1 x 0b0, 1 x 0b1 & 2 x 0b0)

rmt_item32_t packet_syncdata[] = {
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

/*
rmt_item32_t packet_delaypostsync[] = {
	{{{ 4, 0, 4, 0 }}}
};   
*/

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
uint16_t size_start = NUM(packet_startdata);
uint16_t size_sync = NUM(packet_syncdata);

void int_to_RMT(rmt_item32_t *pixel_x_index, pixel_data_s *intdata_pixel_RGD);
void Fill_rmt_tx_pixel_array(rmt_item32_t *rmt_pixel_array_start_index, uint16_t pixel_index, pixel_data_s *pixel_data);
void Fill_RMT_syncdelay_array(rmt_item32_t * RMT_syncdelay_packet_pointer);
void RMT_init(rmt_channel_t channel, gpio_num_t gpio);

static void light_control(void *arg) {
	
	ESP_LOGI(TAG, "[APP] Init");
	
	//Init channel 0
	RMT_init(0, RMT_TX_GPIO);	
	
	//Allocate sync-delay packet memory
	rmt_item32_t *RMT_syncdelay_packet_pointer = calloc((size_sync + 1), sizeof(rmt_item32_t));
	
	//Generate syncdelay RMT array
	Fill_RMT_syncdelay_array(RMT_syncdelay_packet_pointer);
	
	
	//Allocate RMT TX memory
	rmt_item32_t *RMT_RGB_array_pointer_start = calloc((size_start + (PIXELS*RMT_RGB_DATA_ITEMS)), sizeof(rmt_item32_t));
		
	//First, copy a startdata package to allocated RMT tx memory
	memcpy(RMT_RGB_array_pointer_start, packet_startdata, (size_start*sizeof(rmt_item32_t)));
	
	rmt_item32_t *RMT_pixel_array_pointer = RMT_RGB_array_pointer_start + size_start;		//Points to pixel 0 (after start package)
	
	//test colors. Fill 10 PIXELS with some RGB data
	for(uint8_t pixel_ind = 0 ; pixel_ind < PIXELS ; pixel_ind++)
	{
		pixel_data_array[pixel_ind].red = 4000;// + pixel_ind;
		pixel_data_array[pixel_ind].green = 0;//500 + pixel_ind;
		pixel_data_array[pixel_ind].blue = 0;//200 + pixel_ind + 100;
		
		//Fill rmt tx array with given pixel index and RGB data.
		Fill_rmt_tx_pixel_array(RMT_pixel_array_pointer, pixel_ind, &pixel_data_array[pixel_ind]);
			
	}
	//Write reset, reset delay, sync and sync delay
	rmt_write_items(RMT_TX_CHANNEL, packet_resetdevice, NUM(packet_resetdevice), true);
	rmt_write_items(RMT_TX_CHANNEL, packet_delayresetsync, NUM(packet_delayresetsync), true);
	rmt_write_items(RMT_TX_CHANNEL, RMT_syncdelay_packet_pointer, (size_sync+1), true);
	rmt_write_items(RMT_TX_CHANNEL, RMT_RGB_array_pointer_start, ((RMT_RGB_DATA_ITEMS*PIXELS) + size_start), true);         	//test color, including start frame

	//rmt_write_items(RMT_TX_CHANNEL, packet_delaypostsync, NUM(packet_delaypostsync), true);		//Not working correctly??
	
	//rmt_write_items(RMT_TX_CHANNEL, packet_delaypoststart, NUM(packet_delaypoststart), true); 	//125us delay after last data bit
	vTaskDelay(10 / portTICK_PERIOD_MS);
	while (1) {
		
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		rmt_write_items(RMT_TX_CHANNEL, RMT_syncdelay_packet_pointer, (size_sync + 1), true);
		rmt_write_items(RMT_TX_CHANNEL, RMT_RGB_array_pointer_start, ((RMT_RGB_DATA_ITEMS*PIXELS) + size_start), true);          	//test color, including start frame
		

		/*Left to-do:
			-Implement correct delay of 125uS between last data bit and new start
			-Implement a correct delay after Sync: 28.34uS x number of pixels. See packet_delaypostsync
			-Continously send sync?
			
		*/	
		
	}
}

void app_main()
{
	ESP_LOGI(TAG, "[APP] Startup");
	xTaskCreate(light_control, "light_control", 4096, NULL, 5, NULL);
}

void RMT_init(rmt_channel_t channel, gpio_num_t gpio)
{
	// RMT Config
	rmt_config_t config;
	config.rmt_mode = RMT_MODE_TX;
	config.channel = channel;
	config.gpio_num = gpio;
	config.mem_block_num = 1;
	config.tx_config.loop_en = 0;
	config.tx_config.carrier_en = 0;
	config.tx_config.idle_output_en = 1;
	config.tx_config.idle_level = 0;
	config.clk_div = 80;		//Gives: 1 duration = 1 us

	ESP_ERROR_CHECK(rmt_config(&config));
	ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

	ESP_LOGI(TAG, "[APP] Init done");
}

void int_to_RMT(rmt_item32_t *pixel_x_index, pixel_data_s *intdata_pixel_RGD)
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
	
	memcpy(pixel_x_index, &RMT_pixel_local, RMT_RGB_DATA_PACKAGE_LEN);
	
}

void Fill_rmt_tx_pixel_array(rmt_item32_t *rmt_array_pixel_0_index, uint16_t pixel_index, pixel_data_s *pixel_data)
{
	rmt_item32_t *rmt_array_pixel_x_index;
	//Increment pointer to RMT pixel array, points to location of next pixel data
	rmt_array_pixel_x_index = rmt_array_pixel_0_index + (pixel_index * RMT_RGB_DATA_ITEMS);
	
	int_to_RMT(rmt_array_pixel_x_index, pixel_data);

}

void Fill_RMT_syncdelay_array(rmt_item32_t * RMT_syncdelay_packet_pointer)
{
	memcpy(RMT_syncdelay_packet_pointer, packet_syncdata, (size_sync*sizeof(rmt_item32_t)));
	(RMT_syncdelay_packet_pointer + size_sync)->duration0 = (uint32_t)(postsyncduration - (extra_delay / 2));
	(RMT_syncdelay_packet_pointer + size_sync)->level0 = 0;
	(RMT_syncdelay_packet_pointer + size_sync)->duration1 = (uint32_t)(postsyncduration - (extra_delay / 2));
	(RMT_syncdelay_packet_pointer + size_sync)->level1 = 0;
}
