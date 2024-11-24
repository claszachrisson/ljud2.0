#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"

#define TIMER_PERIOD_MS 5000

/*
Todo:
Rewrite lockActiveSource() to not use PLL lock

(maybe)
Create general class for amps since they share many methods etc
Create class tas5827 extends(amp)
			 class tas5805m extends(amp)
*/


i2c_master_bus_handle_t i2c_handle;
i2c_master_dev_handle_t codec_i2c;
i2c_master_dev_handle_t amplr_i2c;
i2c_master_dev_handle_t sub_i2c;

i2c_device_config_t codec_i2c_cfg;
i2c_device_config_t amplr_i2c_cfg;
i2c_device_config_t sub_i2c_cfg;

static const char *I2C_TAG = "I2C_HANDLE";
static const char *CODEC_TAG = "CODEC";
static const char *AMPLR_TAG = "TAS5827";
static const char *SUB_TAG = "TAS5805M";

const TickType_t xDelay = 20 / portTICK_PERIOD_MS;

int currentSource = 0;

enum control_port {
	GOTO_PG,
	RESET_CTRL,
	DEVICE_CTRL_1,
	DEVICE_CTRL_2,
	SIG_CH_CTRL = 0x28,
	FS_MON = 0x37,
	CLKDET_STATUS = 0x39,
	DIG_VOL_CTRL = 0x4C,
	DIG_VOL_CTRL_RIGHT = 0x4D,
	AUTO_MUTE_CTRL = 0x50,
	AUTO_MUTE_TIME = 0x51,
	DSP_MISC = 0x66,
	POWER_STATE = 0x68,
	CHAN_FAULT = 0x70,
	GLOBAL_FAULT_1 = 0x71,
	GLOBAL_FAULT_2 = 0x72,
	FAULT_CLEAR = 0x78,
	GOTO_BK = 0x7f,
	PROG_DELAY = 0xFF
};

void i2c_init(){
	i2c_master_bus_config_t i2c_mst_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = GPIO_NUM_10,
		.scl_io_num = GPIO_NUM_9,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
	};

	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_handle));

	codec_i2c_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = 0x10,
		.scl_speed_hz = 100000,
	};
	amplr_i2c_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = 0x60, // TAS5827 (Amp LR) address
		.scl_speed_hz = 100000,
	};
	sub_i2c_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = 0x2C, // Tas5805M (Sub) address
		.scl_speed_hz = 100000,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &codec_i2c_cfg, &codec_i2c));
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &amplr_i2c_cfg, &amplr_i2c));
	ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_handle, &sub_i2c_cfg, &sub_i2c));
}

int checkPLLLocked(){
	uint8_t data[1];
	i2c_master_transmit_receive(codec_i2c, (uint8_t[]){0x0C}, 1, data, 1, -1); // Read register once to reset RERR
	i2c_master_transmit_receive(codec_i2c, (uint8_t[]){0x0C}, 1, data, 1, -1);
	ESP_LOGI(CODEC_TAG, "Receiver error: %02X", data[0]);
	if(data[0] & 16) { // UNLOCK bit is set
		return 0;
	}
	ESP_LOGI(CODEC_TAG, "PLL locked");
	return 1;
}
int sourceIsActive(){
	if(!checkPLLLocked()) return 0;
	uint8_t data[1];
	i2c_master_transmit_receive(codec_i2c, (uint8_t[]){0x0B}, 1, data, 1, -1);
	if(data[0] & 2){ // Digital silence is set;
		return 0;
	}
	return (data[0] & 0xF0) == 1;
}
void changeSource() {
	if(currentSource) { // Current source is RXP1, set to RXP0
		i2c_master_transmit(codec_i2c, (uint8_t[]){0x04,0x81}, 2, -1);
		currentSource = 0;
	}
	else {
		i2c_master_transmit(codec_i2c, (uint8_t[]){0x04,0x89}, 2, -1);
		currentSource = 1;
	}
}
int lockActiveSource(){
	// Needs rewriting as PLL may be locked without an audio data stream
	// Either check codec for digital silence or !AUDIO on GPIO (needs configuring)
	if(sourceIsActive()) return 1;
	changeSource();
	ESP_LOGI(CODEC_TAG, "Source changed");
	return sourceIsActive();
}
void codec_init(){
	i2c_master_transmit(codec_i2c, (uint8_t[]){0x04,0x81}, 2, -1); // Set state to RUN
	vTaskDelay(xDelay);
	i2c_master_transmit(codec_i2c, (uint8_t[]){0x05,0x80}, 2, -1); // Set serial audio mode to master
	i2c_master_transmit(codec_i2c, (uint8_t[]){0x06,0x7F}, 2, -1); // Set receiver error mask to ones (unmasked)
}
void amplr_init(){
	i2c_master_transmit(amplr_i2c, (uint8_t[]){DEVICE_CTRL_2,0x02}, 2, -1); // Set power state to Hi-Z
	vTaskDelay(xDelay);
	i2c_master_transmit(amplr_i2c, (uint8_t[]){0x30,0x01}, 2, -1); // Set SDOUT to DSP input
	i2c_master_transmit(amplr_i2c, (uint8_t[]){0x60,0x04}, 2, -1); // Set GPIO2 to output
	i2c_master_transmit(amplr_i2c, (uint8_t[]){0x63,0x09}, 2, -1); // Set GPIO2 to output SDOUT
	i2c_master_transmit(amplr_i2c, (uint8_t[]){0x4C,0x60}, 2, -1); // Set left channel to -24 dB
	i2c_master_transmit(amplr_i2c, (uint8_t[]){0x4D,0x60}, 2, -1); // Set right channel to -24 dB
}
void sub_init(){
	i2c_master_transmit(sub_i2c, (uint8_t[]){DEVICE_CTRL_2,0x02}, 2, -1); // Set power state to Hi-Z
	vTaskDelay(xDelay);
	i2c_master_transmit(sub_i2c, (uint8_t[]){DEVICE_CTRL_1,0x04}, 2, -1); // Set DAMP to PBTL mode
	i2c_master_transmit(sub_i2c, (uint8_t[]){0x4C,0x48}, 2, -1); // Set digital volume to -12 dB

	uint8_t bq_coeffs[21] = { 0x18, // 100Hz butterworth LPF
		0x00, 0x00, 0x05, 0x97, 
		0x00, 0x00, 0x0b, 0x2d, 
		0x00, 0x00, 0x05, 0x97, 
		0x0f, 0xed, 0x0b, 0x39, 
		0xf8, 0x12, 0xde, 0x06
	};
	i2c_master_transmit(sub_i2c, (uint8_t[]){GOTO_PG,0x00}, 2, -1); // Go to page0
	i2c_master_transmit(sub_i2c, (uint8_t[]){GOTO_BK,0xAA}, 2, -1); // Go to book AA
	i2c_master_transmit(sub_i2c, (uint8_t[]){GOTO_PG,0x24}, 2, -1); // Go to page 24
	i2c_master_transmit(sub_i2c, bq_coeffs, 21, -1); // Write BQ coeffs
	i2c_master_transmit(sub_i2c, (uint8_t[]){GOTO_PG,0x00}, 2, -1); // Go to page0
	i2c_master_transmit(sub_i2c, (uint8_t[]){GOTO_BK,0x00}, 2, -1); // Go to book 0
}
esp_err_t ampsErrorCheck(){
	uint8_t data[3];
	int err = 0;
	i2c_master_transmit_receive(amplr_i2c, (uint8_t[]){0x70}, 1, &data[0], 1, -1);
	i2c_master_transmit_receive(amplr_i2c, (uint8_t[]){0x71}, 1, &data[1], 1, -1);
	i2c_master_transmit_receive(amplr_i2c, (uint8_t[]){0x72}, 1, &data[2], 1, -1);
	if((data[0] | data[1] | data[2])) {
		err = 1;
		ESP_LOGE(AMPLR_TAG, "chan_fault: 0x%02X, global_fault1: 0x%02X, global_fault2: 0x%02X",data[0],data[1],data[2]);
		i2c_master_transmit(amplr_i2c, (uint8_t[]){0x78,0x80}, 2, -1);
	}
	i2c_master_transmit_receive(sub_i2c, (uint8_t[]){0x70}, 1, &data[0], 1, -1);
	i2c_master_transmit_receive(sub_i2c, (uint8_t[]){0x71}, 1, &data[1], 1, -1);
	i2c_master_transmit_receive(sub_i2c, (uint8_t[]){0x72}, 1, &data[2], 1, -1);
	if((data[0] | data[1] | data[2])) {
		err = 1;
		ESP_LOGE(SUB_TAG, "chan_fault: 0x%02X, global_fault1: 0x%02X, global_fault2: 0x%02X",data[0],data[1],data[2]);
		i2c_master_transmit(sub_i2c, (uint8_t[]){0x78,0x80}, 2, -1);
	}
	if(!err){
		ESP_LOGI("APP_MAIN", "Amps operating normally (no faults)");
		return ESP_OK;
	}
	else{
		return ESP_FAIL;
	}
}

uint8_t getPowerState(i2c_master_dev_handle_t *dev_handle, bool log=false, const char *tag=""){
	uint8_t data[1];
	i2c_master_transmit_receive(amplr_i2c, (uint8_t[]){0x68}, 1, data, 1, -1);
	if(log & !strcmp(tag,"")){
		ESP_LOGI(tag, "Device is in state 0x%02X",data[0]);
	}
	return data[0];
}

void task_function(void *pvParameter) {
	/*
	Right now lockActiveSource() is called to check channels for audio 
	every time task_function() is called.
	Better strategy would be to have a cooldown, maybe lock a source for
	x seconds every time audio is detected, and keep updating cooldown 
	each time.
	Maybe something like this (increases time lock for every iteration
	that audio is present, to a max lock of a minute (assuming it's run
	once every second), to not lock excessively on short bursts)

	int timelock = 0;
	if(audio & (timelock < 60)) { lock++; }
	elseif(!audio) lock--;
	*/
	lockActiveSource();
	ampsErrorCheck();
	vTaskDelete(NULL);  // Delete the task when done
}

void timer_callback(TimerHandle_t xTimer) {
	xTaskCreate(&task_function, "WorkTask", 4096, NULL, 5, NULL);
}

extern "C" void app_main(void)
{
	// Set PDN high to enable amplifiers
	gpio_num_t PDN = GPIO_NUM_3;
	gpio_set_direction(PDN, GPIO_MODE_OUTPUT);
	gpio_set_level(PDN, 1);

	i2c_init();
	codec_init();
	amplr_init();

	esp_err_t err;
	if((err = i2c_master_probe(i2c_handle, codec_i2c_cfg.device_address, 10)) != ESP_OK){
		ESP_LOGE(I2C_TAG, "Failed probing codec: %d", err); // Probably don't continue past this point?
	}
	if((err = i2c_master_probe(i2c_handle, amplr_i2c_cfg.device_address, 10)) != ESP_OK) {
		ESP_LOGE(I2C_TAG, "Failed probing TAS5827: %d", err);
	}
	if((err = i2c_master_probe(i2c_handle, sub_i2c_cfg.device_address, 10)) != ESP_OK) {
		ESP_LOGE(I2C_TAG, "Failed probing TAS5805M: %d", err);
	}

	while(!lockActiveSource()) { // Wait for PLL to lock before continuing
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
	if(checkPLLLocked()) {
		sub_init();
	}
	ESP_LOGI("APP_MAIN", "Amps initialised");

	getPowerState(&amplr_i2c, true, AMPLR_TAG);
	getPowerState(&sub_i2c, true, SUB_TAG);

	if(ampsErrorCheck() == ESP_OK){
		i2c_master_transmit(amplr_i2c, (uint8_t[]){0x03,0x03}, 2, -1);
		i2c_master_transmit(sub_i2c, (uint8_t[]){0x03,0x03}, 2, -1);
	}
	vTaskDelay(xDelay);

	uint8_t pwrstate = getPowerState(&amplr_i2c);
	if(pwrstate != 0x03) {
		ESP_LOGE(AMPLR_TAG, "Failed to set device to play state");
	}
	pwrstate = getPowerState(&sub_i2c);
	if(pwrstate != 0x03) {
		ESP_LOGE(SUB_TAG, "Failed to set device to play state");
	}
	int playing = 0;
	while(!playing){
		vTaskDelay(xDelay);
		esp_err_t err = ampsErrorCheck();
		if(err == ESP_OK){
			i2c_master_transmit(amplr_i2c, (uint8_t[]){0x03,0x03}, 2, -1);
			i2c_master_transmit(sub_i2c, (uint8_t[]){0x03,0x03}, 2, -1);
			uint8_t amplrpwrstate = getPowerState(&amplr_i2c);
			uint8_t subpwrstate = getPowerState(&sub_i2c);
			if((amplrpwrstate == 0x03) & (subpwrstate == 0x03)){
				playing = 1;
				ESP_LOGI("APP_MAIN", "Amps set to play state");
			}
			else {
				ESP_LOGE("APP_MAIN", "Failed to set amps to play state");
			}
		}
	}

	TimerHandle_t timer = xTimerCreate("MyTimer", pdMS_TO_TICKS(TIMER_PERIOD_MS), pdTRUE, (void *)0, timer_callback);
	// Check if the timer was created successfully
	if (timer != NULL) {
			// Start the timer
			xTimerStart(timer, 0);
	} else {
			ESP_LOGE("APP_MAIN", "Failed to create timer");
	}
}
