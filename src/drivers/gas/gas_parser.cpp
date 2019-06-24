// #include <string.h>
// #include <stdlib.h>
// #include <stdio.h>
// #include <string.h>
// #include <stdlib.h>
// #include <px4_getopt.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "gas_parser.h"

#include <uORB/uORB.h>
#include <uORB/topics/sensor_gas.h>


void Parser::Parse(uint8_t c){

    //std::cout << "current state : "<< expectedState <<"\tvalue : "<< (int)c  <<"\n";
    if(expectedState == STATE_START_BYTE) {
        if (c == START_HIGH) {
            expectedState = STATE_START_LOW_BYTE;
        }
    } else if (expectedState == STATE_START_LOW_BYTE) {
        if (c == START_LOW) {
            expectedState = STATE_SIZE_OF_PACKET;
        } else {
            expectedState = STATE_START_BYTE;
        }
    } else if (expectedState == STATE_SIZE_OF_PACKET) {
        if(c == SIZE_OF_PACKET){
            expectedState = STATE_VERSION;
        }else {
            expectedState = STATE_START_BYTE;
        }
    } else if(expectedState == STATE_VERSION){       //channel
        if(c == VERSION_OF_PACKET){
            expectedState = STATE_NUM_OF_CHANNEL;
        }else {
            expectedState = STATE_START_BYTE;
        }
    } else if(expectedState == STATE_NUM_OF_CHANNEL){       //channel
        if(c == 4){
            expectedState = STATE_SENSOR1_CDC_HIGH;
        }else {
            expectedState = STATE_START_BYTE;
        }
    } else if(expectedState == STATE_SENSOR1_CDC_HIGH){     //sensor1
        cdc.sensor1.high = c;
        expectedState = STATE_SENSOR1_CDC_LOW;
    } else if(expectedState == STATE_SENSOR1_CDC_LOW){
        cdc.sensor1.low = c;
        expectedState = STATE_SENSOR1_ID;
    } else if(expectedState == STATE_SENSOR1_ID){
        cdc.sensor1.id = c;
        expectedState = STATE_SENSOR1_STATE;
    } else if(expectedState == STATE_SENSOR1_STATE){
        cdc.sensor1.state = c;
        expectedState = STATE_SENSOR2_CDC_HIGH;
    } else if(expectedState == STATE_SENSOR2_CDC_HIGH){
        cdc.sensor2.high = c;
        expectedState = STATE_SENSOR2_CDC_LOW;
    } else if(expectedState == STATE_SENSOR2_CDC_LOW){
        cdc.sensor2.low = c;
        expectedState = STATE_SENSOR2_ID;
    } else if(expectedState == STATE_SENSOR2_ID){
        cdc.sensor2.id = c;
        expectedState = STATE_SENSOR2_STATE;
    } else if(expectedState == STATE_SENSOR2_STATE){
        cdc.sensor2.state = c;
        expectedState = STATE_SENSOR3_CDC_HIGH;
    } else if(expectedState == STATE_SENSOR3_CDC_HIGH){
        cdc.sensor3.high = c;
        expectedState = STATE_SENSOR3_CDC_LOW;
    } else if(expectedState == STATE_SENSOR3_CDC_LOW){
        cdc.sensor3.low = c;
        expectedState = STATE_SENSOR3_ID;
    } else if(expectedState == STATE_SENSOR3_ID){
        cdc.sensor3.id = c;
        expectedState = STATE_SENSOR3_STATE;
    } else if(expectedState == STATE_SENSOR3_STATE){
        cdc.sensor3.state = c;
        expectedState = STATE_SENSOR4_CDC_HIGH;
    } else if(expectedState == STATE_SENSOR4_CDC_HIGH){
        cdc.sensor4.high = c;
        expectedState = STATE_SENSOR4_CDC_LOW;
    } else if(expectedState == STATE_SENSOR4_CDC_LOW){
        cdc.sensor4.low = c;
        expectedState = STATE_SENSOR4_ID;
    } else if(expectedState == STATE_SENSOR4_ID){
        cdc.sensor4.id = c;
        expectedState = STATE_SENSOR4_STATE;
    } else if(expectedState == STATE_SENSOR4_STATE){
        cdc.sensor4.state = c;
        expectedState = STATE_TEMP_HIGH;
    } else if(expectedState == STATE_TEMP_HIGH){
        cdc.temp_high = c;
        expectedState = STATE_TEMP_LOW;
    } else if(expectedState == STATE_TEMP_LOW){
        cdc.temp_low = c;
        expectedState = STATE_HUM_HIGH;
    } else if(expectedState == STATE_HUM_HIGH){
        cdc.hum_high = c;
        expectedState = STATE_HUM_LOW;
    }else if(expectedState == STATE_HUM_LOW){
        cdc.hum_low = c;
        expectedState = STATE_SENSOR_ID;
    } else if(expectedState == STATE_SENSOR_ID){
        cdc.sensor_id = c;
        expectedState = STATE_MATERIAL_ID;
    } else if (expectedState == STATE_MATERIAL_ID) {
        cdc.material_id = c;
        expectedState = STATE_SENSOR_STATE;
    } else if (expectedState == STATE_SENSOR_STATE) {
        cdc.sensor_state = c;
        expectedState = STATE_INTERVAL_INDEX;
    } else if (expectedState == STATE_INTERVAL_INDEX) {
        cdc.intervel_index = c;
        expectedState = STATE_EMA;
    } else if (expectedState == STATE_EMA) {
        cdc.ema = c;
        expectedState = STATE_PROGVER_HIGH;
    } else if (expectedState == STATE_PROGVER_HIGH) {
        cdc.prog_ver_high = c;
        expectedState = STATE_PROGVER_LOW;
    } else if (expectedState == STATE_PROGVER_LOW) {
        cdc.prog_ber_low = c;
        expectedState = STATE_CH1_HIGH;
    } else if (expectedState == STATE_CH1_HIGH) {
        cdc.ch1.high = c;
        expectedState = STATE_CH1_LOW;
    } else if (expectedState == STATE_CH1_LOW) {
        cdc.ch1.low = c;
        expectedState = STATE_CH2_HIGH;
    } else if (expectedState == STATE_CH2_HIGH) {
        cdc.ch2.high = c;
        expectedState = STATE_CH2_LOW;
    } else if (expectedState == STATE_CH2_LOW) {
        cdc.ch2.low = c;
        expectedState = STATE_CH3_HIGH;
    } else if (expectedState == STATE_CH3_HIGH) {
        cdc.ch3.high = c;
        expectedState = STATE_CH3_LOW;
    } else if (expectedState == STATE_CH3_LOW) {
        cdc.ch3.low = c;
        expectedState = STATE_CH4_HIGH;
    } else if (expectedState == STATE_CH4_HIGH) {
        cdc.ch4.high = c;
        expectedState = STATE_CH4_LOW;
    } else if (expectedState == STATE_CH4_LOW) {
        cdc.ch4.low = c;
        expectedState = STATE_RESERVED1;
    } else if (expectedState == STATE_RESERVED1) {
        cdc.reserved1 = c;
        expectedState = STATE_RESERVED2;
    } else if (expectedState == STATE_RESERVED2) {
        cdc.reserved2 = c;
        expectedState = STATE_RESERVED3;
    } else if (expectedState == STATE_RESERVED3) {
        cdc.reserved3 = c;
        expectedState = STATE_RESERVED4;
    } else if (expectedState == STATE_RESERVED4) {
        cdc.reserved4 = c;
        expectedState = STATE_SERIAL_HIGH;
    } else if (expectedState == STATE_SERIAL_HIGH) {
        cdc.serial_high = c;
        expectedState = STATE_SERIAL_LOW;
    } else if (expectedState == STATE_SERIAL_LOW) {
        cdc.serial_low = c;
        expectedState = STATE_EOF;
    } else if (expectedState == STATE_EOF)
    {
        if(c == 0xFF){
            //Done!!
            PX4_INFO("Done!!!\n");

            struct sensor_gas_s raw;
    	    memset(&raw, 0, sizeof(raw));
	        orb_advert_t gas_pub = orb_advertise(ORB_ID(sensor_gas), &raw);

            raw.temp = 0;
            raw.temp =  cdc.temp_high << 8 ;
            raw.temp += cdc.temp_low;

            orb_publish(ORB_ID(sensor_gas), gas_pub, &raw);
            // raw.humid = ;   // cdc.hub_high + cdc.hum_low
            // raw.sensor1_cdc = ;  //cdc.sensor1.high + cdc.sensor1.low
            // raw.sensor2_cdc = ;  //cdc.sensor2.high + cdc.sensor2.low
            // raw.sensor3_cdc = ;  //cdc.sensor3.high + cdc.sensor3.low
            // raw.sensor4_cdc = ;  //cdc.sensor4.high + cdc.sensor4.low
        }
        expectedState = STATE_START_BYTE;
    }


}
