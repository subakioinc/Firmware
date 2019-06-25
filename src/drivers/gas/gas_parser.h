#include <stdint.h>



//변수 선언
#define START_HIGH  0x11
#define START_LOW 0X22
#define SIZE_OF_PACKET 0x2F
#define VERSION_OF_PACKET 0xC5
#define NUM_OF_CHANNEL 4

enum CDC_STATE{
    STATE_START_BYTE = 0,
    STATE_START_LOW_BYTE,
    STATE_SIZE_OF_PACKET,
    STATE_VERSION,
    STATE_NUM_OF_CHANNEL,

    STATE_SENSOR1_CDC_HIGH,
    STATE_SENSOR1_CDC_LOW,
    STATE_SENSOR1_ID,
    STATE_SENSOR1_STATE,

    STATE_SENSOR2_CDC_HIGH,
    STATE_SENSOR2_CDC_LOW,
    STATE_SENSOR2_ID,
    STATE_SENSOR2_STATE,

    STATE_SENSOR3_CDC_HIGH,
    STATE_SENSOR3_CDC_LOW,
    STATE_SENSOR3_ID,
    STATE_SENSOR3_STATE,

    STATE_SENSOR4_CDC_HIGH,
    STATE_SENSOR4_CDC_LOW,
    STATE_SENSOR4_ID,
    STATE_SENSOR4_STATE,

    STATE_TEMP_HIGH,
    STATE_TEMP_LOW,

    STATE_HUM_HIGH,
    STATE_HUM_LOW,

    STATE_SENSOR_ID,
    STATE_MATERIAL_ID,
    STATE_SENSOR_STATE,
    STATE_INTERVAL_INDEX,
    STATE_EMA,

    STATE_PROGVER_HIGH,
    STATE_PROGVER_LOW,

    STATE_CH1_HIGH,
    STATE_CH1_LOW,
    STATE_CH2_HIGH,
    STATE_CH2_LOW,
    STATE_CH3_HIGH,
    STATE_CH3_LOW,
    STATE_CH4_HIGH,
    STATE_CH4_LOW,
    STATE_RESERVED1,
    STATE_RESERVED2,
    STATE_RESERVED3,
    STATE_RESERVED4,
    STATE_SERIAL_HIGH,
    STATE_SERIAL_LOW,
    STATE_EOF
};

struct Sensor
{
    uint8_t high, low, id, state;
};

struct Ch
{
    uint8_t high, low;
};

struct CDCStruct {
    uint8_t size;
    uint8_t version;
    uint8_t num_of_channel;
    Sensor sensor1, sensor2, sensor3, sensor4;
    uint8_t temp_high;
    uint8_t temp_low;
    uint8_t hum_high;
    uint8_t hum_low;
    uint8_t sensor_id;
    uint8_t material_id;
    uint8_t sensor_state;
    uint8_t intervel_index;
    uint8_t ema;
    uint8_t prog_ver_high;
    uint8_t prog_ber_low;
    Ch ch1, ch2, ch3, ch4;
    uint8_t reserved1,reserved2, reserved3,reserved4;
    uint8_t serial_high;
    uint8_t serial_low;
};

class Parser {
    public:
        CDCStruct cdc;
    private:
        CDC_STATE expectedState;

    public:
    Parser(){
        expectedState = STATE_START_BYTE;
    }

    void Parse(uint8_t c);

    uint16_t make_uint16(uint8_t high, uint8_t low);

};




