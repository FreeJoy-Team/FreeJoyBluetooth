/*
 * FreeJoy UART to BLE
 */

#include <Arduino.h>
#include <BleGamepad.h>

#define LED_PIN 8
uint8_t led_activated = 0;

#define numOfButtons              128
#define numOfHatSwitches          0
#define enableX                   true
#define enableY                   true
#define enableZ                   true
#define enableRZ                  true
#define enableRX                  true
#define enableRY                  true
#define enableSlider1             true
#define enableSlider2             true
#define enableRudder              false
#define enableThrottle            false
#define enableAccelerator         false
#define enableBrake               false
#define enableSteering            false

BleGamepad bleGamepad("FreeJoy joystick", "FreeJoy", 100);

#define MAX_AXIS_NUM							8
#define MAX_BUTTONS_NUM						128		
#define HEADER                    'H'
#define SEPARATOR                 '-'

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t separator;
  uint8_t message_code;
  int16_t axis_data[MAX_AXIS_NUM];
  uint8_t buttons_data[MAX_BUTTONS_NUM/8];
  uint16_t crc;
} uart_report_t;

enum ReceiveState {
  WAITING_FOR_HEADER,
  RECEIVING_DATA
};

uart_report_t received_data;
ReceiveState receive_state = WAITING_FOR_HEADER;
uint8_t receive_buffer[sizeof(uart_report_t)];
uint16_t bytes_received = 0;

/**
  * @brief CRC16 calc with polynominal 0x8005
	* @param data: data for calc
	* @param size: size of data
  */
uint16_t gen_crc16(const uint8_t *data, uint16_t size)
{
	uint16_t out = 0, crc = 0;
	int32_t bits_read = 0, bit_flag = 0, i = 0;
	int32_t j = 0x0001;
	
	if (data == NULL) return 0;
	
	while (size > 0)
	{
		bit_flag = out >> 15;
		out <<= 1;
		out |= (*data >> bits_read) & 1;
		
		bits_read++;
		if (bits_read > 7)
		{
			bits_read = 0;
			data++;
			size --;
		}
		
		if (bit_flag) out ^= 0x8005;
	}
	
	for (i = 0; i < 16; ++i)
	{
		bit_flag = out >> 15;
		out <<= 1;
		if (bit_flag) out ^= 0x8005;
	}
	
	i = 0x8000;
	for (; i !=0; i >>= 1, j <<= 1)
	{
		if (i & out) crc |= j;
	}
	
	return crc;
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK); // CONTROLLER_TYPE_JOYSTICK, CONTROLLER_TYPE_GAMEPAD (DEFAULT), CONTROLLER_TYPE_MULTI_AXIS
  bleGamepadConfig.setButtonCount(numOfButtons);
  bleGamepadConfig.setWhichAxes(enableX, enableY, enableZ, enableRX, enableRY, enableRZ, enableSlider1, enableSlider2);      // Can also be done per-axis individually. All are true by default
  bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);                                                                      // 1 by default
  // Some non-Windows operating systems and web based gamepad testers don't like min axis set below 0, so 0 is set by default
  bleGamepadConfig.setAxesMin(0x8001); // -32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal
  bleGamepadConfig.setAxesMax(0x7FFF); // 32767 --> int16_t - 16 bit signed integer - Can be in decimal or hexadecimal 
  // enable output
  bleGamepadConfig.setEnableOutputReport(true);
  bleGamepadConfig.setOutputReportLength(64);
  //bleGamepadConfig.setVid();
  //bleGamepadConfig.setPid();
  bleGamepad.begin(&bleGamepadConfig);
  // changing bleGamepadConfig after the begin function has no effect, unless you call the begin function again

  pinMode(LED_PIN, OUTPUT);
}


void processReceivedData() {
  if (received_data.header != HEADER || received_data.separator != SEPARATOR) {
    Serial.println("Error: Invalid header or separator");
    return;
  }
  
  // CRC
  uint16_t calc_crc = gen_crc16((uint8_t*)&received_data, sizeof(uart_report_t) - sizeof(uint16_t));
  
  if (calc_crc != received_data.crc) {
    Serial.print("Error: CRC mismatch. Calculated: ");
    Serial.print(calc_crc, HEX);
    Serial.print(", Received: ");
    Serial.println(received_data.crc, HEX);
    return;
  }

  Serial.println("Data received successfully!");

  updateJoyState();
}


void updateJoyState() {
  if (!bleGamepad.isConnected()) return;

  bleGamepad.setAxes(received_data.axis_data[0], received_data.axis_data[1], 
  received_data.axis_data[2], received_data.axis_data[3], 
  received_data.axis_data[4], received_data.axis_data[5], 
  received_data.axis_data[6], received_data.axis_data[7]);

  // custom code in BleGamepad.cpp
  //bleGamepad.setAllButtonsState(received_data.buttons_data);

  for (uint8_t i = 0, number = 0; i < MAX_BUTTONS_NUM/8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      number = j + (i *8);
      if ((received_data.buttons_data[i] & (1 << (j & 0x07)))) {
        bleGamepad.press(number);
      } else if ((received_data.buttons_data[i] & (1 << (j & 0x07))) == false) {
        bleGamepad.release(number);
      }
    }
  }
  bleGamepad.sendReport();
}


void loop()
{
  while (bleGamepad.isConnected() && Serial.available() > 0) {
    uint8_t received_byte = Serial.read();
    
    switch (receive_state) {
      case WAITING_FOR_HEADER:
        if (received_byte == HEADER) {
            receive_buffer[0] = received_byte;
            bytes_received = 1;
            receive_state = RECEIVING_DATA;
        }
        break;
          
      case RECEIVING_DATA:
        receive_buffer[bytes_received] = received_byte;
        bytes_received++;
        
        // all data received
        if (bytes_received >= sizeof(uart_report_t)) {
          memcpy(&received_data, receive_buffer, sizeof(uart_report_t));

          processReceivedData();

          receive_state = WAITING_FOR_HEADER;
          bytes_received = 0;
        }
        break;
    }
  }

  if (bleGamepad.isConnected() && !led_activated) {
    digitalWrite(LED_PIN, LOW);
  } else if (led_activated) {
    digitalWrite(LED_PIN, HIGH);
  }
}