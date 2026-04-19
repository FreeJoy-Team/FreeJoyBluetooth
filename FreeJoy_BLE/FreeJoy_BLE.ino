/*
 * FreeJoy UART to BLE for ESP32-C3
 * Revised UART receiver
 */

#include <Arduino.h>
#include <BleGamepad.h>

// ===================== PINS =====================
#define LED_PIN 8

// УКАЖИ СВОИ ПИНЫ UART ESP32-C3
#define UART_RX_PIN 20
#define UART_TX_PIN 21

// ===================== BLE CONFIG =====================
#define NUM_BUTTONS       128
#define NUM_HAT_SWITCHES  0

#define ENABLE_X          true
#define ENABLE_Y          true
#define ENABLE_Z          true
#define ENABLE_RZ         true
#define ENABLE_RX         true
#define ENABLE_RY         true
#define ENABLE_SLIDER1    true
#define ENABLE_SLIDER2    true

BleGamepad bleGamepad("FreeJoy joystick", "FreeJoy", 100);
HardwareSerial UartPort(1);

// ===================== PROTOCOL =====================
#define MAX_AXIS_NUM      8
#define MAX_BUTTONS_NUM   128
#define HEADER            'H'
#define SEPARATOR         '-'

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t separator;
  uint8_t message_code;
  int16_t axis_data[MAX_AXIS_NUM];
  uint8_t buttons_data[MAX_BUTTONS_NUM / 8];
  uint16_t crc;
} uart_report_t;

static_assert(sizeof(uart_report_t) == 37, "uart_report_t size must be 37 bytes");

enum ReceiveState {
  WAITING_FOR_HEADER,
  RECEIVING_DATA
};

uart_report_t received_data;
ReceiveState receive_state = WAITING_FOR_HEADER;
uint8_t receive_buffer[sizeof(uart_report_t)];
uint16_t bytes_received = 0;

bool message_code_initialized = false;
uint8_t last_message_code = 0;

// ===================== CRC16 =====================
uint16_t gen_crc16(const uint8_t *data, uint16_t size) {
  uint16_t out = 0, crc = 0;
  int32_t bits_read = 0, bit_flag = 0, i = 0;
  int32_t j = 0x0001;

  if (data == NULL) return 0;

  while (size > 0) {
    bit_flag = out >> 15;
    out <<= 1;
    out |= (*data >> bits_read) & 1;

    bits_read++;
    if (bits_read > 7) {
      bits_read = 0;
      data++;
      size--;
    }

    if (bit_flag) out ^= 0x8005;
  }

  for (i = 0; i < 16; ++i) {
    bit_flag = out >> 15;
    out <<= 1;
    if (bit_flag) out ^= 0x8005;
  }

  i = 0x8000;
  for (; i != 0; i >>= 1, j <<= 1) {
    if (i & out) crc |= j;
  }

  return crc;
}

// ===================== BLE UPDATE =====================
void updateJoyState() {
  if (!bleGamepad.isConnected()) return;

  bleGamepad.setAxes(
    received_data.axis_data[0],
    received_data.axis_data[1],
    received_data.axis_data[2],
    received_data.axis_data[3],
    received_data.axis_data[4],
    received_data.axis_data[5],
    received_data.axis_data[6],
    received_data.axis_data[7]
  );

  for (uint8_t i = 0; i < MAX_BUTTONS_NUM / 8; i++) {
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t buttonNumber = (i * 8) + j + 1; // Важно: обычно BleGamepad ждёт кнопки с 1
      bool pressed = (received_data.buttons_data[i] & (1 << j)) != 0;

      if (pressed) {
        bleGamepad.press(buttonNumber);
      } else {
        bleGamepad.release(buttonNumber);
      }
    }
  }

  bleGamepad.sendReport();
}

// ===================== PACKET PROCESS =====================
void processReceivedData() {
  if (received_data.header != HEADER || received_data.separator != SEPARATOR) {
    Serial.println("UART error: invalid header or separator");
    return;
  }

  uint16_t calc_crc = gen_crc16((const uint8_t*)&received_data, sizeof(uart_report_t) - sizeof(uint16_t));
  if (calc_crc != received_data.crc) {
    Serial.print("UART CRC mismatch. Calc=0x");
    Serial.print(calc_crc, HEX);
    Serial.print(" Recv=0x");
    Serial.println(received_data.crc, HEX);
    return;
  }

  if (!message_code_initialized) {
    last_message_code = received_data.message_code;
    message_code_initialized = true;
  } else {
    uint8_t expected = (uint8_t)(last_message_code + 1);
    if (received_data.message_code != expected) {
      Serial.print("UART packet skip/reorder. Expected=");
      Serial.print(expected);
      Serial.print(" Got=");
      Serial.println(received_data.message_code);
    }
    last_message_code = received_data.message_code;
  }

  updateJoyState();
}

// ===================== UART RECEIVE =====================
void handleUartReceive() {
  while (UartPort.available() > 0) {
    uint8_t received_byte = UartPort.read();

    switch (receive_state) {
      case WAITING_FOR_HEADER:
        if (received_byte == HEADER) {
          receive_buffer[0] = received_byte;
          bytes_received = 1;
          receive_state = RECEIVING_DATA;
        }
        break;

      case RECEIVING_DATA:
        receive_buffer[bytes_received++] = received_byte;

        // Ранняя проверка второго байта
        if (bytes_received == 2) {
          if (receive_buffer[1] != SEPARATOR) {
            receive_state = WAITING_FOR_HEADER;
            bytes_received = 0;
          }
        }

        if (bytes_received >= sizeof(uart_report_t)) {
          memcpy(&received_data, receive_buffer, sizeof(uart_report_t));
          processReceivedData();

          receive_state = WAITING_FOR_HEADER;
          bytes_received = 0;
        }
        break;
    }
  }
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Starting BLE + UART receiver");

  UartPort.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  BleGamepadConfiguration bleGamepadConfig;
  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setButtonCount(NUM_BUTTONS);
  bleGamepadConfig.setHatSwitchCount(NUM_HAT_SWITCHES);
  bleGamepadConfig.setWhichAxes(
    ENABLE_X,
    ENABLE_Y,
    ENABLE_Z,
    ENABLE_RX,
    ENABLE_RY,
    ENABLE_RZ,
    ENABLE_SLIDER1,
    ENABLE_SLIDER2
  );
  bleGamepadConfig.setAxesMin(0x8001);
  bleGamepadConfig.setAxesMax(0x7FFF);
  bleGamepadConfig.setEnableOutputReport(true);
  bleGamepadConfig.setOutputReportLength(64);

  bleGamepad.begin(&bleGamepadConfig);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.print("uart_report_t size = ");
  Serial.println(sizeof(uart_report_t));
}

// ===================== LOOP =====================
void loop() {
  handleUartReceive();

  // Светодиод: LOW при подключении BLE, HIGH если не подключено
  digitalWrite(LED_PIN, bleGamepad.isConnected() ? LOW : HIGH);
}
