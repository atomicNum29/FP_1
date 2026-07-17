#include <nrf52840.h>
#include <nrf52840_bitfields.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>

#define PWM_RAM_POLARITY_Pos (15UL)
#define PWM_RAM_POLARITY_FALLING_Val ((1U) << PWM_RAM_POLARITY_Pos)
#define PWM_RAM_COMPARE_Msk (0x7FFFUL)

// falling edge polarity
uint16_t pwm_width[] = {PWM_RAM_POLARITY_FALLING_Val | 0, 0, 0, 0};

void setup_PWM0();
void set_PWM0_width(uint8_t channel, uint16_t width);

uint8_t i2c_tx_buffer[10]; // Buffer for I2C data
uint8_t i2c_rx_buffer[10]; // Buffer for I2C data
void setup_I2C();
void my_i2c_write(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read_register(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
#define AS5600_ADDRESS 0x36 // I2C address of AS5600

void setup_UART();
void my_uart_write(uint8_t *data, uint8_t length);

#define MOTOR_ENCODER_INTERRUPT_PIN_A 21 // P0.21
#define MOTOR_ENCODER_INTERRUPT_PIN_B 23 // P0.23
void setup_QDEC();
void read_QDEC();                         // Function to read the QDEC value and update the motor_encoder_count variable
volatile int32_t motor_encoder_count = 0; // Count of motor encoder pulses
const int32_t pulses_per_revolution = 825; // Number of pulses(steps) per revolution

void setup()
{
  setup_PWM0();
  setup_I2C();
  setup_UART();
  setup_QDEC();
}

void loop()
{
  uint16_t raw_angle = 0x0C;
  my_i2c_read_register(AS5600_ADDRESS, 0x0E, (uint8_t *)&raw_angle, 2); // Read the raw angle from AS5600
  raw_angle = ((raw_angle >> 8) | (raw_angle << 8)) & 0x0FFF;           // Convert from big-endian to little-endian

  read_QDEC(); // Read the QDEC value and update the motor_encoder_count variable

  uint8_t message[64];
  sprintf((char *)message, ">ra:%d,ma:%ld\r\n", raw_angle, motor_encoder_count); // Convert the raw angle and motor encoder count to a string
  my_uart_write(message, strlen((char *)message));   // Send the raw angle over UART

  delay(10);
}

void setup_PWM0()
{
  // Configure PWM
  NRF_P1->OUTCLR = (1 << 11); // Clear P1.11 output
  NRF_P1->DIRSET = (1 << 11); // Set P1.11 as output
  NRF_PWM0->PSEL.OUT[0] = (11 << PWM_PSEL_OUT_PIN_Pos) |
                          (1 << PWM_PSEL_OUT_PORT_Pos) |
                          (PWM_PSEL_OUT_CONNECT_Connected << PWM_PSEL_OUT_CONNECT_Pos);    // Set output pin for channel 0
  NRF_PWM0->PSEL.OUT[1] = (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos); // Set output pin for channel 1
  NRF_PWM0->PSEL.OUT[2] = (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos); // Set output pin for channel 2
  NRF_PWM0->PSEL.OUT[3] = (PWM_PSEL_OUT_CONNECT_Disconnected << PWM_PSEL_OUT_CONNECT_Pos); // Set output pin for channel 3

  NRF_PWM0->ENABLE = (PWM_ENABLE_ENABLE_Enabled << PWM_ENABLE_ENABLE_Pos);
  NRF_PWM0->MODE = PWM_MODE_UPDOWN_Up << PWM_MODE_UPDOWN_Pos;
  NRF_PWM0->PRESCALER = PWM_PRESCALER_PRESCALER_DIV_1 << PWM_PRESCALER_PRESCALER_Pos; // Set prescaler to 1 (16 MHz)
  NRF_PWM0->COUNTERTOP = 1000;                                                        // Set the period of the PWM signal, which is the maximum value of the counter. This will give a PWM frequency of 16 kHz (16 MHz / 1000).
  NRF_PWM0->LOOP = (PWM_LOOP_CNT_Disabled << PWM_LOOP_CNT_Pos);                       // Disable looping
  NRF_PWM0->DECODER = (PWM_DECODER_LOAD_Individual << PWM_DECODER_LOAD_Pos) |
                      (PWM_DECODER_MODE_RefreshCount << PWM_DECODER_MODE_Pos); // Set the decoder to load individual values for each channel and refresh the counter after each period.

  NRF_PWM0->SEQ[0].PTR = (uint32_t)pwm_width;                        // Set the pointer to the array of duty cycle values
  NRF_PWM0->SEQ[0].CNT = (sizeof(pwm_width) / sizeof(pwm_width[0])); // Set the number of values in the sequence
  NRF_PWM0->SEQ[0].REFRESH = 0;                                      // Set the refresh value to 0
  NRF_PWM0->SEQ[0].ENDDELAY = 0;                                     // Set the end delay value to 0

  NRF_PWM0->TASKS_SEQSTART[0] = 1; // Start the sequence
}

void set_PWM0_width(uint8_t channel, uint16_t width)
{
  if (channel < 4)
  {
    if (width > 1000)
    {
      width = 1000; // Limit the width to the maximum value of the counter
    }
    pwm_width[channel] = PWM_RAM_POLARITY_FALLING_Val | (width & PWM_RAM_COMPARE_Msk);
  }
  NRF_PWM0->TASKS_SEQSTART[0] = 1; // Start the sequence
}

void setup_I2C()
{
  // Configure I2C
  NRF_TWIM0->PSEL.SCL = (2 << TWIM_PSEL_SCL_PIN_Pos) | (0 << TWIM_PSEL_SCL_PORT_Pos);   // Set SCL pin to P0.02
  NRF_TWIM0->PSEL.SDA = (31 << TWIM_PSEL_SDA_PIN_Pos) | (0 << TWIM_PSEL_SDA_PORT_Pos);  // Set SDA pin to P0.31
  NRF_TWIM0->FREQUENCY = TWIM_FREQUENCY_FREQUENCY_K400 << TWIM_FREQUENCY_FREQUENCY_Pos; // Set frequency to 100 kHz
  NRF_TWIM0->RXD.PTR = (uint32_t)i2c_rx_buffer;                                         // Set the pointer to the receive buffer
  NRF_TWIM0->RXD.MAXCNT = sizeof(i2c_rx_buffer);
  NRF_TWIM0->RXD.LIST = 0;                      // Disable EasyDMA list for RX
  NRF_TWIM0->TXD.PTR = (uint32_t)i2c_tx_buffer; // Set the pointer to the transmit buffer
  NRF_TWIM0->TXD.MAXCNT = sizeof(i2c_tx_buffer);
  NRF_TWIM0->TXD.LIST = 0; // Disable EasyDMA list for TX
  NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos) |
                      (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos); // Enable shorts for automatic stop after last TX/RX
  NRF_TWIM0->ENABLE = (TWIM_ENABLE_ENABLE_Enabled << TWIM_ENABLE_ENABLE_Pos);           // Enable the I2C peripheral
}

void my_i2c_write(uint8_t address, uint8_t *data, uint8_t length)
{
  NRF_TWIM0->ADDRESS = address;        // Set the I2C address
  memcpy(i2c_tx_buffer, data, length); // Copy data to the transmit buffer
  NRF_TWIM0->TXD.MAXCNT = length;      // Set the number of bytes to transmit
  NRF_TWIM0->TASKS_STARTTX = 1;        // Start the transmission
  while (NRF_TWIM0->EVENTS_STOPPED == 0)
    ;                            // Wait for the transmission to complete
  NRF_TWIM0->EVENTS_STOPPED = 0; // Clear the stopped event
}

void my_i2c_read(uint8_t address, uint8_t *data, uint8_t length)
{
  NRF_TWIM0->ADDRESS = address;   // Set the I2C address
  NRF_TWIM0->RXD.MAXCNT = length; // Set the number of bytes to receive
  NRF_TWIM0->TASKS_STARTRX = 1;   // Start the reception
  while (NRF_TWIM0->EVENTS_STOPPED == 0)
    ;                                  // Wait for the reception to complete
  NRF_TWIM0->EVENTS_STOPPED = 0;       // Clear the stopped event
  memcpy(data, i2c_rx_buffer, length); // Copy data from the receive buffer
}

void my_i2c_read_register(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length)
{
  NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STARTRX_Enabled << TWIM_SHORTS_LASTTX_STARTRX_Pos) |
                      (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos); // Enable shorts for automatic stop
  NRF_TWIM0->ADDRESS = address;                                                         // Set the I2C address
  i2c_tx_buffer[0] = reg;                                                               // Set the register to read from
  NRF_TWIM0->TXD.MAXCNT = 1;                                                            // Set the number of bytes to transmit (the register address)
  NRF_TWIM0->RXD.MAXCNT = length;                                                       // Set the number of bytes to receive
  NRF_TWIM0->TASKS_STARTTX = 1;                                                         // Start the transmission
  while (NRF_TWIM0->EVENTS_STOPPED == 0)
    ;                                  // Wait for the transmission and reception to complete
  NRF_TWIM0->EVENTS_STOPPED = 0;       // Clear the stopped event
  memcpy(data, i2c_rx_buffer, length); // Copy data from the receive buffer
  NRF_TWIM0->SHORTS = (TWIM_SHORTS_LASTTX_STOP_Enabled << TWIM_SHORTS_LASTTX_STOP_Pos) |
                      (TWIM_SHORTS_LASTRX_STOP_Enabled << TWIM_SHORTS_LASTRX_STOP_Pos); // Restore shorts for automatic stop after last TX/RX
}

void setup_UART()
{
  // Configure UART
  NRF_UARTE0->PSEL.TXD = (3 << UARTE_PSEL_TXD_PIN_Pos) | (1 << UARTE_PSEL_TXD_PORT_Pos);      // Set TX pin to P1.03
  NRF_UARTE0->PSEL.RXD = (10 << UARTE_PSEL_RXD_PIN_Pos) | (1 << UARTE_PSEL_RXD_PORT_Pos);     // Set RX pin to P1.10
  NRF_UARTE0->PSEL.RTS = (UARTE_PSEL_RTS_CONNECT_Disconnected << UARTE_PSEL_RTS_CONNECT_Pos); // Set RTS pin to disconnected
  NRF_UARTE0->PSEL.CTS = (UARTE_PSEL_CTS_CONNECT_Disconnected << UARTE_PSEL_CTS_CONNECT_Pos); // Set CTS pin to disconnected
  NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos;   // Set baud rate to 115200
  NRF_UARTE0->CONFIG = (UARTE_CONFIG_PARITY_Excluded << UARTE_CONFIG_PARITY_Pos) |
                       (UARTE_CONFIG_STOP_One << UARTE_CONFIG_STOP_Pos) |
                       (UARTE_CONFIG_HWFC_Disabled << UARTE_CONFIG_HWFC_Pos); // Disable parity and hardware flow control
  // NRF_UARTE0->SHORTS = (UARTE_SHORTS_ << UARTE_SHORTS_ENDTX_STOP_Pos); // Enable short for automatic stop after transmission
  NRF_UARTE0->ENABLE = (UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos); // Enable the UART peripheral
}

void my_uart_write(uint8_t *data, uint8_t length)
{
  NRF_UARTE0->TXD.PTR = (uint32_t)data; // Set the pointer to the transmit buffer
  NRF_UARTE0->TXD.MAXCNT = length;      // Set the number of bytes to transmit
  NRF_UARTE0->TASKS_STARTTX = 1;        // Start the transmission
  while (NRF_UARTE0->EVENTS_ENDTX == 0)
    ;                           // Wait for the transmission to complete
  NRF_UARTE0->EVENTS_ENDTX = 0; // Clear the end event
}

void setup_GPIO_interrupt_for_motor_encoder()
{
  NRF_P0->PIN_CNF[MOTOR_ENCODER_INTERRUPT_PIN_A] =
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos); // Configure P0.21 as input with pull-up and sense for low level
  NRF_P0->PIN_CNF[MOTOR_ENCODER_INTERRUPT_PIN_B] =
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos); // Configure P0.23 as input with pull-up and sense for low level

  NRF_GPIOTE->CONFIG[0] =
      (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
      (MOTOR_ENCODER_INTERRUPT_PIN_A << GPIOTE_CONFIG_PSEL_Pos) |
      (0 << GPIOTE_CONFIG_PORT_Pos) |
      (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
      (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos); // Configure GPIOTE channel 0 for P0.21 with toggle polarity
  NRF_GPIOTE->CONFIG[1] =
      (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
      (MOTOR_ENCODER_INTERRUPT_PIN_B << GPIOTE_CONFIG_PSEL_Pos) |
      (0 << GPIOTE_CONFIG_PORT_Pos) |
      (GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
      (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos); // Configure GPIOTE channel 1 for P0.23 with toggle polarity

  NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos) |
                         (GPIOTE_INTENSET_IN1_Enabled << GPIOTE_INTENSET_IN1_Pos); // Enable interrupts for GPIOTE channels 0 and 1
}

void setup_QDEC()
{
  NRF_QDEC->PSEL.A = MOTOR_ENCODER_INTERRUPT_PIN_A;                                       // Set P0.21 as input for channel A
  NRF_QDEC->PSEL.B = MOTOR_ENCODER_INTERRUPT_PIN_B;                                       // Set P0.23 as input for channel B
  NRF_QDEC->PSEL.LED = (QDEC_PSEL_LED_CONNECT_Disconnected << QDEC_PSEL_LED_CONNECT_Pos); // Disable LED output
  NRF_QDEC->SAMPLEPER = (QDEC_SAMPLEPER_SAMPLEPER_128us << QDEC_SAMPLEPER_SAMPLEPER_Pos); // Set sample period to 128 us
  NRF_QDEC->ENABLE = (QDEC_ENABLE_ENABLE_Enabled << QDEC_ENABLE_ENABLE_Pos);              // Enable the QDEC peripheral
  NRF_QDEC->TASKS_START = 1;                                                              // Start the QDEC
}

void read_QDEC()
{
  // Read the accumulated count from the QDEC
  NRF_QDEC->TASKS_READCLRACC = 1;           // Trigger the READCLRACC task to read and clear the accumulated count
  int32_t acc = (int32_t)NRF_QDEC->ACCREAD; // Read the accumulated count
  motor_encoder_count += acc;               // Update the global variable with the current count
}
