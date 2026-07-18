#include <nrf52840.h>
#include <nrf52840_bitfields.h>
#include <stdio.h>
#include <string.h>
// #include <Arduino.h>

#define PWM_RAM_POLARITY_Pos (15UL)
#define PWM_RAM_POLARITY_FALLING_Val ((1U) << PWM_RAM_POLARITY_Pos)
#define PWM_RAM_COMPARE_Msk (0x7FFFUL)

// falling edge polarity
uint16_t pwm_width[] = {PWM_RAM_POLARITY_FALLING_Val | 0, 0, 0, 0};

void setup_PWM0();
void set_PWM0_width(uint8_t channel, uint16_t width);
void setup_GPIO_for_dir();
void set_GPIO_for_dir(int8_t dir);

#define AS5600_ADDRESS 0x36 // I2C address of AS5600
uint8_t i2c_tx_buffer[10];  // Buffer for I2C data
uint8_t i2c_rx_buffer[10];  // Buffer for I2C data
void setup_I2C();
void my_i2c_write(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read_register(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
void setup_AS5600();                   // Function to setup AS5600
void read_AS5600_raw_angle();          // Function to read the raw angle from AS5600 and update the raw_angle variable
void read_AS5600_angle_deg();          // Function to read the angle in degrees from AS5600 and update the pole_angle variable
volatile uint16_t raw_angle = 0;       // Raw angle value from AS5600
volatile uint16_t raw_angle_zero = 0;  // Raw angle value from AS5600 at zero position
volatile int32_t raw_angle_offset = 0; // Raw angle offset value from AS5600
volatile float pole_angle = 0.0;       // Angle in degrees from AS5600

#define MOTOR_ENCODER_INTERRUPT_PIN_A 21 // P0.21
#define MOTOR_ENCODER_INTERRUPT_PIN_B 23 // P0.23
void setup_QDEC();
void read_QDEC();                          // Function to read the QDEC value and update the motor_encoder_count variable
volatile int32_t motor_encoder_count = 0;  // Count of motor encoder pulses
const int32_t pulses_per_revolution = 825; // Number of pulses(steps) per revolution
volatile float motor_angle = 0.0;          // Angle in degrees from motor encoder

const int control_period_ms = 2; // Control period in milliseconds
void setup_timer3_for_control_period();
void periodic_task();            // Function to be called every control_period_ms milliseconds
volatile uint8_t timer3_cnt = 0; // Counter for Timer3 interrupts

char uart_buffer[64]; // Buffer for UART data
void setup_UART();
void my_uart_write(uint8_t *data, uint8_t length);

void setup()
{
  setup_PWM0();
  setup_GPIO_for_dir();
  setup_UART();
  setup_I2C();
  setup_AS5600();
  setup_QDEC();
  setup_timer3_for_control_period();
  NRF_P1->PIN_CNF[2] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Set P1.02 as output for LED
}

void loop()
{
  if (timer3_cnt >= 5) // Check if 5 control periods have passed (10 ms)
  {
    timer3_cnt = 0;                                                        // Reset the counter
    sprintf(uart_buffer, ">pa:%.2f,ma:%.2f\r\n", pole_angle, motor_angle); // Convert the pole angle and motor angle to a string
    my_uart_write((uint8_t *)uart_buffer, strlen(uart_buffer));            // Send the pole angle over UART
  }
  __WFI(); // Wait for interrupt
}

void periodic_task()
{
  NRF_P1->OUTSET = (1 << 2); // Turn on LED

  read_AS5600_angle_deg(); // Read the angle in degrees from AS5600 and update the pole_angle variable
  read_QDEC();             // Read the QDEC value and update the motor_encoder_count variable

  float error = pole_angle - motor_angle;                              // Calculate the error between the pole angle and motor angle
  float derivative = 0;                                                // Calculate the derivative of the error
  static float previous_error = 0;                                     // Store the previous error for derivative calculation
  derivative = error - previous_error;                                 // Calculate the derivative of the error
  previous_error = error;                                              // Update the previous error
  float integral = 0;                                                  // Calculate the integral of the error
  static float previous_integral = 0;                                  // Store the previous integral for integral calculation
  integral = previous_integral + error * (control_period_ms / 1000.0); // Calculate the integral of the error
  previous_integral = integral;                                        // Update the previous integral

  float Kp = 3.0; // Proportional gain
  float Kd = 0.0; // Derivative gain
  float Ki = 0.5; // Integral gain

  float pid_output = Kp * error + Kd * derivative + Ki * integral;

  uint32_t pwm_value = 0; // Calculate the PWM value based on the error

  if (pid_output > 0)
  {
    set_GPIO_for_dir(1);              // Set the direction of the motor
    pwm_value = (uint16_t)pid_output; // Set the PWM value based on the error
  }
  else
  {
    set_GPIO_for_dir(-1);                // Set the direction of the motor
    pwm_value = (uint16_t)(-pid_output); // Set the PWM value based on the error
  }

  pwm_value = (pwm_value > 1000) ? 1000 : pwm_value; // Limit the PWM value to the maximum value of the counter

  set_PWM0_width(0, pwm_value); // Set the PWM width for channel 0

  NRF_P1->OUTCLR = (1 << 2); // Turn off LED
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

void setup_GPIO_for_dir()
{
  NRF_P1->PIN_CNF[15] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Set P1.15 as output for direction
  NRF_P1->PIN_CNF[13] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                        (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Set P1.13 as output for direction
  NRF_P1->OUTCLR = (1 << 15) | (1 << 13);                                        // Clear P1.15 and P1.13 outputs
}

void set_GPIO_for_dir(int8_t dir)
{
  if (dir > 0)
  {
    NRF_P1->OUTSET = (1 << 15); // Set P1.15 high for forward direction
    NRF_P1->OUTCLR = (1 << 13); // Set P1.13 low for forward direction
  }
  else if (dir < 0)
  {
    NRF_P1->OUTCLR = (1 << 15); // Set P1.15 low for reverse direction
    NRF_P1->OUTSET = (1 << 13); // Set P1.13 high for reverse direction
  }
  else
  {
    NRF_P1->OUTCLR = (1 << 15) | (1 << 13); // Set both P1.15 and P1.13 low for stop
  }
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

void setup_AS5600()
{
  uint32_t angle_sum = 0;
  for (size_t i = 0; i < 10; i++)
  {
    read_AS5600_raw_angle(); // Read the raw angle from AS5600 and update the raw_angle variable
    angle_sum += raw_angle;  // Accumulate the raw angle values
  }
  raw_angle_zero = angle_sum / 10; // Calculate the average raw angle value for zero position
}

void read_AS5600_raw_angle()
{
  uint8_t tmp_raw_angle[2];
  my_i2c_read_register(AS5600_ADDRESS, 0x0C, (uint8_t *)&tmp_raw_angle, 2);      // Read the raw angle from AS5600
  raw_angle = ((tmp_raw_angle[1]) | ((uint16_t)tmp_raw_angle[0] << 8)) & 0x0FFF; // Convert from big-endian to little-endian
}

void read_AS5600_angle_deg()
{
  uint16_t prev_raw_angle = raw_angle;
  read_AS5600_raw_angle();                                             // Read the raw angle from AS5600 and update the raw_angle variable
  if (raw_angle < prev_raw_angle && prev_raw_angle - raw_angle > 2048) // Handle wrap-around from 4095 to 0
  {
    raw_angle_offset += 4096; // Adjust the zero position to account for wrap-around
  }
  else if (prev_raw_angle < raw_angle && raw_angle - prev_raw_angle > 2048) // Handle wrap-around from 0 to 4095
  {
    raw_angle_offset -= 4096; // Adjust the zero position to account for wrap-around
  }
  pole_angle = (((float)raw_angle - raw_angle_zero + raw_angle_offset) / 4096.0) * 360.0; // Convert the raw angle to degrees
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
  NRF_QDEC->TASKS_READCLRACC = 1;                                             // Trigger the READCLRACC task to read and clear the accumulated count
  int32_t acc = (int32_t)NRF_QDEC->ACCREAD;                                   // Read the accumulated count
  motor_encoder_count += acc;                                                 // Update the global variable with the current count
  motor_angle = ((float)motor_encoder_count / pulses_per_revolution) * 360.0; // Convert the count to degrees
}

void TIMER3_IRQHandler(void)
{
  if (NRF_TIMER3->EVENTS_COMPARE[0])
  {
    NRF_TIMER3->EVENTS_COMPARE[0] = 0; // Clear the compare event
    periodic_task();                   // Call the periodic task function
    timer3_cnt++;                      // Increment the timer3 counter
  }
  return;
}

void setup_timer3_for_control_period()
{
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;                             // Set timer mode
  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;              // Set bit mode to 16 bits
  NRF_TIMER3->PRESCALER = 4;                                                                   // Set prescaler to 4 (1 MHz timer frequency)
  NRF_TIMER3->CC[0] = control_period_ms * 1000;                                                // Set compare value for the control period
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos; // Enable shortcut to clear timer on compare match
  NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;       // Enable interrupt on compare match
  NVIC_ClearPendingIRQ(TIMER3_IRQn);                                                           // Clear any pending interrupts for Timer3
  NVIC_SetPriority(TIMER3_IRQn, 2);                                                            // Set priority for Timer3 interrupt
  __NVIC_SetVector(TIMER3_IRQn, (uint32_t)TIMER3_IRQHandler);                                  // Set the interrupt vector for Timer3
  NVIC_EnableIRQ(TIMER3_IRQn);                                                                 // Enable Timer3 interrupt in NVIC
  NRF_TIMER3->TASKS_START = 1;                                                                 // Start the timer
}
