#include <nrf52840.h>
#include <nrf52840_bitfields.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
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
const uint16_t PWM_MIN_WIDTH = 80;   // Minimum PWM width (8% duty cycle)
const uint16_t PWM_MAX_WIDTH = 1000; // Maximum PWM width (100% duty cycle)

#define AS5600_ADDRESS 0x36 // I2C address of AS5600
uint8_t i2c_tx_buffer[10];  // Buffer for I2C data
uint8_t i2c_rx_buffer[10];  // Buffer for I2C data
void setup_I2C();
void my_i2c_write(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read(uint8_t address, uint8_t *data, uint8_t length);
void my_i2c_read_register(uint8_t address, uint8_t reg, uint8_t *data, uint8_t length);
void setup_AS5600();                   // Function to setup AS5600
void read_AS5600_raw_angle();          // Function to read the raw angle from AS5600 and update the raw_angle variable
void read_AS5600_angle();              // Function to read the angle in degrees from AS5600 and update the pole_angle variable
volatile uint16_t raw_angle = 0;       // Raw angle value from AS5600
volatile uint16_t raw_angle_zero = 0;  // Raw angle value from AS5600 at zero position
volatile int32_t raw_angle_offset = 0; // Raw angle offset value from AS5600
volatile float pole_angle = 0.0;       // Angle in Radians from AS5600
volatile float pole_angle_d = 0.0;     // Angle velocity in radians per second from AS5600

#define MOTOR_ENCODER_INTERRUPT_PIN_A 21 // P0.21
#define MOTOR_ENCODER_INTERRUPT_PIN_B 23 // P0.23
void setup_QDEC();
void read_QDEC();                                 // Function to read the QDEC value and update the motor_encoder_count variable
volatile int32_t motor_encoder_count = 0;         // Count of motor encoder pulses
const int32_t pulses_per_revolution = 825;        // Number of pulses(steps) per revolution
volatile float motor_angle = 0.0;                 // Angle in radians from motor encoder
volatile float motor_angle_d = 0.0;               // Angle velocity in radians per second from motor encoder
const float MOTOR_ANGLE_MIN = -30 * M_PI / 180.0; // Minimum angle in radians from motor encoder (-30 degrees)
const float MOTOR_ANGLE_MAX = 30 * M_PI / 180.0;  // Maximum angle in radians from motor encoder (30 degrees)

const int control_period_ms = 2; // Control period in milliseconds
void setup_timer3_for_control_period();
void periodic_task();            // Function to be called every control_period_ms milliseconds
volatile uint8_t timer3_cnt = 0; // Counter for Timer3 interrupts
volatile float pid_output = 0.0; // PID output variable
volatile float p_term = 0.0;     // Proportional term variable
volatile float i_term = 0.0;     // Integral term variable
volatile float d_term = 0.0;     // Derivative term variable

char uart_buffer[64]; // Buffer for UART data
void setup_UART();
void my_uart_write(uint8_t *data, uint8_t length);

#define BUTTON_PIN 27 // P0.27
void setup_GPIO_interrupt_for_button();

void setup_ADC();
void read_ADC();
#define ADC0_PIN 2                   // P0.04
#define ADC1_PIN 3                   // P0.05
volatile int16_t adc_value[2] = {0}; // ADC values from the potentiometer

typedef enum _State
{
  STATE_INIT = 0,
  STATE_MOVE_TO_NEGATIVE_ANGLE = 1,
  STATE_MOVE_TO_POSITIVE_ANGLE = 2,
  STATE_MOVE_TO_ZERO_ANGLE = 3,
  STATE_STOPPED = 4,
  STATE_SWINGING = 5,
  STATE_CONTROL = 6
} State;
volatile State state = 0;          // State variable for the main loop
volatile float target_angle = 0.0; // Target angle in degrees (upright position)

volatile float Kp_control = 1400.0; // Proportional gain for control
volatile float Kd_control = 50.0;   // Derivative gain for control

void setup()
{
  setup_PWM0();
  setup_GPIO_for_dir();
  setup_UART();
  setup_I2C();
  setup_AS5600();
  setup_QDEC();
  setup_timer3_for_control_period();
  setup_GPIO_interrupt_for_button();
  setup_ADC();
  NRF_P1->PIN_CNF[2] = (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Set P1.02 as output for LED

  // my_uart_write((uint8_t *)"pa,pad,ma,mad,mt,pid_output\r\n", strlen("pa,pad,ma,mad,mt,pid_output\r\n")); // Send initialization message over UART
}

void loop()
{
  if (timer3_cnt >= 5) // Check if 5 control periods have passed (10 ms)
  {
    timer3_cnt = 0; // Reset the counter
    // sprintf(uart_buffer, ">pa:%.2f,pad:%.2f,ma:%.2f,mad:%.2f,mt:%.2f,pid_output:%.2f\r\n",
    //         pole_angle, pole_angle_d, motor_angle, motor_angle_d, target_angle, pid_output); // Convert the pole angle and motor angle to a string
    sprintf(uart_buffer, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
            pole_angle * 180.0 / M_PI, pole_angle_d * 180.0 / M_PI, motor_angle * 180.0 / M_PI, motor_angle_d * 180.0 / M_PI, target_angle, pid_output, p_term, d_term, Kp_control, Kd_control); // Convert the pole angle and motor angle to a string
    my_uart_write((uint8_t *)uart_buffer, strlen(uart_buffer));                                                                                                                                  // Send the pole angle over UART
  }
  __WFI(); // Wait for interrupt
}

void periodic_task()
{
  NRF_P1->OUTSET = (1 << 2); // Turn on LED

  read_AS5600_angle(); // Read the angle in degrees from AS5600 and update the pole_angle variable
  read_QDEC();         // Read the QDEC value and update the motor_encoder_count variable
  read_ADC();          // Read the ADC values and update the adc_value array

  Kp_control = 0.9 * Kp_control + 0.1 * (float)adc_value[0] / 4096.0 * 3000.0; // Update the proportional gain based on the potentiometer value
  Kd_control = 0.9 * Kd_control + 0.1 * (float)adc_value[1] / 4096.0 * 500.0;  // Update the derivative gain based on the potentiometer value

  if (state == STATE_INIT)
  {
    // static uint32_t cnt = 0;
    // if (cnt == 10000)
    // {
    //   state = 10; // Change state to 10 when the button is pressed
    // }
    // cnt++;
    // set_GPIO_for_dir(1); // Stop the motor
    // set_PWM0_width(0, cnt / 100);

    // state = STATE_MOVE_TO_POSITIVE_ANGLE; // Change state to MOVE_TO_POSITIVE_ANGLE when the button is pressed

    state = STATE_CONTROL; // Change state to CONTROL when the button is pressed
  }
  else if (state == STATE_MOVE_TO_POSITIVE_ANGLE)
  {
    target_angle = MOTOR_ANGLE_MAX; // Set the target angle to the maximum motor angle
    if (motor_angle > MOTOR_ANGLE_MAX - 0.1)
    {
      state = STATE_MOVE_TO_ZERO_ANGLE; // Change state to MOVE_TO_ZERO_ANGLE when the motor angle is greater than the maximum angle
    }
  }
  else if (state == STATE_MOVE_TO_NEGATIVE_ANGLE)
  {
    target_angle = MOTOR_ANGLE_MIN; // Set the target angle to the minimum motor angle
    if (motor_angle < MOTOR_ANGLE_MIN + 0.1)
    {
      state = STATE_MOVE_TO_ZERO_ANGLE; // Change state to MOVE_TO_ZERO_ANGLE when the motor angle is less than the minimum angle
    }
  }
  else if (state == STATE_MOVE_TO_ZERO_ANGLE)
  {
    target_angle = 0.0; // Set the target angle to 0 degrees
  }
  else if (state == STATE_STOPPED)
  {
    p_term = 0.0;
    i_term = 0.0;
    d_term = 0.0;
    pid_output = 0.0;
    set_GPIO_for_dir(0); // Stop the motor
    set_PWM0_width(0, 0);
  }
  else if (state == STATE_SWINGING)
  {
    static uint16_t cnt = 0;

    if ((cnt & 1) == 0)
    {
      target_angle = MOTOR_ANGLE_MAX;                                         // Set the target angle to the maximum motor angle
      if (motor_angle > MOTOR_ANGLE_MAX - 0.1 && pole_angle > M_PI / 6 * cnt) //
      {
        cnt += 1; // Increment the counter when the motor angle is greater than the maximum angle
      }
    }
    else if ((cnt & 1) == 1)
    {
      target_angle = MOTOR_ANGLE_MIN; // Set the target angle to the minimum motor angle
      if (motor_angle < MOTOR_ANGLE_MIN + 0.1 && pole_angle < -M_PI / 6 * cnt)
      {
        cnt += 1; // Increment the counter when the motor angle is greater than the maximum angle
      }
    }
    // else if (cnt == 2)
    // {
    //   target_angle = 0.0; // Set the target angle to 0 degrees
    if (fabs(pole_angle - M_PI) < M_PI / 3)
    {
      state = STATE_CONTROL; // Change state to CONTROL when the pole angle is close to 0 degrees
      cnt = 0;               // Reset the counter
    }
    // }

    float error = target_angle - motor_angle; // Calculate the error between the motor angle and target angle

    float Kp = 250.0; // Proportional gain

    pid_output = Kp * error; // Calculate the PID output
  }
  else if (state == STATE_CONTROL)
  {
    target_angle = M_PI;                     // Set the target angle to 180 degrees (pi radians)
    float error = target_angle - pole_angle; // Calculate the error between the pole angle and target angle
    float derivative = 0;                    // Calculate the derivative of the error
    static float integral = 0;               // Calculate the integral of the error

    derivative = -pole_angle_d;
    integral += error * (control_period_ms / 1000.0); // Calculate the integral of the error

    float Kp = Kp_control; // Proportional gain
    float Kd = Kd_control; // Derivative gain
    float Ki = 0.0;        // Integral gain

    p_term = Kp * error;      // Calculate the proportional term
    d_term = Kd * derivative; // Calculate the derivative term
    i_term = Ki * integral;   // Calculate the integral term

    pid_output = p_term + d_term + i_term; // Calculate the PID output

    if (motor_angle < MOTOR_ANGLE_MIN || motor_angle > MOTOR_ANGLE_MAX ||
        pole_angle < M_PI / 2 || pole_angle > 3 * M_PI / 2) // Check if the motor angle is out of bounds
    {
      target_angle = 0.0;    // Reset the target angle to 0 degrees when the motor angle is out of bounds
      pid_output = 0;        // Stop the motor when the motor angle is out of bounds
      state = STATE_STOPPED; // Change state to STOPPED when the motor angle is out of bounds
    }

    if (fabs(error) < M_PI / 180.0) // Check if the error is less than 1 degrees
    {
      pid_output = 0; // Stop the motor when the error is less than 1 degrees
    }
  }
  else
  {
    set_GPIO_for_dir(0); // Stop the motor
    set_PWM0_width(0, 0);
  }

  // if (state == STATE_MOVE_TO_POSITIVE_ANGLE || state == STATE_MOVE_TO_NEGATIVE_ANGLE || state == STATE_MOVE_TO_ZERO_ANGLE || state == STATE_CONTROL)
  {
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

    pwm_value = (pwm_value > PWM_MAX_WIDTH) ? PWM_MAX_WIDTH : pwm_value;              // Limit the PWM value to the maximum value of the counter
    pwm_value = (pwm_value && pwm_value < PWM_MIN_WIDTH) ? PWM_MIN_WIDTH : pwm_value; // Limit the PWM value to the minimum value of the counter

    set_PWM0_width(0, pwm_value); // Set the PWM width for channel 0
  }

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
  NRF_PWM0->COUNTERTOP = PWM_MAX_WIDTH;                                               // Set the period of the PWM signal, which is the maximum value of the counter. This will give a PWM frequency of 16 kHz (16 MHz / 1000).
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
    if (width > PWM_MAX_WIDTH)
    {
      width = PWM_MAX_WIDTH; // Limit the width to the maximum value of the counter
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
  for (size_t i = 0; i < 100; i++)
  {
    read_AS5600_raw_angle(); // Read the raw angle from AS5600 and update the raw_angle variable
    angle_sum += raw_angle;  // Accumulate the raw angle values
  }
  raw_angle_zero = angle_sum / 100; // Calculate the average raw angle value for zero position
}

void read_AS5600_raw_angle()
{
  uint8_t tmp_raw_angle[2];
  my_i2c_read_register(AS5600_ADDRESS, 0x0C, (uint8_t *)&tmp_raw_angle, 2);      // Read the raw angle from AS5600
  raw_angle = ((tmp_raw_angle[1]) | ((uint16_t)tmp_raw_angle[0] << 8)) & 0x0FFF; // Convert from big-endian to little-endian
}

void read_AS5600_angle()
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

  static float prev_pole_angle_arr[5] = {0};               // Store the previous pole angle for velocity calculation
  static int prev_index = 0;                               // Store the previous index for velocity calculation
  float prev_pole_angle = prev_pole_angle_arr[prev_index]; // Store the previous pole angle for velocity calculation
  float prev_pole_angle_d = pole_angle_d;                  // Get the previous pole angle velocity from the array
  float alpha = 0.8;                                       // Low-pass filter coefficient for angle velocity

  pole_angle = (((float)raw_angle - raw_angle_zero + raw_angle_offset) / 4096.0) * 2.0 * M_PI;                                    // Convert the raw angle to radians
  pole_angle_d = (1.0 - alpha) * (pole_angle - prev_pole_angle) / (control_period_ms * 5.0 / 1000.0) + alpha * prev_pole_angle_d; // Apply low-pass filter to angle velocity

  prev_pole_angle_arr[prev_index] = pole_angle; // Store the current pole angle velocity in the array
  prev_index = (prev_index + 1) % 5;            // Update the index for the next velocity calculation
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

  static float prev_motor_angle_arr[5] = {0};                // Store the previous motor angle for velocity calculation
  static int prev_index = 0;                                 // Store the previous index for velocity calculation
  float prev_motor_angle = prev_motor_angle_arr[prev_index]; // Store the previous motor angle for velocity calculation
  float prev_motor_angle_d = motor_angle_d;                  // Get the previous motor angle velocity from the array
  float alpha = 0.8;                                         // Low-pass filter coefficient for angle velocity

  motor_angle = ((float)motor_encoder_count / pulses_per_revolution) * 2.0 * M_PI;                                                    // Convert the count to radians
  motor_angle_d = (1.0 - alpha) * (motor_angle - prev_motor_angle) / (control_period_ms * 5.0 / 1000.0) + alpha * prev_motor_angle_d; // Apply low-pass filter to angle velocity

  prev_motor_angle_arr[prev_index] = motor_angle; // Store the current motor angle velocity in the array
  prev_index = (prev_index + 1) % 5;              // Update the index for the next velocity calculation
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
  NVIC_SetPriority(TIMER3_IRQn, 1);                                                            // Set priority for Timer3 interrupt
  __NVIC_SetVector(TIMER3_IRQn, (uint32_t)TIMER3_IRQHandler);                                  // Set the interrupt vector for Timer3
  NVIC_EnableIRQ(TIMER3_IRQn);                                                                 // Enable Timer3 interrupt in NVIC
  NRF_TIMER3->TASKS_START = 1;                                                                 // Start the timer
}

void my_GPIOTE_IRQHandler(void)
{
  if (NRF_GPIOTE->EVENTS_IN[0])
  {
    NRF_GPIOTE->EVENTS_IN[0] = 0; // Clear the event
    state = STATE_INIT;           // Reset the state to INIT when the button is pressed
  }
}

void setup_GPIO_interrupt_for_button()
{
  NRF_P0->PIN_CNF[BUTTON_PIN] =
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos); // Configure P0.27 as input with pull-up and sense for low level
  NRF_GPIOTE->CONFIG[0] =
      (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) |
      (BUTTON_PIN << GPIOTE_CONFIG_PSEL_Pos) |
      (0 << GPIOTE_CONFIG_PORT_Pos) |
      (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
      (GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);                    // Configure GPIOTE channel 0 for P0.27 with HiToLo polarity (falling edge)
  NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos); // Enable interrupt for GPIOTE channel 0
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);                                               // Clear any pending interrupts for GPIOTE
  NVIC_SetPriority(GPIOTE_IRQn, 2);                                                // Set priority for GPIOTE interrupt
  __NVIC_SetVector(GPIOTE_IRQn, (uint32_t)my_GPIOTE_IRQHandler);                   // Set the interrupt vector for GPIOTE
  NVIC_EnableIRQ(GPIOTE_IRQn);                                                     // Enable GPIOTE interrupt in NVIC
}

void setup_ADC()
{
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit << SAADC_RESOLUTION_VAL_Pos;                // Set resolution to 12 bits
  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass << SAADC_OVERSAMPLE_OVERSAMPLE_Pos; // Set oversample to bypass
  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task << SAADC_SAMPLERATE_MODE_Pos;               // Set sample rate to task mode

  NRF_SAADC->RESULT.PTR = (uint32_t)adc_value;                         // Set the pointer to the ADC result buffer
  NRF_SAADC->RESULT.MAXCNT = sizeof(adc_value) / sizeof(adc_value[0]); // Set the maximum number of samples to read

  // Configure channel 0 for ADC0_PIN (P0.04)
  NRF_P0->PIN_CNF[4] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Configure ADC0_PIN as input
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos; // No negative input
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput2;                   // Set positive input to AIN2 (P0.04)
  NRF_SAADC->CH[0].CONFIG = (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
                            (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
                            (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                            (SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos) |
                            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                            (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos); // Configure channel 0 settings

  // Configure channel 1 for ADC1_PIN (P0.05)
  NRF_P0->PIN_CNF[5] = (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
                       (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
                       (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
                       (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
                       (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos); // Configure ADC1_PIN as input
  NRF_SAADC->CH[1].PSELN = SAADC_CH_PSELN_PSELN_NC << SAADC_CH_PSELN_PSELN_Pos; // No negative input
  NRF_SAADC->CH[1].PSELP = SAADC_CH_PSELP_PSELP_AnalogInput3;                   // Set positive input to AIN3 (P0.05)
  NRF_SAADC->CH[1].CONFIG = (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
                            (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
                            (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
                            (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
                            (SAADC_CH_CONFIG_TACQ_40us << SAADC_CH_CONFIG_TACQ_Pos) |
                            (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
                            (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos); // Configure channel 1 settings

  NRF_SAADC->ENABLE = (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos); // Enable the SAADC

  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1; // Start offset calibration
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0)
    ;                                  // Wait for calibration to complete
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0; // Clear the calibration done event
}

void read_ADC()
{
  NRF_SAADC->TASKS_START = 1; // Start the ADC
  while (NRF_SAADC->EVENTS_STARTED == 0)
    ;                            // Wait for the ADC to start
  NRF_SAADC->EVENTS_STARTED = 0; // Clear the started event

  NRF_SAADC->TASKS_SAMPLE = 1; // Trigger a sample
  // while (NRF_SAADC->EVENTS_RESULTDONE == 0)
  //   ;                               // Wait for the sample to complete
  // NRF_SAADC->EVENTS_RESULTDONE = 0; // Clear the result done event
  // NRF_SAADC->TASKS_SAMPLE = 1;      // Trigger a sample
  while (NRF_SAADC->EVENTS_END == 0)
    ;                        // Wait for the ADC to start
  NRF_SAADC->EVENTS_END = 0; // Clear the end event

  NRF_SAADC->TASKS_STOP = 1; // Stop the ADC
  while (NRF_SAADC->EVENTS_STOPPED == 0)
    ;                            // Wait for the ADC to stop
  NRF_SAADC->EVENTS_STOPPED = 0; // Clear the stopped event
}
