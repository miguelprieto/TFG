/*
 * ax12.cpp
 */

#include "defines.h"
#include <p33FJ128MC802.h>
#include <libpic30++.h>
#include <string.h>
#include <stdio.h>
#include "ax12.h"


// TIMER 4 IS USED FOR UART TIMEOUT

#define USEC_TO_TIM4(v)   (int)(v / 1.65)

void timer4_init(void)
{

    T4CONbits.TON = 0; // disabilito il timer4

    T4CONbits.TSIDL = 0 ; //non ci fermiamo in idle mode
    T4CONbits.T32 = 0 ; //timer a 16 bit
    T4CONbits.TCS = 0; // clock interno Fosc/2
    T4CONbits.TGATE = 0 ; //disabilito la modalita' gated timer

    T4CONbits.TCKPS = 0b10; // 1:64 prescaler

    // clock increment = (77385000/2)/64, period of 1.65 microsecs (approx)

    TMR4 =  0;
    PR4 = 0xffff; // periodo_timer(s)*((Fosc)/(2*prescaler)) :abbiamo un timer a circa 5ms


    IPC6bits.T4IP = 1;	// priorita' a 1
    IEC1bits.T4IE = 0;    // disabilito Timer4 interrupt
    IFS1bits.T4IF = 0; 	// reset del dell' interrupt flag

}


void timer4_start(int usec_val)
{
    T4CONbits.TON = 0;

    TMR4 = 0;
    PR4 = USEC_TO_TIM4(usec_val);

    IFS1bits.T4IF = 0;

    T4CONbits.TON = 1;
}


bool timer4_elapsed(void)
{
    return IFS1bits.T4IF == 1;
}

// ------------------------------------------------------

Dynamixel * this_dynamixel;


#define STATE_WAIT_FF         0
#define STATE_WAIT_HEADER     1
#define STATE_WAIT_DATA       2
#define STATE_WAIT_CHECKSUM   3


int dynamixel_state = STATE_WAIT_FF;
int rx_count = 0;
int rx_data_ready = 0;
unsigned char rx_buffer[32];

#define error_field      rx_buffer[2]
#define param_field      (rx_buffer+3)


void dynamixel_rx_handler(char ch)
{
    uint8_t c;

    if (!this_dynamixel->m_rx_on)
        return;

    c = (uint8_t)ch;

    switch (dynamixel_state) {

    case STATE_WAIT_FF:
        if (c == 0xff){
            rx_count ++;
            if (rx_count == 2) { // two consecutive FF
                dynamixel_state = STATE_WAIT_HEADER;
                rx_count = 0;
            }
        }
        else
            rx_count = 0; //Pacchetto errato
        break;

    case STATE_WAIT_HEADER:
        rx_buffer[rx_count] = c;
        rx_count ++;
        if (rx_count == 3) {  // three bytes of header
            rx_count = 0;
            if (rx_buffer[1] == 2)
                dynamixel_state = STATE_WAIT_CHECKSUM;
            else
                dynamixel_state = STATE_WAIT_DATA;
        }
        break;

    case STATE_WAIT_DATA:
        if (rx_count < 29) {
            rx_buffer[rx_count+3] = c;
            rx_count++;
            if (rx_count == rx_buffer[1] - 2) {
                dynamixel_state = STATE_WAIT_CHECKSUM;
            }
        }
        break;

    case STATE_WAIT_CHECKSUM:
        if (rx_buffer[1] < 30) {
            rx_buffer [rx_buffer[1] + 1] = c;
            //rx_data_ready = 1;
            dynamixel_state = STATE_WAIT_FF;
            rx_count = 0;
            this_dynamixel->notify();
        }
        break;

    }
}



Dynamixel::Dynamixel(UART_BASE & uart)
    : m_uart(uart)
{
    this_dynamixel = this;
    m_rx_on = false;
    m_notification = false;
    timer4_init();
}

bool Dynamixel::wait()
{
    timer4_start(20000); // wait 20 milliseconds
    m_notification = false;
    while (!m_notification) {
        if (timer4_elapsed())
            return false;
    }
    return true;
}

void Dynamixel::notify()
{
    m_notification = true;
}


t_dymx_error Dynamixel::protocol_transaction(uint8_t id, uint8_t instruction, uint8_t * parameters, int param_size)
{
  uint8_t buffer[32];
  uint8_t checksum;
  int trials, i;

  trials = 5;

  // prepare request packet
  buffer[0] = 0xff;
  buffer[1] = 0xff;
  buffer[2] = id;
  buffer[3] = param_size + 2;
  buffer[4] = instruction;
  memcpy (&buffer[5], parameters, param_size);

  //compute checksum
  checksum = 0;
  for (i = 2; i < (5 + param_size); i++)
      checksum += buffer [i];
  buffer[5+param_size] = checksum ^ 0xff;

 tx_try:

  m_rx_on = false;

  for (i=0; i <= 5+param_size; i++) {
      m_uart.put_char_no_fifo(buffer[i]);
  }
  // let's add a "dummy" character to avoid problems when checksum is "ff"
  m_uart.put_char_no_fifo(0x00);

  if (id == 0xfe) { // broadcast packet, no reply
      __delay_ms(10);
      return 0;
  }

  m_rx_on = true;

  bool result = wait();

  m_rx_on = false;

#if 0
  for (int i = 0; i < 10;i++) printf("%02X ", rx_buffer[i]);
  printf("\n\r");
#endif

  __delay_ms(5);

  if (!result) {
      // printf("TIMEOUT ON SERVO %x\n\r", id);
      // for (int i = 0; i <= 5+param_size;i++) printf("%02X ", buffer[i]);
      // printf("\n\r");
      return ERR_TIMEOUT;
  }
  else
      return error_field;
}


void Dynamixel::setup(int speed, int RB_TX, int RB_RX, bool tx_open_drain, bool tx_interrupt, bool rx_interrupt)
{
    if (speed == AX12_PIC_SPEED_1M) {
        m_uart.open(AX12_PIC_SPEED_1M, RB_TX, RB_RX, tx_open_drain, tx_interrupt, rx_interrupt);
        m_uart.set_baud_rate(AX12_PIC_SPEED_1M, true);
        OSCTUNbits.TUN = 8;
    }
    else {
        m_uart.open(speed, RB_TX, RB_RX, tx_open_drain, tx_interrupt, rx_interrupt);
        m_uart.set_baud_rate(speed, false);
        OSCTUNbits.TUN = 0;
    }
    m_uart.set_rx_handler(dynamixel_rx_handler);
}


void Dynamixel::setup_uart_speed(int speed)
{
    if (speed == AX12_PIC_SPEED_1M) {
        m_uart.set_baud_rate(AX12_PIC_SPEED_1M, true); // Set High Speed mode
        OSCTUNbits.TUN = 8;
    }
    else {
        m_uart.set_baud_rate(speed, false);
        U1MODEbits.BRGH  = 0; 	  // Set Low Speed mode
        OSCTUNbits.TUN = 0;
    }
    m_uart.set_rx_handler(dynamixel_rx_handler);
}


t_dymx_error Dynamixel::reset(uint8_t id)
{
    return protocol_transaction (id, AX12_RESET, NULL, 0);
}


t_dymx_error Dynamixel::ping (uint8_t id)
{
    return protocol_transaction (id, AX12_PING, NULL, 0);
}

t_dymx_error Dynamixel::write_register_8(uint8_t ID, int reg, uint8_t value)
{
    uint8_t send[2];
    send[0] = reg;
    send[1] = value;
    return protocol_transaction(ID, AX12_WRITE, send, 0x02);
}

t_dymx_error Dynamixel::write_register_16(uint8_t ID, int reg, uint16_t value)
{
    uint8_t send[3];
    send[0] = reg;
    send[1] = value & 0xFF;
    send[2] = value >> 8;
    return protocol_transaction(ID, AX12_WRITE, send, 0x03);
}


t_dymx_error Dynamixel::read_register_8(uint8_t id, int reg, uint8_t & value)
{
    uint8_t send[2];
    send[0] = reg;
    send[1] = 0x01; // one byte to read
    t_dymx_error ret_val = protocol_transaction(id, AX12_READ, send, 0x02);
    if (ret_val == 0)
        value = param_field[0];
    return ret_val;
}


t_dymx_error Dynamixel::read_register_16(uint8_t id, int reg, uint16_t & value)
{
    uint8_t send[2];
    send[0] = reg;
    send[1] = 0x02; // two bytes to read
    t_dymx_error ret_val = protocol_transaction(id, AX12_READ, send, 0x02);
    if (ret_val == 0)
        value = param_field[0] | (param_field[1] << 8);
    return ret_val;
}


// ------------------------------------------------------------------------

t_dymx_error Dynamixel::set_new_speed(uint8_t id, int new_pic_speed)
{
    int value;
    switch (new_pic_speed) {
    case AX12_PIC_SPEED_1M:      value = AX12_LOCALSPEED_1M; break;
    case AX12_PIC_SPEED_57600:   value = AX12_LOCALSPEED_57600; break;
    case AX12_PIC_SPEED_58K:     value = AX12_LOCALSPEED_58K; break;
    case AX12_PIC_SPEED_20408:   value = AX12_LOCALSPEED_20408; break;
    case AX12_PIC_SPEED_13071:   value = AX12_LOCALSPEED_13071; break;
    default: return ERR_BAD_PARAM; // FIXME! return a proper error value;
    }
    t_dymx_error err = write_register_8(id, AX12_BAUD_RATE, value);
    if (err == 0)
        setup_uart_speed(new_pic_speed);

    return err;
}


t_dymx_error Dynamixel::set_servo_led(uint8_t ID, bool state)
{
    return write_register_8(ID, AX12_LED, state);
}

t_dymx_error Dynamixel::set_new_id(uint8_t ID, uint8_t new_id)
{
    return write_register_8(ID, AX12_ID, new_id);
}

t_dymx_error Dynamixel::move(uint8_t ID, int position)
{
    return write_register_16(ID, AX12_GOAL_POSITION_L, position);
}

t_dymx_error Dynamixel::degrees(uint8_t ID, float position)
{
    if (position < -150 || position > 150)
        return ERR_BAD_PARAM; // fixme use a proper value
    else {
        int val = position * 0x200 / 150 + 0x1ff;
        return move(ID, val);
    }
}


t_dymx_error Dynamixel::torque(uint8_t ID, bool state)
{
    return write_register_8(ID, AX12_TORQUE_ENABLE, state);
}


t_dymx_error Dynamixel::current_position(uint8_t id, int & position)
{
    uint16_t p;
    t_dymx_error ret_val = read_register_16(id, AX12_PRESENT_POSITION_L, p);
    if (ret_val == 0)
        position = p;
    return ret_val;
}


t_dymx_error Dynamixel::current_angle(uint8_t id, float & position)
{
    uint16_t p;
    t_dymx_error ret_val = read_register_16(id, AX12_PRESENT_POSITION_L, p);
    if (ret_val == 0)
        position = ((float)p - 0x1ff) * 150.0 / 0x200;
    return ret_val;
}

