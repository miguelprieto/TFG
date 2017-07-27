
int state = STATE_WAIT_FF;
int rx_count = 0;
int rx_data_ready = 0;
char rx_buffer[32];


#define AX12_Write(c) putc(c)

#define AX12_Read() getc()

#define AX12_Data_Ready() kbhit()

#define AX12_Tx_Idle() TXSTA_TRMT   // -----------------------------------------------Funzionante

void AX12_Flush(){while(AX12_Data_Ready()) AX12_Read();}


void WAIT_CHAR(unsigned char character) {
     unsigned char fine;
     fine= ~character;
     while (fine!=character)
           if (AX12_Data_Ready()) fine=AX12_Read();
}

unsigned char RECV_CHAR(){while(AX12_Data_Ready()==0); return (unsigned char) AX12_Read();}


char * timeout_str = ">TIMEOUT\n\r";

unsigned char AX12_protocol_transaction (unsigned char id, unsigned char instruction, unsigned char * parameters, unsigned char param_size)
{
  unsigned char buffer[32]; unsigned char checksum; char tried;
  int i; //Indice generico
  //unsigned char recv_id, recv_checksum;
  unsigned char recv_error,recv_len;
  int reply_size;
  tried=5;
  recv_error = 0xff;
  //Compilo lista da inviare
  buffer[0] = 0xFF;
  buffer[1] = 0xFF;
  buffer[2] = id;
  buffer[3] = param_size + 2;
  buffer[4] = instruction;
  memcpy (&buffer[5], parameters, param_size);
  //Calcolo Checksum
  checksum = 0;
  for (i = 2; i < (5 + param_size); i++) checksum += buffer [i];
  buffer[5+param_size] = checksum ^ 0xFF;  //Scrittura Checksum
  //invio il comando sulla seriale
  STATUS_FAIL = 0;
 tx_try:
  LATC_RXE = 0;
  LATC_TXE = 1;  //Predispongo il canale per la trasmissione
  for (i=0; i<=5+param_size; i++){
    while(AX12_Tx_Idle()==0);
    AX12_Write(buffer[i]);
  }
  while(AX12_Tx_Idle()==0); //Aspetto che finisce di trasmettere

  tried--;
  rx_count = 0;
  state = STATE_WAIT_FF;
  LATC_TXE = 0;
  LATC_RXE = 1;  //Predispongo il canale per la ricezione

  if (id != 0xFE) {
    AX12_TimeOut=0;
    while ((rx_data_ready == 0) && (AX12_TimeOut != 4)) ;
    if (AX12_TimeOut == 4) {
      /* char str[255]; */
      /* sprintf (str, ">TIMEOUT %d, %d, %d, %d\n\r", state, rx_count, RCSTA_FRAMING, RCSTA_OVERRUN); */
      /* print (str); */
      if (RCSTA_OVERRUN == 1) {
        RCSTA_CREN = 0;
        delay_ms(1);
        RCSTA_CREN = 1;
      }
      if (tried > 0) {
        delay_ms(105);
        goto tx_try;
      }
      else {
        STATUS_FAIL = 1;
        RE = 0;
        RegisterMap[4] = 0xff;
        return -1;
      }
    }
    recv_len = rx_buffer[1] - 2;
    recv_error = rx_buffer[2];
    RegisterMap [3] = recv_error;
    memcpy (reply, rx_buffer + 3, recv_len);
    reply_size = recv_len;
  }
  rx_data_ready = 0;
  RegisterMap[4] = 0;
  return recv_error;
}



void AX12_Interrupt()
{
  char c;
  c = AX12_Read ();
  switch (state) {

  case STATE_WAIT_FF:
    if (c == 0xff){
      rx_count ++;
      if (rx_count == 2) { // two consecutive FF
        state = STATE_WAIT_HEADER;
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
        state = STATE_WAIT_CHECKSUM;
      else
        state = STATE_WAIT_DATA;
    }
    break;

  case STATE_WAIT_DATA:
    if (rx_count < 29) {
      rx_buffer[rx_count+3] = c;
      rx_count++;
      if (rx_count == rx_buffer[1] - 2) {
        state = STATE_WAIT_CHECKSUM;
      }
    }
    break;

  case STATE_WAIT_CHECKSUM:
    if (rx_buffer[1] < 30) {
      rx_buffer [rx_buffer[1] + 1] = c;
      rx_data_ready = 1;
      state = STATE_WAIT_FF;
      rx_count = 0;
    }
    break;

  }
}
