void AX12_Write(unsigned char byte_);
unsigned char AX12_Read();
unsigned AX12_Data_Ready();
char AX12_Tx_Idle();
char AX12_Tx_Busy();
unsigned char RECV_CHAR();
void WAIT_CHAR(unsigned char character);
unsigned char AX12_protocol_transaction (unsigned char id, unsigned char instruction, unsigned char * parameters, unsigned char param_size);
void AX12_Flush();
void AX12_Interrupt();
