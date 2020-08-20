#include "serial.h"
#include <string.h>

// Multi-thread: synchronized LOCK

// Definitions of Serials
SerialType hserial1, hserial2, hserial3, hserial4;


void Q_init(CircularQueue *que)
{
	que->count = 0;
	memset(que->data, 0, sizeof(que->data));
	que->front = que->data + 1;
	que->rear = que->data;
}
void Q_push(CircularQueue *que, uint8_t data)
{
	if (que->count >= QUEUE_SIZE) return ; // Cannot add into queue
	que->rear++; // offset 1
	if (que->rear >= que->data + QUEUE_SIZE) que->rear = que->data;
	*(que->rear) = data;
	que->count++;
}
void Q_pop(CircularQueue *que)
{
	if (que->count == 0) return ; // Queue is Empty
	*(que->front) = 0; // remove front data
	que->front++;
	if (que->front >= que->data + QUEUE_SIZE) que->front = que->data;
	que->count--;
}

void Serial_Init(SerialType *hserial,
		GPIO_TypeDef *RxPort, uint16_t RxPin,
		GPIO_TypeDef *TxPort, uint16_t TxPin ) {
	HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	// init pin ports
	hserial->RX.port = RxPort;
	hserial->RX.pin = RxPin;
	hserial->TX.port = TxPort;
	hserial->TX.pin = TxPin;
	// reset TX pin To HIGH
	HAL_GPIO_WritePin(hserial->TX.port, hserial->TX.pin, GPIO_PIN_SET);
	// init finite state machines
	hserial->TX_status = hserial->RX_status = 0;
	hserial->RX_bit_status = hserial->TX_bit_status = BIT_IDLE;

	// init buffers
	Q_init(&hserial->RX_buf);
	Q_init(&hserial->TX_buf);
	HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
}

// send
void Serial_write(SerialType *hserial, uint8_t data){
	// write 1 byte
	// In this function, I just need to push the 1 byte into send queue.
	// non-block logical
	HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	Q_push(&hserial->TX_buf, data);
	HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
}
void Serial_writestr(SerialType *hserial, const char *strdata){
	// loop
	HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	for (const char *cptr = strdata; *cptr != 0; cptr++)
		Q_push(&hserial->TX_buf, (uint8_t)*cptr);
	HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
}

// receive
uint8_t Serial_available(SerialType *hserial) {
	// returns the data size in Rx buffer
	return hserial->RX_buf.count;
}
uint8_t Serial_peek(SerialType *hserial){
	// do not pop the first data
	return Q_front(&hserial->RX_buf);
}
uint8_t Serial_read(SerialType *hserial){
	HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	uint8_t rd = Q_front(&hserial->RX_buf);
	Q_pop(&hserial->RX_buf);
	HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
	return rd;
}
uint16_t Serial_readall(SerialType *hserial, uint8_t *databuf) {
	HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	uint8_t *pbuf = databuf;
	while (Q_size(&hserial->RX_buf) > 0) {
		uint8_t data = Q_front(&hserial->RX_buf);
		Q_pop(&hserial->RX_buf);
		*(pbuf++) = data;
	}
	*pbuf = 0; // END OF STRING
	HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
	return pbuf - databuf; // returns the size of received-data
}

// LOW-LEVEL
// Finite State Machines of TX and RX
void _Serial_TX_bit(SerialType *hserial) {
	// HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt
	if (!hserial->TX_status) {
		// Not in Send Mode
		hserial->TX_bit_status = BIT_IDLE; // reset Bit Status
		HAL_GPIO_WritePin(hserial->TX.port, hserial->TX.pin, GPIO_PIN_SET); // reset TX Pin to HIGH
		return ;
	}
	switch (hserial->TX_bit_status) {
	case BIT_IDLE:
		// send start bit
		HAL_GPIO_WritePin(hserial->TX.port, hserial->TX.pin, GPIO_PIN_RESET);
		hserial->TX_bit_status = BIT_START; // next state
		break;
	case BIT_START: case BIT_BIT0: case BIT_BIT1: case BIT_BIT2:
	case BIT_BIT3: case BIT_BIT4: case BIT_BIT5: case BIT_BIT6:
		// send data bit
		do {
			uint8_t data_bit_i = (uint8_t)hserial->TX_bit_status - (uint8_t)BIT_START;
			HAL_GPIO_WritePin(hserial->TX.port, hserial->TX.pin,
					((hserial->TX_byte) & (1U << (data_bit_i))) ? GPIO_PIN_SET : GPIO_PIN_RESET
			);
		} while(0);
		hserial->TX_bit_status = hserial->TX_bit_status + 1; // next state
		break;
	case BIT_BIT7:
		// send Stop bit
		HAL_GPIO_WritePin(hserial->TX.port, hserial->TX.pin, GPIO_PIN_SET);
		hserial->TX_bit_status = BIT_STOP; // next state
		break;
	case BIT_STOP:
		// the stop bit has finished sent
		// reset this serial's work status
		hserial->TX_bit_status = BIT_IDLE;
		hserial->TX_status = 0; // DISABLE TX
		hserial->TX_byte = 0;
		break;
	}
	// HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
}

void _Serial_RX_bit(SerialType *hserial) {
	// HAL_TIM_Base_Stop_IT(&htim1); // LOCK TIM Interrupt

	if (!hserial->RX_status) {
		hserial->RX_bit_status = BIT_IDLE;
		return ;
	}

	switch (hserial->RX_bit_status) {
	case BIT_START:
		// ready to receive
		hserial->RX_byte = 0;
		hserial->RX_bit_status = BIT_BIT0;
		break;
	case BIT_BIT0: case BIT_BIT1: case BIT_BIT2: case BIT_BIT3:
	case BIT_BIT4: case BIT_BIT5: case BIT_BIT6: case BIT_BIT7:
		// receive the current bit
		do {
			uint8_t pinlevel = HAL_GPIO_ReadPin(hserial->RX.port, hserial->RX.pin);
			// printf("RX Bit FSM %d = Bit %d\n", hserial->RX_bit_status, pinlevel);
			uint8_t bit_i = hserial->RX_bit_status - BIT_BIT0;
			hserial->RX_byte |= (pinlevel << bit_i);
		}while (0);
		hserial->RX_bit_status = hserial->RX_bit_status + 1;
		break;
	case BIT_STOP:
		// receive finished
		Q_push(&hserial->RX_buf, hserial->RX_byte); // push the received data into buffer
		hserial->RX_byte = 0; // clear the buffered byte
		hserial->RX_status = 0; // DISABLE RX
		hserial->RX_bit_status = BIT_IDLE; // reset the bit state
		break;
	case BIT_IDLE: // Should Close RX
		hserial->RX_status = 0;
		break;
	}
	// HAL_TIM_Base_Start_IT(&htim1); // UNLOCK TIM Interrupt
}
// Enable TX: TX_buf not empty + TX_status==0
// Enable RX: Detected RX LOW + RX_status==0

void _Serial_check(SerialType *hserial) {
	// always @ (posedge clk)
	// TX check
	if (hserial->TX_status) {
		// It is now sending!
		hserial->TX_tim_counter++; // fractional(divide) frequency
		if (hserial->TX_tim_counter >= SERIAL_TIM_COUNTER_MAX) {
			hserial->TX_tim_counter = 0;
			_Serial_TX_bit(hserial);
		}
	}
	else {
		// Check TX Buffer
		if (Q_size(&hserial->TX_buf) > 0) {
			// Ready to Send
			hserial->TX_byte = Q_front(&hserial->TX_buf);
			Q_pop(&hserial->TX_buf);
			hserial->TX_tim_counter = SERIAL_TIM_COUNTER_MAX - 1;
			hserial->TX_status = 1; // ENABLE TX
		}
	}

	// RX check
	if (hserial->RX_status) {
		// It is now receiving!
		hserial->RX_tim_counter++;
		if (hserial->RX_tim_counter >= SERIAL_TIM_COUNTER_MAX) {
			hserial->RX_tim_counter = 0;
			_Serial_RX_bit(hserial);
		}
	}
	else {
		// Detect RX Voltage Level
		uint8_t rx_level = HAL_GPIO_ReadPin(hserial->RX.port, hserial->RX.pin);
		if (rx_level == GPIO_PIN_RESET) { // start bit of uart
			// ready to receive
			hserial->RX_tim_counter = SERIAL_TIM_COUNTER_MAX / 2;
			hserial->RX_byte = 0; // clear byte buffer
			hserial->RX_status = 1; // ENABLE RX
			hserial->RX_bit_status = BIT_START; // set state of FSM
		}
	}
}

// TIM IRQ Handler
void Serial_TIM_Callback(){
	// always @ (posedge clk)
	// TODO: finish this function, poll every handler of sim_uart serial.


	_Serial_check(&hserial1);
	// _Serial_check(&hserial2);
    // _Serial_check(&hserial3);
    // _Serial_check(&hserial4);
}
