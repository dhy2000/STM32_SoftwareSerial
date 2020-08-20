#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

/* --- Includes --- */
#include "main.h"
#include <stdint.h>

/* --- Definitions of Structures and Typedefs --- */
/* -- Pins -- */
typedef struct SerialPinType {
	GPIO_TypeDef * port;
	uint16_t pin;
}SerialPinType; // RX, TX

/* -- Buffers -- */
#define BUFFER_SIZE 64
// Circular Queue
#define QUEUE_SIZE BUFFER_SIZE
typedef struct CircularQueue {
	// Loop-Queue
	uint8_t data[QUEUE_SIZE];
	uint8_t *front, *rear;
	uint16_t count;
}CircularQueue;
void Q_init(CircularQueue *que);
void Q_push(CircularQueue *que, uint8_t data);
void Q_pop(CircularQueue *que);
#define Q_front(_que) (*((_que)->front))
#define Q_size(_que) ((_que)->count)
// Buffer Type
typedef CircularQueue BufferType;

/* -- State FSM -- */

typedef enum SerialBitStatus {
	BIT_IDLE = 0,
	BIT_START,
	BIT_BIT0,
	BIT_BIT1,
	BIT_BIT2,
	BIT_BIT3,
	BIT_BIT4,
	BIT_BIT5,
	BIT_BIT6,
	BIT_BIT7,
	BIT_STOP
}SerialBitStatus;

/* -- Serial Structure Define -- */
typedef struct SerialType {
	// Full-Duplex Serial
	SerialPinType RX, TX; // Pin Define
	BufferType RX_buf, TX_buf; // Buffer Define
	uint8_t RX_status, TX_status; // Work Status : 1=work, 0=idle
	SerialBitStatus RX_bit_status, TX_bit_status; // Bit(T/R) Status
	// bit send/ bit recv buff
	uint8_t TX_byte, RX_byte;
	// timer counter
	uint8_t RX_tim_counter, TX_tim_counter;
}SerialType;

/* Methods */
// initialize
void Serial_Init(SerialType *hserial,
		GPIO_TypeDef *RxPort, uint16_t RxPin,
		GPIO_TypeDef *TxPort, uint16_t TxPin
);
// send
void Serial_write(SerialType *hserial, uint8_t data); // write 1 byte
void Serial_writestr(SerialType *hserial, const char *strdata);
// receive
uint8_t Serial_available(SerialType *hserial) ; // returns 1 or 0
uint8_t Serial_peek(SerialType *hserial);
uint8_t Serial_read(SerialType *hserial);
uint16_t Serial_readall(SerialType *hserial, uint8_t *databuf) ;


/* --- LOW Level Part --- */
#define SERIAL_TIM_COUNTER_MAX 10

/* --- TIM Interrupt Callbacks --- */
void Serial_TIM_Callback(); // add this function into TIM1_Up_IRQHandler.



/* --- Serial Instances --- */
extern SerialType hserial1, hserial2, hserial3, hserial4;

/* --- BAUD RATE ---- */
#define BAUD_9600_DELAY_US 104
#define BAUD_9600_TIM_PERIOD 333 // htim1.Init.Period = 333;

#endif /* INC_SERIAL_H_ */
