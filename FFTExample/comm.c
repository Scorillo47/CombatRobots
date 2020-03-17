/*
 * comm.c - Communications functions for USARTs
 */

#include <comm.h>
#include <main.h>
#include <periph.h>

// USART 2
static RingBuffer_TypeDef rx;
static RingBuffer_TypeDef tx;

// Removes a byte from the head of the given buffer
char ringPullByte(volatile RingBuffer_TypeDef* buffer) {
	uint16_t head = buffer->head; char value;
	value = buffer->buffer[head];
	buffer->head = (head + 1) & (USART_BUFFER_SIZE - 1);
	return value;
}

// Queues a byte onto the tail of the given buffer
void ringQueueByte(volatile RingBuffer_TypeDef* buffer, char value) {
	uint16_t tail = buffer->tail;
	buffer->buffer[tail] = value;
	tail = (tail + 1) & (USART_BUFFER_SIZE - 1);
	buffer->tail = tail;
}

// Reads a byte from the serial port
char serialReadByte() {
	while (ringIsBufferEmpty(&rx)) __WFI();
	return ringPullByte(&rx);
}

// Returns true if a character is available on the serial input
bool serialReadReady() {
	return !ringIsBufferEmpty(&rx);
}

// Writes a byte to the serial port
void serialWriteByte(const char c) {
	while (ringIsBufferFull(&tx)) __WFI();
	ringQueueByte(&tx, c);
	USART3->CR1 |= USART_CR1_TXEIE;
}

// Buffered character I/O handler for UART port 1
ISR_Handler USART3_IRQHandler() {
	char value;
	if (USART3->ISR & USART_ISR_RXNE) {
		// Read to clear the flag
		value = (char)USART3->RDR;
		if (!ringIsBufferFull(&rx))
			// Buffer it up, if it's not full to the brim
			ringQueueByte(&rx, value);
	}
	if (USART3->ISR & USART_ISR_TXE) {
		if (ringIsBufferEmpty(&tx))
			// Nothing to send, disable interrupt
			USART3->CR1 &= ~USART_CR1_TXEIE;
		else {
			value = ringPullByte(&tx);
			USART3->TDR = value;
		}
	}
}
