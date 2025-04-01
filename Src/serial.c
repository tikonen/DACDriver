
#include "main.h"
#include "printf.h"

#define UART_RINGBUFFER_SIZE 512

typedef struct {
	volatile uint32_t ridx;
	volatile uint32_t widx;
	uint8_t buffer[UART_RINGBUFFER_SIZE];
} RingByteBuffer;

static RingByteBuffer sUartBufferTX;
static RingByteBuffer sUartBufferRX;

static inline uint32_t rb_avail(const RingByteBuffer *buffer)
{
	return buffer->widx - buffer->ridx;
}

extern UART_HandleTypeDef huart2;

#define enter_CRITICAL() __disable_irq()
#define exit_CRITICAL()  __enable_irq()

static void uart_transmit_pending(UART_HandleTypeDef *huart)
{
	if(huart->gState == HAL_UART_STATE_READY) {
		uint32_t sidx = sUartBufferTX.ridx % UART_RINGBUFFER_SIZE;
		uint32_t eidx = sUartBufferTX.widx % UART_RINGBUFFER_SIZE;
		uint32_t len = eidx < sidx ? UART_RINGBUFFER_SIZE - sidx : eidx - sidx;
		if(len) {
			sUartBufferTX.ridx += len;
			HAL_UART_Transmit_DMA(huart, sUartBufferTX.buffer + sidx, len);
			//HAL_UART_Transmit_IT(&huart2, sUartBufferTX.buffer + sidx, len);
		}
	}
}

static void _putchar_unsafe(char c, void *arg)
{
	(void)arg;
	sUartBufferTX.buffer[sUartBufferTX.widx++ % UART_RINGBUFFER_SIZE] = c;
}

void serial_printfln_ts(const char *format, ...)
{
	va_list arglist;
	const uint32_t ts = HAL_GetTick();
	const uint32_t sec = ts / 1000;
	const uint32_t ms = ts - sec * 1000;

	enter_CRITICAL();
	fctprintf(_putchar_unsafe, NULL, "[% 3u.%03u] ", sec, ms);
	va_start(arglist, format);
	vfctprintf(_putchar_unsafe, NULL, format, arglist);
	va_end(arglist);
	_putchar_unsafe('\n', NULL);
	uart_transmit_pending(&huart2);
	exit_CRITICAL();
}

// for printf
void _putchar(char c) {

	// buffer is allocated in critical section to allow multiple threads to print to uart concurrently
	enter_CRITICAL();
	sUartBufferTX.buffer[sUartBufferTX.widx++ % UART_RINGBUFFER_SIZE] = c;
	if(c == '\n') uart_transmit_pending(&huart2);
	exit_CRITICAL();
}

void serial_writec(char c)
{
	// buffer is allocated in critical section to allow multiple threads to print to uart concurrently
	enter_CRITICAL();
	sUartBufferTX.buffer[sUartBufferTX.widx++ % UART_RINGBUFFER_SIZE] = c;
	uart_transmit_pending(&huart2);
	exit_CRITICAL();
}

void serial_write(const char *str)
{
	// buffer is allocated in critical section to allow multiple threads to print to uart concurrently.
	enter_CRITICAL();
	for(int i=0; str[i] ; i++) {
		sUartBufferTX.buffer[sUartBufferTX.widx++ % UART_RINGBUFFER_SIZE] = str[i];
	}
	uart_transmit_pending(&huart2);
	exit_CRITICAL();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rb_avail(&sUartBufferTX)) {
		uart_transmit_pending(huart);
	}
}

int serial_available()
{
	return rb_avail(&sUartBufferRX);
}

char serial_read_char()
{
	return sUartBufferRX.buffer[sUartBufferRX.ridx++ % UART_RINGBUFFER_SIZE];
}

void start_receive()
{
	HAL_UART_Receive_IT(&huart2, &sUartBufferRX.buffer[sUartBufferRX.widx % UART_RINGBUFFER_SIZE], 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &sUartBufferRX.buffer[++sUartBufferRX.widx % UART_RINGBUFFER_SIZE], 1);
}
