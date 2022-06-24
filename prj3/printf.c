/*=================================================================
 * puts: send characters in input string to UART TX FIFO in order
 * @s: input string
 *
 * Return: return the actual string length that has been sent out
 *=================================================================
 */
int
puts(const char *s)
{
	// TODO: Add your driver code here
	// volatile unsigned int* tx_fifo = uart + 1;
	// volatile unsigned int* status = uart + 2;
	volatile unsigned int* tx_fifo = (unsigned int*)((unsigned int)uart + UART_TX_FIFO);
	volatile unsigned int* status = (unsigned int*)((unsigned int)uart + UART_STATUS);

	int i = 0;
	while (s[i] != '\0') {
		// loop when the queue is full
		while (*status & UART_TX_FIFO_FULL);

		// write s[i] to UART_TX_FIFO
		*tx_fifo = (unsigned int)s[i];

		i++;
	}
	
	return i;
}