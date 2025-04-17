#include <stdint.h>

// ==============================
// Base Address Definitions
// ==============================

#define PERIPH_BASE             (0x40000000UL)                      // Base address for all peripherals
#define AHB1PERIPH_OFFSET       (0x00020000UL)
#define AHB1PERIPH_BASE         (PERIPH_BASE + AHB1PERIPH_OFFSET)

#define APB1PERIPH_OFFSET       (0x00000000UL)
#define APB1PERIPH_BASE         (PERIPH_BASE + APB1PERIPH_OFFSET)

#define GPIOA_OFFSET            (0x0000UL)
#define GPIOB_OFFSET            (0x0400UL)
#define RCC_OFFSET              (0x3800UL)
#define USART2_OFFSET           (0x4400UL)
#define USART3_OFFSET           (0x4800UL)

#define GPIOA_BASE              (AHB1PERIPH_BASE + GPIOA_OFFSET)
#define GPIOB_BASE              (AHB1PERIPH_BASE + GPIOB_OFFSET)
#define RCC_BASE                (AHB1PERIPH_BASE + RCC_OFFSET)
#define USART2_BASE             (APB1PERIPH_BASE + USART2_OFFSET)
#define USART3_BASE             (APB1PERIPH_BASE + USART3_OFFSET)

// Clock enable bits
#define GPIOAEN                 (1U << 0)                           // Bit 0 enables clock to GPIOA
#define GPIOBEN                 (1U << 1)                           // Bit 1 enables clock to GPIOB
#define USART2EN                (1U << 17)                          // Bit 17 enables clock to USART2
#define USART3EN                (1U << 18)                          // Bit 18 enables clock to USART3

#define SYS_FREQ                16000000                            // System frequency = 16 MHz
#define APB1_CLK                SYS_FREQ                            // APB1 clock assumed same as system
#define UART_BAUDRATE           115200                              // Desired baud rate for UART

// ==============================
// Typedef Struct Definitions
// ==============================

#define __IO volatile

// GPIO register structure
typedef struct {
	__IO uint32_t MODER;         // Mode register
	__IO uint32_t OTYPER;        // Output type register
	__IO uint32_t OSPEEDR;       // Output speed register
	__IO uint32_t PUPDR;         // Pull-up/pull-down register
	__IO uint32_t IDR;           // Input data register
	__IO uint32_t ODR;           // Output data register
	__IO uint32_t BSRR;          // Bit set/reset register
	__IO uint32_t LCKR;          // Configuration lock register
	__IO uint32_t AFRL;          // Alternate function low register (pins 0–7)
	__IO uint32_t AFRH;          // Alternate function high register (pins 8–15)
} GPIO_TypeDef;

// RCC register structure
typedef struct {
	__IO uint32_t CR;
	__IO uint32_t PLLCFGR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t AHB1RSTR;
	__IO uint32_t AHB2RSTR;
	__IO uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__IO uint32_t APB1RSTR;
	__IO uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__IO uint32_t AHB1ENR;
	__IO uint32_t AHB2ENR;
	__IO uint32_t AHB3ENR;
	uint32_t RESERVED2;
	__IO uint32_t APB1ENR;
	__IO uint32_t APB2ENR;
	uint32_t RESERVED3[2];
	__IO uint32_t AHB1LPENR;
	__IO uint32_t AHB2LPENR;
	__IO uint32_t AHB3LPENR;
	uint32_t RESERVED4;
	__IO uint32_t APB1LPENR;
	__IO uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	uint32_t RESERVED6[2];
	__IO uint32_t SSCGR;
	__IO uint32_t PLLI2SCFGR;
	__IO uint32_t PLLSAICFGR;
	__IO uint32_t DCKCFGR;
} RCC_TypeDef;

// USART register structure
typedef struct {
	__IO uint32_t USART_SR;      // Status register
	__IO uint32_t USART_DR;      // Data register
	__IO uint32_t USART_BRR;     // Baud rate register
	__IO uint32_t USART_CR1;     // Control register 1
	__IO uint32_t USART_CR2;
	__IO uint32_t USART_CR3;
	__IO uint32_t USART_GTPR;
} USART_TypeDef;

// Peripheral base pointers
#define RCC     ((RCC_TypeDef *) RCC_BASE)
#define GPIOA   ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB   ((GPIO_TypeDef *) GPIOB_BASE)
#define USART2  ((USART_TypeDef *) USART2_BASE)
#define USART3  ((USART_TypeDef *) USART3_BASE)

// ==============================
// Function Prototypes
// ==============================

void usart2_init(void);                        // Initializes USART2 TX (PA2)
void uart2_write(int ch);
char uart2_read(void);// Sends a single character over UART
static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);  // Sets BRR
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);                       // Baud rate calc


void usart3_init(void);                        // Initializes USART2 TX (PA2)
void uart3_write(int ch);
char uart3_read(void);
// ==============================
// Main Function
// ==============================
char letter;
char letter2;
int main(void) {

	usart2_init();               // Initialize UART2
	usart3_init();				 // Initialize UART3
	while (1) {
		uart3_write('T');
		for(volatile int i = 0; i < 2000000; i++){
			__asm__("nop");
		}
		letter = uart2_read();
		for(volatile int i = 0; i < 1000000; i++){
			__asm__("nop");
		}
		uart2_write('T');
		for(volatile int i = 0; i < 2000000; i++){
			__asm__("nop");
		}
		letter2 = uart3_read();
		for(volatile int i = 0; i < 1000000; i++){
			__asm__("nop");
		}
	}
}

// ==============================
// UART Initialization
// ==============================

void usart2_init(void) {

	/********** Configure uart gpio pin **********/

	/* Enable clock access to gpio */
	RCC->AHB1ENR |= GPIOAEN;

	/* Set PA2 mode to alternate function mode */
	GPIOA->MODER |= (1U << 5);        // MODER2[1]
	GPIOA->MODER &= ~(1U << 4);       // MODER2[0] → AF mode (10)

	/* Set PA2 alternate function type to UART_TX (AF7) */
	GPIOA->AFRL |= (1U << 8);         // AFRL2[0]
	GPIOA->AFRL |= (1U << 9);         // AFRL2[1]
	GPIOA->AFRL |= (1U << 10);        // AFRL2[2]
	GPIOA->AFRL &= ~(1U << 11);       // AFRL2[3]


	/* Set PA3 mode to alternate function mode */
	GPIOA->MODER |= (1U << 7);        // MODER2[1]
	GPIOA->MODER &= ~(1U << 6);       // MODER2[0] → AF mode (10)

	/* Set PA3 alternate function type to UART_RX (AF7) */
	GPIOA->AFRL |= (1U << 12);         // AFRL2[0]
	GPIOA->AFRL |= (1U << 13);         // AFRL2[1]
	GPIOA->AFRL |= (1U << 14);        // AFRL2[2]
	GPIOA->AFRL &= ~(1U << 15);       // AFRL2[3]


	/********** Configure uart module **********/
	/* Enable clock access to uart2 */
	RCC->APB1ENR |= USART2EN;

	/* Configure baudrate */
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	/* Configure the transfer direction */
	// '=' is used instead of '|=' to reset other CR1 bits and only set TE (transmitter enable)
	USART2->USART_CR1 = ((1U << 3) | (1U << 2) );

	/* Enable uart module */
	USART2->USART_CR1 |= (1U << 13);   // UE = 1
}

void usart3_init(void) {

	/********** Configure uart gpio pin **********/

	/* Enable clock access to gpio */
	RCC->AHB1ENR |= GPIOBEN;

	/* Set PB10 mode to alternate function mode */
	GPIOB->MODER |= (1U << 21);        // MODER2[1]
	GPIOB->MODER &= ~(1U << 20);       // MODER2[0] → AF mode (10)

	/* Set PB10 alternate function type to UART_TX (AF7) */
	GPIOB->AFRH |= (1U << 8);
	GPIOB->AFRH |= (1U << 9);
	GPIOB->AFRH |= (1U << 10);
	GPIOB->AFRH &= ~(1U << 11);


	/* Set PB11 mode to alternate function mode */
	GPIOB->MODER |= (1U << 23);        // MODER2[1]
	GPIOB->MODER &= ~(1U << 22);       // MODER2[0] → AF mode (10)

	/* Set PB11 alternate function type to UART_RX (AF7) */
	GPIOB->AFRH |= (1U << 12);         // AFRL2[0]
	GPIOB->AFRH |= (1U << 13);         // AFRL2[1]
	GPIOB->AFRH |= (1U << 14);        // AFRL2[2]
	GPIOB->AFRH &= ~(1U << 15);       // AFRL2[3]


	/********** Configure uart module **********/
	/* Enable clock access to uart3 */
	RCC->APB1ENR |= USART3EN;

	/* Configure baudrate */
	uart_set_baudrate(USART3, APB1_CLK, UART_BAUDRATE);

	/* Configure the transfer direction */
	// '=' is used instead of '|=' to reset other CR1 bits and only set TE (transmitter enable)
	USART3->USART_CR1 = ((1U << 3) | (1U << 2) );

	/* Enable uart module */
	USART3->USART_CR1 |= (1U << 13);   // UE = 1
}

// ==============================
// Write and read a character to USART2
// ==============================

char uart2_read(void) {
	/* Wait until transmit data register is empty (TXE) */
	while (!(USART2->USART_SR & (1U << 5))) { }

	/* Write data to the data register */
	return USART2->USART_DR;
}

void uart2_write(int ch) {
	/* Wait until transmit data register is empty (TXE) */
	while (!(USART2->USART_SR & (1U << 7))) { }

	/* Write data to the data register */
	USART2->USART_DR = (ch & 0xFF);
}

// ==============================
// Write and read a character to USART3
// ==============================

char uart3_read(void) {
	/* Wait until transmit data register is empty (TXE) */
	while (!(USART3->USART_SR & (1U << 5))) { }

	/* Write data to the data register */
	return USART3->USART_DR;
}

void uart3_write(int ch) {
	/* Wait until transmit data register is empty (TXE) */
	while (!(USART3->USART_SR & (1U << 7))) { }

	/* Write data to the data register */
	USART3->USART_DR = (ch & 0xFF);
}



// ==============================
// Baud Rate Configuration
// ==============================

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate) {
	USARTx->USART_BRR = compute_uart_bd(PeriphClk, BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate) {
	// Formula: USARTDIV = Fclk / Baudrate
	return ((PeriphClk + (BaudRate / 2U)) / BaudRate);
}



