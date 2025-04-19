# STM32 Bare-Metal: Bidirectional UART Communication (USART2 <-> USART3)

---

## 📌 Summary

This project demonstrates **two-way serial communication** between **USART2** and **USART3** on the STM32F4 Discovery board.  
Both USARTs are configured at the **register level** without using HAL or CMSIS.  
A pair of jumper wires is used to connect the TX/RX lines between the two peripherals, allowing them to send and receive messages from each other.

---

## 🔁 Previous Lesson

If you haven’t completed the previous lesson where we implemented USART-based communication, check it out here:

👉 [Previous Lesson: Bidirectional UART (USART2 <-> USART3)](https://github.com/iek2443/stm32-baremetal-uart-tx-printf-)

---

## 🧠 What You Will Learn

- How to configure **USART2 and USART3** for bidirectional UART communication
- How to set up **PA2/PA3** for USART2 and **PB10/PB11** for USART3
- How to handle transmission and reception logic using low-level flags (`TXE`, `RXNE`)
- How to build a minimal full-duplex UART interface on STM32

---

## ⚙️ Key Registers Used

- `RCC->AHB1ENR` → Enables clocks to GPIOA and GPIOB
- `RCC->APB1ENR` → Enables clocks to USART2 and USART3
- `GPIOA->MODER`, `GPIOB->MODER` → Configures alternate function mode
- `GPIOA->AFRL`, `GPIOB->AFRH` → Assigns AF7 for USART pins
- `USARTx->BRR` → Baud rate configuration
- `USARTx->CR1` → Enables TX, RX, and USART
- `USARTx->SR`  → TXE and RXNE flags for transmit/receive
- `USARTx->DR`  → Data read/write register

---

## 🔧 Requirements

- STM32F4 Discovery Board
- Jumper wires (2x)
- ARM GCC Toolchain or compatible

---

📁 Project Structure
--------------------

stm32-baremetal-uart-bidirectional/\
├── src/\
│   └── main.c         --> USART2 <-> USART3 bidirectional code\
├── inc/               --> (Optional: header files)\
└── README.md

---

🧭 Pin Mapping

| USART     | TX Pin | RX Pin | Port | AF Function |
|-----------|--------|--------|------|-------------|
| USART2    | PA2    | PA3    | A    | AF7         |
| USART3    | PB10   | PB11   | B    | AF7         |

---

## 🔌 Wiring

- Connect **PA2 (USART2_TX)** → **PB11 (USART3_RX)**
- Connect **PB10 (USART3_TX)** → **PA3 (USART2_RX)**

---

## 🔁 Communication Flow
- USART2 sends –> USART3 receives
- USART3 sends –> USART2 receives
---
