#ifndef ASCII_MODBUS_SLAVE_H
#define ASCII_MODBUS_SLAVE_H

#include <stdint.h>
#include <avr/io.h>
/*
 
 This implementation DOES NOT fully comply with the Modbus specifications.
 
 SimpleModbusSlave implements an unsigned int return value on a call to modbus_update().
 This value is the total error count since the slave started. It's useful for fault finding.
 
 This code is for a Modbus slave implementing functions 3, 6 
 function 3: Reads the binary contents of holding registers (4X references)
 function 6: Presets a value into a single holding register (4X reference)
 
*/


// -------------------------------
//   MODBUS exceptions
// -------------------------------

#define EXCEPTION_ILLEGAL_FN 1
#define EXCEPTION_ILLEGAL_DATA_ADDRESS 2
#define EXCEPTION_ILLEGAL_DATA_VALUE 3
#define EXCEPTION_SLAVE_DEVICE_FAILURE 4


// -------------------------------
//         FUNCTIONS TO CALL
// -------------------------------

// modbus_init() must be called once before using. 
// modbus_update() must be called in the main loop at least at the rate that bytes are expected to arrive. 
// at 57600bps that is approx 173uS.
// Call the UPATE MODBUS_TIMER() in an interrupt routine and define the MODBUS_DELAY value. Example:
// modbus spec is a 3.5 character delay between send and transmist. At 57600 that is approx 1/57600 * 10 * 3.5 = 600uS.
// Say your using an interrupt routine which is set to 38uS (so if UPDATE_MODBUS_TIMER() is called every 38us) then 
// you would set the the MODBUS_DELAY to be 600/38 which is approximatly 16. 
// Note to be atomic on an eight bit processor the value must be a uint8_t and so it has to be less than 255. 

void modbus_init(uint8_t slaveID_init);
uint16_t modbus_update();
#define UPDATE_MODBUS_TIMER() ({if (modbus_timer > 0) modbus_timer--;})
#define MODBUS_DELAY 20



// -------------------------------------
//          HOLDING REGISTER MAP 
// -------------------------------------

// Leave hr_ARRAY_SIZE at the end of the enum, its used to create the array to hold the registers. 
// adjust all the other entries according to the application. 
// NOTE: No more than 255 register entries are supported by the current code. 

enum HREG {
  hr_L_MOTOR_SPEED_SETTING,
  hr_R_MOTOR_SPEED_SETTING,
  hr_L_MOTOR_SPEED_MEASURED,
  hr_R_MOTOR_SPEED_MEASURED,
  hr_ERRORCOUNT,
  hr_ARRAY_SIZE
};

// -------------------------------
//  HARDWARE CONFIGURATION 
// -------------------------------

// The following to allow the modbus code to use
// the hardware UART and output pin availble on 
// the microcontroller. Only needs to be 
// adjusted if the hardware is being changed.   
// The following is for the ATMEGA328.

// --------------------------
//    TRANSMIT PIN ENABLE
// --------------------------


#define NOP __asm__ __volatile__ ("nop\n\t")

// The following pin will be set high/low so that an RS485 chip can be connected to he tx/rx pins 
// and used on a multidrop bus. Replace the following three definitions with NOP if you don't want to use the pin this way. 
// EG:  #define SET_TX_ENABLE_LOW() NOP
// the TX_PIN_SETUP macro will be called once as part of modbus_init to configure the hardware. 

#define SET_TX_ENABLE_LOW() (PORTD &= ~(_BV(6))) 
#define SET_TX_ENABLE_HIGH() (PORTD |= _BV(6))
#define TX_PIN_SETUP() NOP  


// --------------------------
//    HARDWARE UART MACROS
// --------------------------

// Macros to setup the hardware UART, check the status of the UART (is it ready to send, does it have data)
// and macros to actually get a byte from the hardware and send a byte. 
// the UART_SETUP macro will be called once as part of modbus_init to configure the hardware. 
 
#define UART_BYTE_AVAILABLE() (UCSR0A & (_BV(RXC0)))
#define UART_READY_TO_SEND() (UCSR0A & (_BV(UDRE0)))
#define UART_GET_BYTE() (UDR0)
#define UART_SEND_BYTE(data) (UDR0 = data)

// set baud rate to 57600bps as per datasheet table for 8MHZ xtal.
// UBRR = (8MHz / (8*57600bps)) - 1 = 16.4 => round to 16
// Timing error = 2.1%   
// 8,N,1 No interrupts. 

#define UART_SETUP() ({              \
                UBRR0H = 0;          \
                UBRR0L = 16;         \
                UCSR0A = 0b01100010; \
                UCSR0B = 0b00011000; \
                UCSR0C = 0b00000110; \
        })



//----------------------------------------------
// Don't use the following....
// The following go with the UPDATE_MODBUS_TIMER() 
// but are used by AsciiModbusSlave.cpp

#define SET_MODBUS_TIMER(val) (modbus_timer = val)
#define MODBUS_TIMER_EXPIRED() ((modbus_timer == 0) ? 1 : 0) 
extern uint8_t modbus_timer;

// -------------------------------------

#endif
