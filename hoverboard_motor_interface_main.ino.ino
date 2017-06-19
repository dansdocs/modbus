/*
  check pinb is working and reading stop bit properly
  can check data by doind first word after resynch

  next step: every four bits - convert to ascii and send. set serial speed to 115200
  disable all arduino serial stuff and directly set it up, don't use serial interrups as it messes up timing
*/
#include "AsciiModbusSlave.h"

#define R_DATA 9
#define DATA 1
#define L_DATA 3
#define SYNCH 6
#define DATA_FRAME 5
#define DATA_SAMPLE 4

#define SET_R_DATA_HIGH_NEXT_TICK() (TCCR1A |= _BV(6)) 
#define SET_R_DATA_LOW_NEXT_TICK() (TCCR1A &= ~(_BV(6))) 

#define SET_SYNCH_HIGH() (PORTD |= _BV(6)) 
#define SET_SYNCH_LOW() (PORTD &= ~(_BV(6))) 



int16_t speed = 0;

void InitTimer1(void) {
  // Clear the timer
  TCNT1 = 0;
  TIFR1  = 0b00100111; // clear any pending interups
  TIMSK1 = 0b00000010; // Enable output compare match A
  TCCR1A = 0b11110000; // CTC mode, use OCR1A set both OC1A and OC1B
  TCCR1B = 0b00001001; // CTC mode, no prescaler  
  
  // 38uS / (1/8MHz) = 304   38uS is one bit time, 8MHz is the processer clock speed. 
  // This means we will get an interupt every bit time. 
  OCR1A = 304; 

}






// the setup routine runs on reset:
void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(R_DATA, OUTPUT);  
  pinMode(SYNCH, OUTPUT);  
  pinMode(DATA_FRAME, OUTPUT);  
  pinMode(DATA_SAMPLE, OUTPUT);


  digitalWrite(SYNCH, LOW);  
  digitalWrite(DATA_FRAME, LOW);  
  digitalWrite(DATA_SAMPLE, LOW);
  digitalWrite(DATA, HIGH);

  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TIMSK0 &= ~_BV(TOIE0);    // disable timer0 overflow interrupt, this means delay() millis() and micros() no longer work.
  InitTimer1();
  interrupts();             // enable all interrupts

  modbus_init(0xA5);
}


// There are two statemachines running together. One that marks where we are up to in the message frame and one that marks where
// we are up to in sending out each individual bit for a byte that makes up the message. (where byte is actually 9bits plus start and stop). 
// The 9th bit is only used to indicate the start of a message frame so it doesn't make up any part of the actual data - the actual data is byte based. 

// A data frame consists of a start and end frame with a repeated speed (twos compliment 16 bit integer). 
enum MSGST {sSTART_FRAME, sSPEED1, sSPEED2, sSPEED3, sSPEED4, sPAUSE};
//typedef enum MSGST message_states;  
//message_states message_state = sPAUSE;
uint8_t message_state = sPAUSE;

// States for where we are up to when sending an individuale byte (well its 9bits of data plus start and stop bits) 
enum BITST {sD0, sD1, sD2, sD3, sD4, sD5, sD6, sD7, sD8, sSTOP, sFINISH, sIDLE, sSTART};
//typedef enum BITST bit_states;  
//bit_states bit_state = sIDLE;
uint8_t bit_state = sIDLE;

uint8_t speed_update_guard = 0; 
int16_t rspeed = 0;
int16_t lspeed = 0;

int16_t buffered_rspeed = 0;
int16_t buffered_lspeed = 0;

uint8_t rdata = 0;
uint8_t ldata = 0;

static uint8_t delay_count = 0;
ISR(TIMER1_COMPA_vect) {

  UPDATE_MODBUS_TIMER();
  
  // send out the byte of data along with start and stop bits. The 9th bit is treated specially since its only used if 
  // we are sending the start of frame byte. All the other times its zero. 
  switch(bit_state) {
      case sIDLE :
        // It is up to the message state machine to move this statemachine out of sIDLE. 
        SET_R_DATA_HIGH_NEXT_TICK();
        break; 
      case sSTART:    
        SET_R_DATA_LOW_NEXT_TICK();
        bit_state = sD0;
        break;           
      case sD0: case sD1: case sD2: case sD3: case sD4: case sD5: case sD6: case sD7: 
        if (rdata & _BV(bit_state)) SET_R_DATA_HIGH_NEXT_TICK();
        else SET_R_DATA_LOW_NEXT_TICK();
        bit_state++;
        break;  
      case sD8:
        // the only time the 9th bit is used is to send the frame start message. 
        if (message_state == sSTART_FRAME) SET_R_DATA_HIGH_NEXT_TICK();
        else SET_R_DATA_LOW_NEXT_TICK();
        bit_state = sSTOP;
        break;
      case sSTOP:       
        SET_R_DATA_HIGH_NEXT_TICK();  
        bit_state = sFINISH;     
        break;
      case sFINISH:
        SET_R_DATA_HIGH_NEXT_TICK();  
        bit_state = sIDLE;     
        break;      
      default :
        bit_state = sIDLE;
        SET_R_DATA_HIGH_NEXT_TICK();
        break;        
  }

  // cycle through the a frame of data and prepare each byte of data for the state machine above to send out. 
  // this statemachine only progresses when the bit statemachine above is in the idle state - i.e. after each byte has been sent
  // we work out what to send next. 
  if (bit_state == sIDLE) {
    switch(message_state) { 
        case sSTART_FRAME : rdata = (rspeed & 0xFF); break;   
        case sSPEED1 : rdata = ((rspeed >> 8) & 0xFF); break;     
        case sSPEED2 : rdata = (rspeed & 0xFF); break;    
        case sSPEED3 : rdata = ((rspeed >> 8) & 0xFF); break;    
        case sSPEED4 : rdata = 0x55; break;   
        case sPAUSE : break;                                  
        default : break;          
    }
    if (message_state >= sPAUSE) {
      // delay for 500uS between each frame of data 500uS is longer than sending out 11bits which is one word. This big regular pause 
      // makes it easy for a software bit banging serial port to synchronise to the data, and easy to see frames on a scope or logic analyser. 
      // Also, update the speed values ready for the next frame of data. Since the speed values are 16bit, check the guard first to make sure
      // they aren't part way through being updated when the interrupt happned. 
      SET_SYNCH_HIGH();
       delay_count++;
       if (!(speed_update_guard)) {
         buffered_rspeed = rspeed;
         buffered_lspeed = lspeed;
       }
       if (delay_count > 13) {
         delay_count = 0;    
         rdata = 0x00;
         message_state = sSTART_FRAME;  
         bit_state = sSTART; // kick the other statemachine off to send out the byte of data.  
         SET_SYNCH_LOW();
       }   
    }
    else {
      message_state++;
      bit_state = sSTART; // kick the other statemachine off to send out the byte of data.
    }    
  }  
}





//uint8_t ascii_hex(uint16_t in)
//{ 
//  uint8_t ascii_char = 0;
//  // convert high bits into an ascii hex value
//  if ( (((in & 0b11110000) >> 4) + 48) < 58 )  
//    ascii_char = ( ((in & 0b11110000) >> 4) + 48 ); 
//  else 
//    ascii_char = ( ((in & 0b11110000) >> 4) + 55 ); 
//  
//                                                     
//  // convert low bits into an ascii hex value
//    if ( ((in & 0b00001111) + 48) < 58 )
//        ascii_char = ( (in & 0b00001111) + 48 );
//  else 
//        ascii_char = ( (in & 0b00001111) + 55 );    
//
//  return 0;
//}

// the loop routine runs over and over again forever:
void loop() {

  modbus_update();
  
//  uint8_t uart_data = 0;
  
//  if (UART_DATA_AVAILABLE()) {
 //   uart_data = UDR0;  // reading register clears it. 
 //   if (uart_data == '.') {
 //     speed_update_guard = 1;
 //     rspeed += 10;
 //     speed_update_guard = 0;
 //     UDR0 = '+';
 //   }
 //   else if (uart_data == '>') {
 //     speed_update_guard = 1;
 //     rspeed += 100;
 //     speed_update_guard = 0;
 //     UDR0 = '*';
 //   }    
 //   else if (uart_data == ',') {
 //     speed_update_guard = 1;
 //     rspeed -= 10;
 //     speed_update_guard = 0;
 //     UDR0 = '-';  
 //   }
 //   else if (uart_data == '<') {
 //     speed_update_guard = 1;
 //     rspeed -= 100;
 //     speed_update_guard = 0;
 //     UDR0 = '=';  
  //  }    
  //  else if (uart_data == ' ') {
  //    speed_update_guard = 1;
 //     rspeed = 0;
 //     speed_update_guard = 0;
 //     UDR0 = '0';  
 //   }
 //   else UDR0 = '?';
//  }
}



