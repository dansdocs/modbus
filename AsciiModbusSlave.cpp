#include "AsciiModbusSlave.h"
#include <avr/pgmspace.h> // needed when using Progmem

// AsciiModbusSlave
//----file globals------------------------- 
 
// These global variables are pointed to by both the transmit and recieve structures.
// MODBUS is a masterslave system so only the transmit OR receive statemachines will be 
// changing things at any one point in time.  
uint8_t slaveID = 0x01;                     // the modbus ID of this slave device. 
uint8_t functionCode = 0x06;                // the function code sent by the master 
uint16_t holding_registers[hr_ARRAY_SIZE] = {0x0000, 0x3456, 0x0001, 0x0002, 0x0003};  // the modbus registers 
uint8_t modbus_timer = 0;                   // updated by an interrupt (outside this file) it decrements to zero every 38uS. 
                                            // set it to a value, when it gets to zero then that value x 38uS have elapsed.
                                            // used by the tx statemachine start state.  
                          
//----helper functions------------------------- 

uint8_t ascii_to_uint8(uint8_t ch);
uint8_t send_bin_as_ascii_char(uint16_t data, uint8_t bits);
uint8_t calculateLRC(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t array_start_index, uint8_t number_of_array_elements, uint16_t the_array[]);
#define UINT8_TO_ASCII(in) (((in) < 10) ? ((in) + '0') : ((in) < 16) ? ((in) + ('A' - 10)) : 255)
void exceptionResponse(uint8_t exception);

//----General State machine infrustructure------

// a row of the statemachine table array, to make the array a bit nicer.  
#define ST(current_state, forward_state, branch_state, repeat_state) {\
            {current_state, forward_state}, \
            {current_state, branch_state},  \
            {current_state, repeat_state},  \
        }

// The ONLY valid return values from a state machine function
// noting that rMAX... is not a valid return value, its used to define the array size.          
enum return_values {rFORWARD, rBRANCH, rREPEAT, rMAX_RETURN_VALUES};

// To create something a bit like a poor mans dictionary, current state is paired with each possible future state.
// in the state table .
enum transition_index {tCURRENT_STATE, tFUTURE_STATE, tMAX_TRANSITION_INDEX};

// The macro works out the next state using the current state and the value returned from the function. 
#define NEXT_STATE(current_state, return_val) (\
              return_val == rFORWARD ? txs_state_table[current_state][rFORWARD][tFUTURE_STATE] \
            : return_val == rBRANCH ? txs_state_table[current_state][rBRANCH][tFUTURE_STATE] \
            : txs_state_table[current_state][rREPEAT][tFUTURE_STATE]\
        )    

// The following macros are used to return one of the valid return values from a state function. 
// note that the in parameter isn't used, it can be used to document whats happening and can be extracted 
// to draw the state machine diagram. 
#define REPEAT_UNTIL(in) do{return rREPEAT;}while(0)
#define FORWARD_WHEN(in) do{return rFORWARD;}while(0)
#define BRANCH_IF(in) do{return rBRANCH;}while(0)

//----State function prototypes and data structures for receive and transmit statemachines------

// Structure of data for the slave transmit state machine. Some of the entries are pointers to the 
// global variables which are shared with the slave receive state machine. 
typedef struct TXRX {
   uint8_t   messageReadyToSend = 1;         // Setting to 1 makes this slave send a response (moves out of mFINISH state)
   uint8_t   data_in;                        // Data from the UART   
   uint8_t*  slave_ID = &slaveID;            // MODBUS slave ID for this device. Wraps the global ID into this struct for neatness
   uint8_t*  fnCode = &functionCode;         // Master function command. Wraps the global into this struct for neatness.  
   uint8_t   exception = 0x01;               // Exception code if an exception happened when msg rx from master. 
   uint8_t   numBytes = 0x02;                // Number of bytes to be sent back to master (should be 2x numRegisters)  
   uint8_t   dataAddress = 0x01;             // Data adress to start reading data back to the master. 
   uint8_t   numRegisters = 0x01;            // The number of registers to send back to the master (should be half of the numBytes). 
   uint16_t* registers = holding_registers;  // this just wraps the global array into the struct for neatness. 
}TXRXdata;

TXRXdata txrx;

// Structure of data for the slave receive state machine. Some of the entries are pointers to the 
// global variables which are shared with the slave transmit state machine. 
typedef struct RXS {
   uint8_t   data_in;                        // Data from the UART
   uint8_t*  slave_ID = &slaveID;            // MODBUS slave ID for this device. Wraps the global ID into this struct for neatness
   uint8_t*  fnCode = &functionCode;         // Master function command. Wraps the global into this struct for neatness.  
   uint8_t   exception = 0x01;               // Exception code if an exception happened when msg rx from master. 
   uint8_t   numBytes = 0x02;                // Number of bytes to be sent back to master (should be 2x numRegisters)  
   uint8_t   dataAddress = 0x01;             // Data adress to start reading data back to the master. 
   uint8_t   numRegisters = 0x01;            // The number of registers to send back to the master (should be half of the numBytes). 
   uint16_t* registers = holding_registers;  // this just wraps the global array into the struct for neatness. 
}Rxstdata;

Rxstdata rxst;


uint8_t ftx_mTXSTART_beginDelayTimer (uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mCOLON_sendColon(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mSLAVEID_sendAddress(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mFNCODE_sendFnCode(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mCHOICE_fn3OrFn6(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mEXCEPT_sendException(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mLRC_sendLrc(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mFINISH_waitUntilNewMsg(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mNUMDATA_txNumBytes(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mTXREG_sendRegisters(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mDATAADD_txDataAddress(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mVALUE_txDataValue(uint8_t previous_state, TXRXdata* txd);
uint8_t ftx_mCRLF_sendCrLf(uint8_t previous_state, TXRXdata* txd); 

uint8_t frx_rSTART_waitForColon(uint8_t previous_state, Rxstdata* rxd); 
uint8_t frx_rSLAVEID_rxSlaveId(uint8_t previous_state, Rxstdata* rxd); 
uint8_t frx_rFNCODE_rxFnCode(uint8_t previous_state, Rxstdata* rxd); 

//----Array of ALL state functions and a set of indexes to retrieve the function from the array------

// Must keep the enum (array indexes) consistent with the array of function pointers. 
enum STATESENUM{
    mTXSTART, mCOLON, mSLAVEID, mFNCODE, mCHOICE, mEXCEPT, mLRC, mFINISH, mNUMDATA, mTXREG, mDATAADD, mVALUE, mCRLF,
    rSTART, rSLAVEID, rFNCODE, 
    mMAX_STATES, mNULL // leave these two in place at the end of the enum
};

uint8_t (*state_fns_array[mMAX_STATES])(uint8_t, TXRXdata*) = { 
          ftx_mTXSTART_beginDelayTimer, ftx_mCOLON_sendColon, ftx_mSLAVEID_sendAddress, ftx_mFNCODE_sendFnCode, 
          ftx_mCHOICE_fn3OrFn6, ftx_mEXCEPT_sendException, ftx_mLRC_sendLrc, ftx_mFINISH_waitUntilNewMsg, ftx_mNUMDATA_txNumBytes,
          ftx_mTXREG_sendRegisters, ftx_mDATAADD_txDataAddress, ftx_mVALUE_txDataValue, ftx_mCRLF_sendCrLf,
          frx_rSTART_waitForColon, frx_rSLAVEID_rxSlaveId, frx_rFNCODE_rxFnCode
};

//----Transmit statemachine table------

// This is one specific statemachine a sequence of the functions above. 
 
// the pattern for each row is:
// ST( currentState, forwardState, branchState, repeatState)
uint8_t txs_state_table[mMAX_STATES][rMAX_RETURN_VALUES][tMAX_TRANSITION_INDEX] = {
    ST(mTXSTART, mCOLON, mNULL, mNULL),
    ST(mCOLON, mSLAVEID, mNULL, mCOLON),
    ST(mSLAVEID, mFNCODE, mNULL, mSLAVEID),
    ST(mFNCODE, mCHOICE, mEXCEPT, mFNCODE),    
    ST(mCHOICE, mNUMDATA, mDATAADD, mNULL),        
    ST(mEXCEPT, mLRC, mNULL, mEXCEPT),            
    ST(mLRC, mCRLF, mNULL, mLRC),                
    ST(mFINISH, mTXSTART, mNULL, mFINISH),
    ST(mNUMDATA, mTXREG, mNULL, mNUMDATA),   
    ST(mTXREG, mLRC, mNULL, mTXREG),   
    ST(mDATAADD, mVALUE, mNULL, mDATAADD),       
    ST(mVALUE, mLRC, mNULL, mVALUE),           
    ST(mCRLF, mFINISH, mNULL, mCRLF)
};


//----Receive statemachine table------

// This is one specific statemachine. a sequence of the functions above. 
 
// the pattern for each row is:      
// ST( currentState, forwardState, branchState, repeatState)
uint8_t rxs_state_table[mMAX_STATES][rMAX_RETURN_VALUES][tMAX_TRANSITION_INDEX] = {
    ST(rSTART, rSLAVEID, mNULL, rSTART),
    ST(rSLAVEID, rFNCODE, mNULL, mNULL),
    ST(rFNCODE, rSTART, mNULL, mNULL)
};

//------------------------------------------------------------------------------


void modbus_init(uint8_t slaveID_init) {
  UART_SETUP();
  TX_PIN_SETUP();
  slaveID = slaveID_init;
}


enum MODBUSST {sSLAVE_ADDRESS, sCOMMAND, sREG_ADDRESS1, sREG_NUM_OR_ADDRESS2, sVALUE1, sVALUE2, sLRC, sCR, sLF, sEXCEPTION, sCOLON};



// ignore the start and end (colon and cr, lf), but store all the other bytes in the dataframe.
// The rxFrame is layed out: [slaveAddress, command, registeraddresshighbyte, registeraddresslowbyte, valuehighbyte, valuelowbyte, lrc] 
// The layout of the array is the same as the enumerated state variable so the state variable can be used as the index. 
#define NUMBER_OF_BYTES_IN_RX_FRAME 7
uint8_t rxFrame[NUMBER_OF_BYTES_IN_RX_FRAME];

uint8_t modbus_rx_state = sCOLON;

void modbus_receive_statemachine (uint8_t data_in) {

  static uint8_t get_hi = 1;
  static uint8_t hi_nibble = 0;
  static uint8_t lo_nibble = 0;

  // if a colon is recieved at any stage, it resets things back to the start. 
  if (data_in == ':') modbus_rx_state = sCOLON;

  // ignore tab and space bytes (whitespace)
  if ((data_in == 9) || (data_in == 32)) return 0;

  switch(modbus_rx_state) {
      case sCOLON :
          if (data_in == ':') {
              modbus_rx_state = sSLAVE_ADDRESS;
              get_hi = 1;
          }
          break;
      case sSLAVE_ADDRESS:   
          if (get_hi) {
              get_hi = 0;
              hi_nibble = ascii_to_uint8(data_in);
          }
          else { 
              get_hi = 1;            
              lo_nibble = ascii_to_uint8(data_in);
              rxFrame[sSLAVE_ADDRESS] = ((hi_nibble << 8) | lo_nibble);
              // wait for the next frame to start if this isn't our address. 
              if (rxFrame[sSLAVE_ADDRESS] != slaveID) modbus_rx_state = sCOLON;  
              else modbus_rx_state = sCOMMAND;
          }
          break;
      case sCOMMAND :
         if (get_hi) {
              get_hi = 0;
              hi_nibble = ascii_to_uint8(data_in);
          }
          else { 
              get_hi = 1;
              lo_nibble = ascii_to_uint8(data_in);
              rxFrame[sCOMMAND] = ((hi_nibble << 8) | lo_nibble);
              // Only commands 3 and 6 are availble, everything else is an error. 
              if ((rxFrame[sCOMMAND] == 3) || (rxFrame[sCOMMAND] == 6)) modbus_rx_state = sREG_ADDRESS1;  
              else {
                exceptionResponse(EXCEPTION_ILLEGAL_FN);
                modbus_rx_state = sCOLON;
              }
          }      
          break;
      case sREG_ADDRESS1 : case sREG_NUM_OR_ADDRESS2 :
          if (get_hi) {
              get_hi = 0;
              hi_nibble = ascii_to_uint8(data_in);
          }
          else {
              get_hi = 1;  
              lo_nibble = ascii_to_uint8(data_in);
              rxFrame[modbus_rx_state] = ((hi_nibble << 8) | lo_nibble);  
              if (modbus_rx_state == sREG_ADDRESS1) {
                  if (rxFrame[sREG_ADDRESS1] != 0) {
                      exceptionResponse(EXCEPTION_ILLEGAL_DATA_ADDRESS);  // not supporting more than 255 so this has to be zero. 
                      modbus_rx_state = sCOLON;                                             
                  }
                  else modbus_rx_state = sREG_NUM_OR_ADDRESS2;           
              }
              else {
                  if (rxFrame[sREG_NUM_OR_ADDRESS2] >= hr_ARRAY_SIZE) {
                      exceptionResponse(EXCEPTION_ILLEGAL_DATA_ADDRESS);  // can't have an address greater than defined registers. 
                      modbus_rx_state = sCOLON;                                             
                  }
                  else modbus_rx_state = sVALUE1;
              }
          }
          break;  
      case sVALUE1 : case sVALUE2 :
          if (get_hi) {
              get_hi = 0;
              hi_nibble = ascii_to_uint8(data_in);
          }
          else {
              get_hi = 1;  
              lo_nibble = ascii_to_uint8(data_in);
              rxFrame[modbus_rx_state] = ((hi_nibble << 8) | lo_nibble);  
              modbus_rx_state++;   
          }      
          break;  
      case sLRC :
          if (get_hi) {
              get_hi = 0;
              hi_nibble = ascii_to_uint8(data_in);
          }
          else {
              get_hi = 1;  
              lo_nibble = ascii_to_uint8(data_in);
              rxFrame[sLRC] = ((hi_nibble << 8) | lo_nibble);  
              // calculate LRC on the received bytes and check. Don't inlude the LRC itself so subtract 1 from index. 
//              if (calculateLRC(rxFrame, NUMBER_OF_BYTES_IN_RX_FRAME - 1) == rxFrame[sLRC]) modbus_rx_state = sCR;
//              else {
//                  exceptionResponse(EXCEPTION_ILLEGAL_DATA_VALUE);  // LRC didn't match. 
                  modbus_rx_state = sCOLON;                
//              }
          }      
          break;                                                        
      case sCR :
          if (data_in == '\r') modbus_rx_state = sLF;  
          else {
              modbus_rx_state = sCOLON;  
              exceptionResponse(EXCEPTION_ILLEGAL_DATA_VALUE);
          }
          break;  
      case sLF :
          if (data_in == '\n') {
              modbus_rx_state = sCOLON;
              // DO SOMETHING WITH THE RECEIVED FRAME!!!!!!-----------------------------------  
          }
          else {
              modbus_rx_state = sCOLON;  
              exceptionResponse(EXCEPTION_ILLEGAL_DATA_VALUE);
          }
          break;         
      default :
          break;      
  }  
}


uint16_t modbus_update() {

    static uint8_t cur_state = mFINISH;
    static uint8_t previous_state = mFINISH;
    uint8_t rc;
    uint8_t (* state_fun)(uint8_t, TXRXdata*);

    if (UART_READY_TO_SEND()) {
        state_fun = state_fns_array[cur_state];
        rc = state_fun(previous_state, &txrx);
        previous_state = cur_state;
        cur_state = NEXT_STATE(cur_state, rc);      
    }

  
    return 0;
}

// return the ascii character representation for integers 0,1,2,3...15
// return 255 if its an error (a number bigger than 15)
uint8_t uint8_to_ascii(uint8_t in) {
	if (in < 10) return (in + '0');
  else if (in < 16) return (in + ('A' - 10));
	return 255;
}

// return the integer value for the ascii characters '0', '1', '2', .... '9', 'A', 'B', .... 'F'
// return 255 if any other character. 
uint8_t ascii_to_uint8(uint8_t ch) {
  if ((ch >= '0') && (ch <= '9')) return (ch - '0');    
  else if ((ch >= 'A') && (ch <= 'F')) return (ch - ('A' - 10)); 
  return 255;
}



//--Slave transmit state machine functions-------------------------------------------------------------------------------

uint8_t ftx_mTXSTART_beginDelayTimer (uint8_t previous_state, TXRXdata* txd) {
     // there is supposed to be a delay between a message being recieved and a message being sent. 
     SET_MODBUS_TIMER(MODBUS_DELAY);
     FORWARD_WHEN("Delay timer started");  
}

uint8_t ftx_mCOLON_sendColon(uint8_t previous_state, TXRXdata* txd){
  if (!MODBUS_TIMER_EXPIRED()) REPEAT_UNTIL("Delay timer has expired"); 
  UART_SEND_BYTE(':');
  FORWARD_WHEN("Delay timer expired and ':' sent");  
}

uint8_t ftx_mSLAVEID_sendAddress(uint8_t previous_state, TXRXdata* txd){
    if (send_bin_as_ascii_char((uint16_t) *(txd->slave_ID), 8)) FORWARD_WHEN("All SalveID nibbles sent");
    REPEAT_UNTIL("All SlaveID nibbles sent");           
}

uint8_t ftx_mFNCODE_sendFnCode(uint8_t previous_state, TXRXdata* txd){
    if (send_bin_as_ascii_char((uint16_t) *(txd->fnCode), 8)) 
    {
      if (*(txd->fnCode) & 0b10000000) BRANCH_IF("Fn is an exception");
      else FORWARD_WHEN("All FnCode nibbles sent"); 
    }    
    REPEAT_UNTIL("All FnCode nibbles sent");           
}

uint8_t ftx_mCHOICE_fn3OrFn6(uint8_t previous_state, TXRXdata* txd){
      if (*(txd->fnCode) == 0x03) FORWARD_WHEN("Fn Code 03");
      BRANCH_IF("Fn Code 06");        
}

uint8_t ftx_mEXCEPT_sendException(uint8_t previous_state, TXRXdata* txd) {
    if (send_bin_as_ascii_char((uint16_t) txd->exception, 8)) FORWARD_WHEN("Exception nibbles sent");
    REPEAT_UNTIL("Exception nibbles sent");     
}

uint8_t ftx_mFINISH_waitUntilNewMsg(uint8_t previous_state, TXRXdata* txd){
    if (txd->messageReadyToSend == 1) FORWARD_WHEN("Msg to send");
    else REPEAT_UNTIL("Msg available");   
}
uint8_t ftx_mNUMDATA_txNumBytes(uint8_t previous_state, TXRXdata* txd){
    if (send_bin_as_ascii_char((uint16_t)txd->numBytes, 8)) FORWARD_WHEN("Nibbles for num bytes sent");
    REPEAT_UNTIL("Nibbles for num bytes sent");     
}

uint8_t ftx_mDATAADD_txDataAddress(uint8_t previous_state, TXRXdata* txd){
    if (send_bin_as_ascii_char((uint16_t)txd->dataAddress, 16)) FORWARD_WHEN("Data address nibbles sent");
    REPEAT_UNTIL("Data address nibbles sent");     
}

uint8_t ftx_mVALUE_txDataValue(uint8_t previous_state, TXRXdata* txd){
    if (send_bin_as_ascii_char((uint16_t) txd->registers[txd->dataAddress], 16)) FORWARD_WHEN("Value nibbles sent");
    REPEAT_UNTIL("Value nibbles sent");     
}

uint8_t ftx_mLRC_sendLrc(uint8_t previous_state, TXRXdata* txd){
    static uint8_t lrc;
    
    if (previous_state == mEXCEPT) {
      //                 b1,                b2,            b3,             b4,   array_start_index, number_of_array_elements, the_array[]      
      lrc = calculateLRC(*(txd->slave_ID), *(txd->fnCode), txd->exception, 0x00, 0x00,              0x00,                     txd->registers);    
    }    
    else if (previous_state == mTXREG) {
      //                 b1,                b2,            b3,            b4,   array_start_index, number_of_array_elements, the_array[]     
      lrc = calculateLRC(*(txd->slave_ID), *(txd->fnCode), txd->numBytes, 0x00, txd->dataAddress,  txd->numRegisters,        txd->registers);          
    }
    else if (previous_state == mVALUE) {
      //                 b1,                b2,            b3,                b4,   array_start_index, number_of_array_elements, the_array[]     
      lrc = calculateLRC(*(txd->slave_ID), *(txd->fnCode), txd->dataAddress,  0x00, txd->dataAddress,  0x01,                     txd->registers);          
    }    
    
    if (send_bin_as_ascii_char((uint16_t)lrc, 8)) FORWARD_WHEN("LRC nibbles sent");
    REPEAT_UNTIL("LRC nibbles sent");
}

uint8_t ftx_mTXREG_sendRegisters(uint8_t previous_state, TXRXdata* txd) {
    static uint8_t i = 0;
    uint8_t address = 0;
    
    if (previous_state != mTXREG) i = 0; 
    address = txd->dataAddress + i;
    
    if (send_bin_as_ascii_char((uint16_t)txd->registers[address], 16)) {
      i++;
      if (i >= txd->numRegisters) FORWARD_WHEN("Registers sent");
    }
    REPEAT_UNTIL("Registers sent");     
}

uint8_t ftx_mCRLF_sendCrLf(uint8_t previous_state, TXRXdata* txd) {
    if (previous_state != mCRLF) {
      UART_SEND_BYTE(0x0D);
      REPEAT_UNTIL("CR and LF sent");
    } 
    else {
      UART_SEND_BYTE(0x0A);
      FORWARD_WHEN("CR and LF sent");  
    } 
}

//--Slave receive state machine functions-------------------------------------------------------------------------------

uint8_t frx_rSTART_waitForColon(uint8_t previous_state, Rxstdata* rxd) {
  if (rxd->data_in == ":") FORWARD_WHEN("Rx Colon");
  else REPEAT_UNTIL("Rx Colon");  
} 

uint8_t frx_rSLAVEID_rxSlaveId(uint8_t previous_state, Rxstdata* rxd) {
  FORWARD_WHEN("null");    
  
}

uint8_t frx_rFNCODE_rxFnCode(uint8_t previous_state, Rxstdata* rxd) {
  FORWARD_WHEN("null");    
}

//----Helper functions---------------------------------------------------------------------------------------------

// Convert binary data into an ASCII hexidecimal representation & send out the uart. 
// uint8_t (8 bits) will be represented as two ASCII charactes. Type cast the data as follows:
// send_bin_as_ascii_char((uint16_t)0xA7, 8)  The eight signifies eight bits so two characters sent. 
// uint16_t will be sent as four ascii characters. It is called as per:
// send_bin_as_ascii_char((uint16_t)0xCDA7, 16)  The 16 signifies sixteen bits so four characters sent. 
// The function assumes it is called sequentially until all is done otherwise it will get out of synch.   
// returns 1 meaning the data has been sent. 
// return 0 means the function has to be called again with the same data to complete. 
//
uint8_t send_bin_as_ascii_char(uint16_t data, uint8_t numBits){
    static uint8_t nibble_counter = 0;
    uint8_t total_nibbles = 0;
    uint8_t data_nibble;
    uint8_t shift;
    
    total_nibbles = numBits >> 2;  // each four bits is represented by one ascii character so divide by four.
    shift = ((total_nibbles - 1) - nibble_counter) << 2;
    data_nibble = (uint8_t)((data >> shift) & 0x000F); 
    UART_SEND_BYTE(UINT8_TO_ASCII(data_nibble)); 
    nibble_counter++;
    if (nibble_counter >= total_nibbles) {
      nibble_counter = 0;
      return 1;
    }
    return 0;  
}


// You can calculate the LCR using none, some or all of b1 through to b4 and include none, some or all of the_array by 
// setting setting number_of_array_elements to zero (so none of the array will be included into the calculation) 
// or to some positive value which will include part of the array starting from array_start_index. 
// Basically set the things you don't want to impact the LRC calculation to zero. 
//
// NB: array_start_index and number_of_array_elements are NOT included in the LRC calculation.   
//
uint8_t calculateLRC(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t array_start_index, uint8_t number_of_array_elements, uint16_t the_array[]) {
  uint8_t i = 0;
  uint8_t lrc = 0;
  
  lrc = b1 + b2 + b3 + b4;
  for (i=array_start_index; i < array_start_index + number_of_array_elements; i++){
    lrc = lrc + (uint8_t)((the_array[i] >> 8) & 0x00FF);    
    lrc = lrc + (uint8_t)(the_array[i] & 0x00FF);          
  }
  
  // 2's compliment
  lrc = 0xFF - lrc;
  lrc = lrc + 1;

  return lrc;
}

