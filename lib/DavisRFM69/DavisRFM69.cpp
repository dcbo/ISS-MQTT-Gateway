// Driver implementation for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from 
// a Davis Instrument wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work for now from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/

#include <DavisRFM69.h>
#include <RFM69registers.h>
#include <SPI.h>

volatile byte DavisRFM69::_data[DAVIS_PACKET_LEN];                 // packet receive buffer
volatile byte DavisRFM69::_channel = 0;                            // actual channel
volatile bool DavisRFM69::_hasCrcError = false;                      // current packet has been read
volatile bool DavisRFM69::_packetReceived = false;                 // packet has been received
volatile int  DavisRFM69::_rssi;                                   // RSSI measured immediately after payload reception
volatile byte DavisRFM69::_mode;                                   // current transceiver state

DavisRFM69* DavisRFM69::selfPointer;

/************************************************************
 * Init receiver 
 ************************************************************/
void DavisRFM69::init(void) {
  const byte CONFIG[][2] = {
    /* 0x01 */ { REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY },
    /* 0x02 */ { REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_10 }, // Davis uses Gaussian shaping with BT=0.5
    /* 0x03 */ { REG_BITRATEMSB, RF_BITRATEMSB_19200}, // Davis uses a datarate of 19.2 KBPS
    /* 0x04 */ { REG_BITRATELSB, RF_BITRATELSB_19200},
    /* 0x05 */ { REG_FDEVMSB, RF_FDEVMSB_4800}, // Davis uses a deviation of 4.8 kHz
    /* 0x06 */ { REG_FDEVLSB, RF_FDEVLSB_4800},
    // 0x07      REG_FRFMSB  No sense setting here. Done in main routine.
    // 0x08      REG_FRF     No sense setting here. Done in main routine. 
    // 0x09      REG_FRFSB.  No sense setting here. Done in main routine. 
    /* 0x0B */ { REG_AFCCTRL, RF_AFCLOWBETA_OFF }, // TODO: Should use LOWBETA_ON, but having trouble getting it working
    //           looks like PA1 and PA2 are not implemented on RFM69W, hence the max output power is 13dBm
    //             +17dBm and +20dBm are possible on RFM69HW
    //             +13dBm formula: Pout=-18+OutputPower (with PA0 or PA1**)
    //             +17dBm formula: Pout=-14+OutputPower (with PA1 and PA2)**
    //             +20dBpaym formula: Pout=-11+OutputPower (with PA1 and PA2)** 
    //                                and high power PA settings (section 3.3.7 in datasheet)
    // 0x11    { REG_PALEVEL, RF_PALEVEL_PA0_ON | RF_PALEVEL_PA1_OFF | RF_PALEVEL_PA2_OFF | RF_PALEVEL_OUTPUTPOWER_11111},
    // 0x13    { REG_OCP, RF_OCP_ON | RF_OCP_TRIM_95 }, //over current protection (default is 95mA)
    /* 0x18 */ { REG_LNA, RF_LNA_ZIN_50 | RF_LNA_GAINSELECT_AUTO}, // Not sure which is correct!
    // RXBW defaults are { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_24 | RF_RXBW_EXP_5} (RxBw: 10.4khz)
    /* 0x19 */ { REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4 }, // Use 25 kHz BW (BitRate < 2 * RxBw)
    /* 0x1A */ { REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3 }, // Use double the bandwidth during AFC as reception
    /* 0x1B - 0x1D These registers are for OOK.  Not used */
    /* 0x1E */ { REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON },
    // 0x1F      AFC MSB
    // 0x20      AFC LSB
    // 0x21      FEI MSB
    // 0x22      FEI LSB
    // 0x23      RSSI MSB 
    // 0x24      RSSI LSB values
    /* 0x25 */ { REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01 }, //DIO0 is the only IRQ we're using
    // 0x26      RegDioMapping2
    // 0x27      RegIRQFlags1
    /* 0x28 */ { REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN }, // Reset the FIFOs. Fixes a problem I had with bad first packet.
    /* 0x29 */ { REG_RSSITHRESH, 0xA0 }, //must be set to dBm = (-Sensitivity / 2) - default is 0xE4=228 so -114dBm
    // 0x2a      RegRxTimeout1
    // 0x2b      RegRxTimeout2
    // 0x2c      RegPreambleMsb - use zero default */
    /* 0x2d */ { REG_PREAMBLELSB, 4 }, // Davis has four preamble bytes 0xAAAAAAAA
    /* 0x2e */ { REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_2 },  // Allow a couple erros in the sync word
    /* 0x2f */ { REG_SYNCVALUE1, 0xcb }, // Davis ISS first sync byte. http://madscientistlabs.blogspot.ca/2012/03/first-you-get-sugar.html
    /* 0x30 */ { REG_SYNCVALUE2, 0x89 }, // Davis ISS second sync byte.
    // 0x31      REG_SYNCVALUE3 not used 
    // 0x32      REG_SYNCVALUE4 not used 
    // 0x33      REG_SYNCVALUE5 not used 
    // 0x34      REG_SYNCVALUE6 not used 
    // 0x35      REG_SYNCVALUE7 not used 
    // 0x36      REG_SYNCVALUE8 not used 
    /* 0x37 */ { REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF }, // Fixed packet length and we'll check our own CRC
    /* 0x38 */ { REG_PAYLOADLENGTH, DAVIS_PACKET_LEN }, // Davis sends 8 bytes of payload, including CRC that we check manually.
    // 0x39    { REG_NODEADRS, nodeID }, // Turned off because we're not using address filtering
    // 0x3a    { REG_BROADCASTADRS, RF_BROADCASTADDRESS_VALUE }, // Not using this
    // 0x3b      REG_AUTOMODES - Automatic modes are not used in this implementation.
    /* 0x3c */ { REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFOTHRESH | 0x07 }, // TX on FIFO having more than seven bytes
    /* 0x3d */ { REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF }, //RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x3e - 0x4d  AES Key not used in this implementation */
    /* 0x6F */ { REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0 }, // // TODO: Should use LOWBETA_ON, but having trouble getting it working
    /* 0x71 */ { REG_TESTAFC, 0 }, // AFC Offset for low mod index systems
    {255, 0}
  };
  
  // init SPI
  pinMode(_slaveSelectPin, OUTPUT);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2); //max speed, except on Due which can run at system clock speed
  SPI.begin();

  // sync
  do {
    writeReg(REG_SYNCVALUE1, 0xaa);
  } while (readReg(REG_SYNCVALUE1) != 0xaa);
  do {
     writeReg(REG_SYNCVALUE1, 0x55); 
  } while (readReg(REG_SYNCVALUE1) != 0x55);

  // send config
  for (byte i = 0; CONFIG[i][0] != 255; i++) {
    writeReg(CONFIG[i][0], CONFIG[i][1]);
  }
  
  // Standby
  setMode(RF69_MODE_STANDBY);
  
  // Wait for ModeReady
  while ((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); 
  
  // Register ISR
  pinMode(_interruptPin, INPUT);
  attachInterrupt(_interruptPin, DavisRFM69::isr0, RISING);
  
  // myself
  selfPointer = this;
}


/************************************************************
 * RFM Receive Packet Interrupt Handler 
 * - read RSSI
 * - get data received to _data Buffer
 ************************************************************/
void DavisRFM69::interruptHandler(void) {
  _rssi = readRSSI();  // Read up front when it is most likely the carrier is still up  
  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {    
    setMode(RF69_MODE_STANDBY);        
    select();    // Select RFM69 module, disable interrupts
    // get data received
    SPI.transfer(REG_FIFO & 0x7f);
    for (byte i = 0; i < DAVIS_PACKET_LEN; i++){
      _data[i] = reverseBits(SPI.transfer(0));      
    } 
    _packetReceived = true;    
    _hasCrcError = false;
    unselect();  // Unselect RFM69 module, enable interrupts
  }  
}


/************************************************************
 * Set Channel 
 * - and activate receiver
 ************************************************************/
void DavisRFM69::setChannel(byte channel) {
  _channel = channel;
  if (_channel > DAVIS_FREQ_TABLE_LENGTH - 1) _channel = 0;
  writeReg(REG_FRFMSB, pgm_read_byte(&FRF[_channel][0]));
  writeReg(REG_FRFMID, pgm_read_byte(&FRF[_channel][1]));
  writeReg(REG_FRFLSB, pgm_read_byte(&FRF[_channel][2]));
  receiveBegin();
}


/************************************************************
 * Hop to next Channel and activate receiver 
 ************************************************************/
void DavisRFM69::hop(void) {
  setChannel(++_channel);
}


/************************************************************
 * Reverse bits in a byte
 * - The data bytes come over the air from the ISS have 
 *   least significant bit first. 
 * - from http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
 * @param[in] b value to be reversed
 * @return    reversed value
 ************************************************************/
byte DavisRFM69::data(byte index) {
  byte v;
  if ((index < 0) || (index > 7)) {
    // no valid index
    v = 0xff;
  } else {
    v = _data[index];
  }
  return(v);
}

/************************************************************
 * Reverse bits in a byte
 * - The data bytes come over the air from the ISS have 
 *   least significant bit first. 
 * - from http://www.ocf.berkeley.edu/~wwu/cgi-bin/yabb/YaBB.cgi?board=riddles_cs;action=display;num=1103355188
 * @param[in] b value to be reversed
 * @return    reversed value
 ************************************************************/
byte DavisRFM69::reverseBits(byte b) {
  b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4);
  b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2);
  b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1);
  return(b);
}

/************************************************************
 * get CRC16
 * @return CRC value for packet in receive buffer 
 ************************************************************/
uint16_t DavisRFM69::crc16(void) {
  // First 6 Bate 
  // unsigned int crc;
  // crc = compute_crc16(_data, 6);
  return compute_crc16(_data, 6);
}

/************************************************************
 * rssi
 * @return RSSI measured immediately after payload reception
 ************************************************************/
int DavisRFM69::rssi(void) {  
  return _rssi;
}

/************************************************************
 * channel
 * @return current channel
 ************************************************************/
byte DavisRFM69::channel(void) {  
  return _channel;
}



/************************************************************
 * Davis CRC calculation
 * - from http://www.menie.org/georges/embedded/
 * - changed to not support othe start value as 0
 * @param[in] buf pointer to buffer 
 * @param[in] len legth of buffer 
 ************************************************************/
uint16_t DavisRFM69::compute_crc16(volatile byte *buf, byte len){  
  unsigned int crc = 0;
  while (len--) {
    int i;
    crc ^= *(char *)buf++ << 8;
    for( i = 0; i < 8; ++i ) {
      if( crc & 0x8000 )
        crc = (crc << 1) ^ 0x1021;
      else
        crc = crc << 1;
    }
  }
  return crc;
}


/************************************************************
 * Write Frequency
 * - Write FRF to RFM69 Register
 * - the 3 FRF Bytes are stored in one uint32_t value
 * @param[in] FRF value to be written
 ************************************************************/
void DavisRFM69::setFrequency(uint32_t FRF) {
  writeReg(REG_FRFMSB, FRF >> 16);
  writeReg(REG_FRFMID, FRF >> 8);
  writeReg(REG_FRFLSB, FRF);
}


/************************************************************
 * Set RFM69 Mode  
 * - TX and SYNTH not needed to receive data from ISS
 * @param[in] mode  New mode, can be RX, STANDBY, SLEEP
 ************************************************************/
void DavisRFM69::setMode(byte mode) {  
  if (mode == _mode) return;
  _mode = mode;  
  switch (_mode) {
    case RF69_MODE_TX:
      // Not supported      
      // writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_TRANSMITTER);      
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_RECEIVER);      
      break;
    case RF69_MODE_SYNTH:
      // Not supported
      // writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (readReg(REG_OPMODE) & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default: return;
  }
  // we are using packet mode, so this check is not really needed
  // but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
  while (_mode == RF69_MODE_SLEEP && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00); // Wait for ModeReady
}

/************************************************************
 * Set Mode to Sleep
 ************************************************************/
void DavisRFM69::sleep(void) {
  setMode(RF69_MODE_SLEEP);
}

/************************************************************
 * Packet Receive ISR
 ************************************************************/
void DavisRFM69::isr0(void) { 
  selfPointer->interruptHandler(); 
}


/************************************************************
 * Activate Receiver 
 ************************************************************/
void DavisRFM69::receiveBegin(void) {
  _packetReceived = false;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY){
    writeReg(REG_PACKETCONFIG2, (readReg(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART); // avoid RX deadlocks
  }
  // set DIO0 to "PAYLOADREADY" in receive mode
  writeReg(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01);                 
  setMode(RF69_MODE_RX);
}

/************************************************************
 * receiveDone
 ************************************************************
 * @return  true if packet received since last channel hop
 ************************************************************/
bool DavisRFM69::receiveDone(void) {
  return _packetReceived;
}

/************************************************************
 * mark current packet has CRC Error
  ************************************************************/
void DavisRFM69::markCrcError(void) {
  _hasCrcError = true;
}
 
/************************************************************
 * get CRC Error State of current packet 
 ************************************************************/
boolean DavisRFM69::getCrcError() {
  return _hasCrcError;
}


/************************************************************
 * Read actual RSSI
 ************************************************************
 * @return          RSSI value
 ************************************************************/
int DavisRFM69::readRSSI(void) {
  int rssi;  
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}


/************************************************************
 * read from RFM Register
 ************************************************************
 * @param[in] addr  address of register
 * @return          registervalue 
 ************************************************************/
byte DavisRFM69::readReg(byte addr) {
  select();
  SPI.transfer(addr & 0x7F);
  byte regval = SPI.transfer(0);
  unselect();
  return regval;
}


/************************************************************
 * write to RFM Register
 ************************************************************
 * @param[in] addr  address of register
 * @param[in] value value tha ha to be written
 ************************************************************/
void DavisRFM69::writeReg(byte addr, byte value) {
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}


/************************************************************
 * SPI select RFM69
 ************************************************************
 * - disable Interrupts
 * - Select RFM (SS: low) 
 ************************************************************/
void DavisRFM69::select() {
  noInterrupts();
  digitalWrite(_slaveSelectPin, LOW);
}


/************************************************************
 * SPI unselect RFM69
 ************************************************************
 * - Unselect RFM (SS: high) 
 * - enable Interrupts
 ************************************************************/
void DavisRFM69::unselect() {
  digitalWrite(_slaveSelectPin, HIGH);
  interrupts();
}


/************************************************************
 * read all RFM registers for debugging
 ************************************************************
 * Attention: This functions prints directly to serial port
 ************************************************************/
void DavisRFM69::readAllRegs() {
  byte regVal;
  for (byte regAddr = 1; regAddr <= 0x4F; regAddr++) {
    select();
    SPI.transfer(regAddr & 0x7f); // send address + r/w bit
    regVal = SPI.transfer(0);
    unselect();
    Serial.print(regAddr, HEX);
    Serial.print(" - ");
    Serial.print(regVal,HEX);
    Serial.print(" - ");
    Serial.println(regVal,BIN);
  }
  unselect();
}


/************************************************************
 * read Chip Temperature
 ************************************************************
 * Attention: Mode is set to Standby afterwards
 ************************************************************
 * @param[in] calFactor correction factor, rising temp = rising val
 * @return    Temperature in centigrade
 ************************************************************/
byte DavisRFM69::readTemperature(byte calFactor) { 
  setMode(RF69_MODE_STANDBY);
  writeReg(REG_TEMP1, RF_TEMP1_MEAS_START);
  while ((readReg(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING)) Serial.print('*');
  return ~readReg(REG_TEMP2) + calFactor; 
}                                                           


/************************************************************
 * Do RC-Calibration
 ************************************************************/ 
void DavisRFM69::rcCalibration() {
  writeReg(REG_OSC1, RF_OSC1_RCCAL_START);
  while ((readReg(REG_OSC1) & RF_OSC1_RCCAL_DONE) == 0x00);
}


/************************************************************
 * Set Modul to Standby
 ************************************************************/ 
void DavisRFM69::standby(void) {
  setMode(RF69_MODE_STANDBY);  
}                            



