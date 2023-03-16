// Driver definition for HopeRF RFM69W/RFM69HW, Semtech SX1231/1231H used for
// compatibility with the frequency hopped, spread spectrum signals from a Davis Instrument
// wireless Integrated Sensor Suite (ISS)
//
// This is part of the DavisRFM69 library from https://github.com/dekay/DavisRFM69
// (C) DeKay 2014 dekaymail@gmail.com
//
// As I consider this to be a derived work from the RFM69W library from LowPowerLab,
// it is released under the same Creative Commons Attrib Share-Alike License
// You are free to use/extend this library but please abide with the CC-BY-SA license:
// http://creativecommons.org/licenses/by-sa/3.0/

#ifndef DAVISRFM69_h
#define DAVISRFM69_h

// Uncomment ONE AND ONLY ONE of the four #define's below.  This determines the
// frequency table the code will use.  Note that only the US (actually North
// America) and EU frequencies are defined at this time.  Australia and New
// Zealand are placeholders.  Note however that the frequencies for AU and NZ
// are not known at this time.
//#define DAVIS_FREQS_US
#define DAVIS_FREQS_EU
//#define DAVIS_FREQS_AU
//#define DAVIS_FREQS_NZ

#include <Arduino.h>            //assumes Arduino IDE v1.0 or greater

#define DAVIS_PACKET_LEN      8 // ISS has fixed packet lengths of eight bytes, including CRC
#define RF69_PIN_CS           5 // SS connected to this pin:   ESP32 GPIO 5
#define RF69_PIN_IRQ          2 // DIO0 connected to this pin: ESP32 GPIO 2
#define RF69_MODE_SLEEP       0 // XTAL OFF
#define RF69_MODE_STANDBY     1 // XTAL ON
#define RF69_MODE_SYNTH       2 // PLL ON
#define RF69_MODE_RX          3 // RX MODE
#define RF69_MODE_TX          4 // TX MODE

class DavisRFM69 {
  public:    
    // constructor
    DavisRFM69(byte slaveSelectPin=RF69_PIN_CS, byte interruptPin=RF69_PIN_IRQ) {       
      _slaveSelectPin = slaveSelectPin;
      _interruptPin = interruptPin;            
      _mode = RF69_MODE_STANDBY;
      _packetReceived = false;
      _hasCrcError = false;
    }    
    // functions
    uint16_t crc16(void);                                                   // get crc value from last received packet    
    byte channel(void);                                                     // get actual channel 
    byte data(byte index);                                                  // read byte of receiver data
    bool receiveDone();                                                     // getter for _packetReceived       
    void markCrcError(void);                                                // mark current packet has CRC Error
    boolean getCrcError(void);                                              // get CRC Error State of current packet 
    void setChannel(byte channel);                                          // set current channel
    void hop();                                                             // hot to next channel        
    void init();                                                            // initialize the chip                
    byte readTemperature(byte calFactor=0);                                 // get CMOS temperature (8bit)    
    void rcCalibration(); //calibrate the internal RC oscillator for use in wide temperature variations - see datasheet section [4.3.5. RC Timer Accuracy]    
    void readAllRegs();                                                     // allow debugging registers    
    int  rssi();                                                            // get RSSI measured immediately after payload reception
    void sleep();                                                           // Switch Mode to Sleep
    void standby();                                                         // Switch Mode to Standby
  
  protected:
    // vars    
    static volatile byte _data[DAVIS_PACKET_LEN];  // receive buffer    
    static volatile byte _channel;                 // actual channel 
    static volatile bool _hasCrcError;               // received packet has been transfered to the user
    static volatile bool _packetReceived;          // a Packet has been received    
    static volatile int  _rssi;                    // RSSI measured immediately after payload reception
    static volatile byte _mode;                                             // mode (sleep, Standby, Synth, RX or TX) 
    byte _slaveSelectPin;
    byte _interruptPin;    
    // functions    
    uint16_t compute_crc16(volatile byte *buf, byte len);                   // calculate the crc value 
    static DavisRFM69* selfPointer;
    int  readRSSI();                                                        // get RSSI
    void virtual interruptHandler();
    static void isr0();
    byte readReg(byte addr);void receiveBegin();
    byte reverseBits(byte b);
    void select();
    void setFrequency(uint32_t FRF);                                        // set Frequency 
    void setMode(byte mode);    
    void unselect();    
    void writeReg(byte addr, byte val);
};

// FRF_MSB, FRF_MID, and FRF_LSB for the 51 North American channels & 5 European channels.
// used by Davis in frequency hopping

#ifdef DAVIS_FREQS_US
#warning ** USING NORTH AMERICAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 51
static const uint8_t FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xE3, 0xDA, 0x7C},
  {0xE1, 0x98, 0x71},
  {0xE3, 0xFA, 0x92},
  {0xE6, 0xBD, 0x01},
  {0xE4, 0xBB, 0x4D},
  {0xE2, 0x99, 0x56},
  {0xE7, 0x7D, 0xBC},
  {0xE5, 0x9C, 0x0E},
  {0xE3, 0x39, 0xE6},
  {0xE6, 0x1C, 0x81},
  {0xE4, 0x5A, 0xE8},
  {0xE1, 0xF8, 0xD6},
  {0xE5, 0x3B, 0xBF},
  {0xE7, 0x1D, 0x5F},
  {0xE3, 0x9A, 0x3C},
  {0xE2, 0x39, 0x00},
  {0xE4, 0xFB, 0x77},
  {0xE6, 0x5C, 0xB2},
  {0xE2, 0xD9, 0x90},
  {0xE7, 0xBD, 0xEE},
  {0xE4, 0x3A, 0xD2},
  {0xE1, 0xD8, 0xAA},
  {0xE5, 0x5B, 0xCD},
  {0xE6, 0xDD, 0x34},
  {0xE3, 0x5A, 0x0A},
  {0xE7, 0x9D, 0xD9},
  {0xE2, 0x79, 0x41},
  {0xE4, 0x9B, 0x28},
  {0xE5, 0xDC, 0x40},
  {0xE7, 0x3D, 0x74},
  {0xE1, 0xB8, 0x9C},
  {0xE3, 0xBA, 0x60},
  {0xE6, 0x7C, 0xC8},
  {0xE4, 0xDB, 0x62},
  {0xE2, 0xB9, 0x7A},
  {0xE5, 0x7B, 0xE2},
  {0xE7, 0xDE, 0x12},
  {0xE6, 0x3C, 0x9D},
  {0xE3, 0x19, 0xC9},
  {0xE4, 0x1A, 0xB6},
  {0xE5, 0xBC, 0x2B},
  {0xE2, 0x18, 0xEB},
  {0xE6, 0xFD, 0x42},
  {0xE5, 0x1B, 0xA3},
  {0xE3, 0x7A, 0x2E},
  {0xE5, 0xFC, 0x64},
  {0xE2, 0x59, 0x16},
  {0xE6, 0x9C, 0xEC},
  {0xE2, 0xF9, 0xAC},
  {0xE4, 0x7B, 0x0C},
  {0xE7, 0x5D, 0x98}
};
#elif defined (DAVIS_FREQS_EU)
// #warning ** USING EUROPEAN FREQUENCY TABLE **
#define DAVIS_FREQ_TABLE_LENGTH 5
// static const uint8_t __attribute__ ((progmem)) FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
static const uint8_t FRF[DAVIS_FREQ_TABLE_LENGTH][3] =
{
  {0xD9, 0x04, 0x45},
  {0xD9, 0x13, 0x04},
  {0xD9, 0x21, 0xC2},
  {0xD9, 0x0B, 0xA4},
  {0xD9, 0x1A, 0x63}
};
#elif defined (DAVIS_FREQS_AU)
#error ** ERROR DAVIS FREQS FOR AU ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#elif defined (DAVIS_FREQS_NZ)
#error ** ERROR DAVIS FREQS FOR NZ ARE NOT KNOWN AT THIS TIME. ONLY US & EU DEFINED **
#else
#error ** ERROR DAVIS_FREQS MUST BE DEFINED AS ONE OF _US, _EU, _AZ, or NZ **
#endif  // DAVIS_FREQS

// For the packet stats structure used in response to the RXCHECK command


#endif  // DAVISRFM_h
