#ifndef CC1101_H	/* prevent circular inclusions */
#define CC1101_H	/* by using protection macros */

#include "xspips.h"

//***************************************CC1101 define**************************************************//
// CC1101 CONFIG REGISTER
#define CC1101_IOCFG2       0x00        // GDO2 output pin configuration
#define CC1101_IOCFG1       0x01        // GDO1 output pin configuration
#define CC1101_IOCFG0       0x02        // GDO0 output pin configuration
#define CC1101_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1        0x04        // Sync word, high INT8U
#define CC1101_SYNC0        0x05        // Sync word, low INT8U
#define CC1101_PKTLEN       0x06        // Packet length
#define CC1101_PKTCTRL1     0x07        // Packet automation control
#define CC1101_PKTCTRL0     0x08        // Packet automation control
#define CC1101_ADDR         0x09        // Device address
#define CC1101_CHANNR       0x0A        // Channel number
#define CC1101_FSCTRL1      0x0B        // Frequency synthesizer control
#define CC1101_FSCTRL0      0x0C        // Frequency synthesizer control
#define CC1101_FREQ2        0x0D        // Frequency control word, high INT8U
#define CC1101_FREQ1        0x0E        // Frequency control word, middle INT8U
#define CC1101_FREQ0        0x0F        // Frequency control word, low INT8U
#define CC1101_MDMCFG4      0x10        // Modem configuration
#define CC1101_MDMCFG3      0x11        // Modem configuration
#define CC1101_MDMCFG2      0x12        // Modem configuration
#define CC1101_MDMCFG1      0x13        // Modem configuration
#define CC1101_MDMCFG0      0x14        // Modem configuration
#define CC1101_DEVIATN      0x15        // Modem deviation setting
#define CC1101_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CC1101_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CC1101_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CC1101_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CC1101_BSCFG        0x1A        // Bit Synchronization configuration
#define CC1101_AGCCTRL2     0x1B        // AGC control
#define CC1101_AGCCTRL1     0x1C        // AGC control
#define CC1101_AGCCTRL0     0x1D        // AGC control
#define CC1101_WOREVT1      0x1E        // High INT8U Event 0 timeout
#define CC1101_WOREVT0      0x1F        // Low INT8U Event 0 timeout
#define CC1101_WORCTRL      0x20        // Wake On Radio control
#define CC1101_FREND1       0x21        // Front end RX configuration
#define CC1101_FREND0       0x22        // Front end TX configuration
#define CC1101_FSCAL3       0x23        // Frequency synthesizer calibration
#define CC1101_FSCAL2       0x24        // Frequency synthesizer calibration
#define CC1101_FSCAL1       0x25        // Frequency synthesizer calibration
#define CC1101_FSCAL0       0x26        // Frequency synthesizer calibration
#define CC1101_RCCTRL1      0x27        // RC oscillator configuration
#define CC1101_RCCTRL0      0x28        // RC oscillator configuration
#define CC1101_FSTEST       0x29        // Frequency synthesizer calibration control
#define CC1101_PTEST        0x2A        // Production test
#define CC1101_AGCTEST      0x2B        // AGC test
#define CC1101_TEST2        0x2C        // Various test settings
#define CC1101_TEST1        0x2D        // Various test settings
#define CC1101_TEST0        0x2E        // Various test settings

//CC1101 Strobe commands, single byte only
#define CC1101_SRES         0x30        // Reset chip.
#define CC1101_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CC1101_SXOFF        0x32        // Turn off crystal oscillator.
#define CC1101_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CC1101_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CC1101_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CC1101_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CC1101_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CC1101_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CC1101_SWORRST      0x3C        // Reset real time clock.
#define CC1101_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // INT8Us for simpler software.

//CC1101 STATUS REGISTER, burst read only
#define CC1101_PARTNUM      0x30
#define CC1101_VERSION      0x31
#define CC1101_FREQEST      0x32
#define CC1101_LQI          0x33
#define CC1101_RSSI         0x34
#define CC1101_MARCSTATE    0x35
#define CC1101_WORTIME1     0x36
#define CC1101_WORTIME0     0x37
#define CC1101_PKTSTATUS    0x38
#define CC1101_VCO_VC_DAC   0x39
#define CC1101_TXBYTES      0x3A
#define CC1101_RXBYTES      0x3B

//CC1101 PATABLE,TXFIFO,RXFIFO
#define CC1101_PATABLE      0x3E
#define CC1101_TXFIFO       0x3F		// Only for write operation
#define CC1101_RXFIFO       0x3F		// Only for read operation

//CC1101 Other constants
#define	  MAX_DATA       	60
#define	CC1101_CRYSTAL_FREQ	26


extern volatile int TransferInProgress;
extern int Error;
/****************CC1101 functions****************/
void CC1101_setInterrupt(u8 interruptBool);
void CC1101_init(XSpiPs *SpiPtr);
void CC1101_writeReg(XSpiPs *SpiPtr, u8 addr, u8 value);
void CC1101_writeBurstReg(XSpiPs *SpiPtr, u8 addr, u8* buffer, u8 num);
u8	 CC1101_writeStrobe(XSpiPs *SpiPtr, u8 addr);
u8	 CC1101_readReg(XSpiPs *SpiPtr, u8 addr);
void CC1101_readBurstReg(XSpiPs *SpiPtr, u8 addr, u8* buffer, u8 num);
u8	 CC1101_readStatus(XSpiPs *SpiPtr, u8 addr);
void CC1101_setCCMode(XSpiPs *SpiPtr, u8 s);
void CC1101_setModulation(XSpiPs *SpiPtr, u8 m);
void CC1101_setMHZ(XSpiPs *SpiInstancePtr, float mhz);
void CC1101_setPA(XSpiPs *SpiPtr, int p);
void CC1101_setTX(XSpiPs* SpiPtr);
void CC1101_setRX(XSpiPs* SpiPtr);
void CC1101_sendData(XSpiPs* SpiPtr, u8 *txBuffer, u8 size);
u8	 CC1101_receiveData(XSpiPs* SpiPtr, u8 *rxBuffer);

void CC1101_setSyncMode(XSpiPs* SpiPtr, u8 v);
void CC1101_setCRC(XSpiPs* SpiPtr, u8 v);
void CC1101_setDRate(XSpiPs* SpiPtr, double d);
u8	 CC1101_checkReceiveFlag(XSpiPs* SpiPtr);
u8	 CC1101_getRssi(XSpiPs* SpiPtr);
void CC1101_setFEC(XSpiPs* SpiPtr, u8 v);
void CC1101_setLengthConfig(XSpiPs* SpiPtr, u8 v);
void CC1101_setPacketLength(XSpiPs* SpiPtr, u8 l);

/****************Interrupt setup****************/
//void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount);

#endif	/* CC1101_H */ /* end of protection macro */
