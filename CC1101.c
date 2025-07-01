#include "CC1101.h"

#include "xil_printf.h"

#include "sleep.h"
#include <math.h>

/************************** Constant Definitions *****************************/

#define		WRITE_SINGLE		0x00		//write single not used because it is 0x00
#define		WRITE_BURST			0x40		//write burst
#define		READ_SINGLE			0x80		//read single
#define		READ_BURST			0xC0		//read burst
#define		BYTES_IN_RXFIFO		0x7F		//byte number in RXfifo


#define		ADDRESS_OFFSET		0		// Address to read or write too
#define		DATA_OFFSET			1		// Start of data to read or write



/***************** Macros (Inline Functions) Definitions *********************/

#define map(x, in_min, in_max, out_min, out_max)(((x) - (in_min)) * ((out_max) - (out_min)) / ((in_max) - (in_min)) + (out_min))



/************************** Variable Definitions *****************************/

u8 modulation = 2;
u8 frend0;
u8 chan = 0;
int pa = 12;
u8 last_pa;
u8 GDO0;
u8 GDO2;
u8 gdo_set=0;
u8 spi = 0;
u8 ccmode = 0;
float MHz = 433.92;
u8 m4RxBw = 0;
u8 m4DaRa;
u8 m2DCOFF;
u8 m2MODFM;
u8 m2MANCH;
u8 m2SYNCM;
u8 m1FEC;
u8 m1PRE;
u8 m1CHSP;
u8 pc1PQT;
u8 pc1CRC_AF;
u8 pc1APP_ST;
u8 pc1ADRCHK;
u8 pc0WDATA;
u8 pc0PktForm;
u8 pc0CRC_EN;
u8 pc0LenConf;
u8 trxstate = 0;
u8 clb1[2]= {24,28};
u8 clb2[2]= {31,38};
u8 clb3[2]= {65,76};
u8 clb4[2]= {77,79};

//volatile char TransferInProgress = FALSE;
volatile int TransferInProgress = FALSE;
int Error = 0;
u8 interrupt = FALSE;

u8 writeBuffer[MAX_DATA + DATA_OFFSET];
u8 readBuffer [MAX_DATA + DATA_OFFSET /*+ DUMMY_SIZE*/];

/****************************************************************/
u8 PA_TABLE[8]      = {0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00};
//                          -30  -20  -15  -10   0    5    7    10
uint8_t PA_TABLE_315[8]  = {0x12,0x0D,0x1C,0x34,0x51,0x85,0xCB,0xC2,};             //300 - 348
uint8_t PA_TABLE_433[8]  = {0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0,};             //387 - 464
//                          -30  -20  -15  -10  -6    0    5    7    10   12
uint8_t PA_TABLE_868[10] = {0x03,0x17,0x1D,0x26,0x37,0x50,0x86,0xCD,0xC5,0xC0,};  //779 - 899.99
//                          -30  -20  -15  -10  -6    0    5    7    10   11
uint8_t PA_TABLE_915[10] = {0x03,0x0E,0x1E,0x27,0x38,0x8E,0x84,0xCC,0xC3,0xC0,};  //900 - 928




/********************** Static function declarations *************************/

static void CC1101_calibrate(XSpiPs *SpiPtr);
static void CC1101_split_PKTCTRL1(XSpiPs* SpiPtr);
static void CC1101_split_PKTCTRL0(XSpiPs* SpiPtr);
static void CC1101_split_MDMCFG4(XSpiPs* SpiPtr);
static void CC1101_split_MDMCFG2(XSpiPs* SpiPtr);
static void CC1101_split_MDMCFG1(XSpiPs* SpiPtr);
static void CC1101_regConfigSettings(XSpiPs* SpiPtr);
static u8 CC1101_getState(XSpiPs* SpiPtr);




/************************** Function definitions *****************************/

/******************************************************************************
*
* This function sets the mode for XSpiPS transfer. The default mode is polled.
*
* @param	interruptBool, TRUE for using interrupt and FALSE for using polling
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setInterrupt(u8 interruptBool){
	interrupt = interruptBool;
}

/******************************************************************************
*
* This function initializes the CC1101 before first use.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_init(XSpiPs *SpiPtr){
	CC1101_regConfigSettings(SpiPtr);
}


/******************************************************************************
*
* This function writes a value to an address on to CC1101 over the SPI interface.
* See the datasheet for all the addresses to write to.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address to write data.
* @param	value contains the value to write to the address.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_writeReg(XSpiPs *SpiPtr, u8 addr, u8 value){
	u8 tempBuffer[]={addr,value};

	if (interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, tempBuffer, NULL, sizeof(tempBuffer));

		int i=0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, tempBuffer, NULL, sizeof(tempBuffer));
	}
}


/******************************************************************************
*
* This function writes a buffer to an address on to CC1101 over the SPI interface.
* See the datasheet for all the addresses to write to.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address to write data to.
* @param	buffer is a pointer to the buffer to write to the address.
* @param	num is the length of the buffer.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_writeBurstReg(XSpiPs *SpiPtr, u8 addr, u8* buffer, u8 num){
	writeBuffer[ADDRESS_OFFSET] = (addr | WRITE_BURST);
	memcpy(writeBuffer+DATA_OFFSET, buffer, num);

	if(interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, writeBuffer, NULL, num+DATA_OFFSET);

		int i=0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, writeBuffer, NULL, num+DATA_OFFSET);
	}
}


/******************************************************************************
*
* This function writes to a strobe address on to CC1101 over the SPI interface.
* See the datasheet for all the strobe addresses to write to.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address to write data to.
*
* @return	Chip Status Byte: Bits (6:4) comprise the STATE value and (3:0) contains the FIFO_BYTES_AVAILABLE.
*
* @note		None.
*
******************************************************************************/
u8 CC1101_writeStrobe(XSpiPs *SpiPtr, u8 addr){
	u8 tempBuffer=addr;

	if (interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, &tempBuffer, &tempBuffer, sizeof(tempBuffer));

		int i=0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, &tempBuffer, &tempBuffer, sizeof(tempBuffer));
	}
	return tempBuffer;
}


/******************************************************************************
*
* This function reads the register for the specified address on to CC1101 over the SPI interface.
* See the datasheet for all the registers to read from.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address of the register.
*
* @return	Value from the specified register.
*
* @note		None.
*
******************************************************************************/
u8 CC1101_readReg(XSpiPs *SpiPtr, u8 addr){
	u8 tempBuffer[]={(addr|READ_SINGLE),0};

	if (interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, tempBuffer, tempBuffer, sizeof(tempBuffer));

		int i = 0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, tempBuffer, tempBuffer, sizeof(tempBuffer));
	}

	return tempBuffer[DATA_OFFSET];
}


/******************************************************************************
*
* This function reads the register for the specified address into a buffer on to CC1101 over the SPI interface.
* See the datasheet for all the registers to read from.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address of the register.
* @param	buffer is a pointer to the buffer where the readdata should be stored.
* @param	num is the length of the data to read.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_readBurstReg(XSpiPs *SpiPtr, u8 addr, u8* buffer, u8 num){
	writeBuffer[ADDRESS_OFFSET] = (addr | READ_BURST);

	if (interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, writeBuffer, readBuffer, num);

		int i = 0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, writeBuffer, readBuffer, num+DATA_OFFSET);
	}

	memcpy(buffer, readBuffer+DATA_OFFSET, num+DATA_OFFSET);
}


/******************************************************************************
*
* This function read a value from an address on to CC1101 over the SPI interface.
* See the datasheet for all the addresses to read from.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	addr contains the address to read data from.
*
* @return	Value from the specified register.
*
* @note		None.
*
******************************************************************************/
u8 CC1101_readStatus(XSpiPs *SpiPtr, u8 addr){
	u8 tempBuffer[] = {(addr | READ_BURST),0};

	if (interrupt){
		TransferInProgress = TRUE;
		XSpiPs_Transfer(SpiPtr, tempBuffer, tempBuffer, sizeof(tempBuffer));

		int i = 0;
		while (TransferInProgress){
			i++;
			if (i>1000000){
				xil_printf("transfer in progress failed \r\n");
				break;
			}
		}
	}else{
		XSpiPs_PolledTransfer(SpiPtr, tempBuffer, tempBuffer, sizeof(tempBuffer));
	}

	return tempBuffer[DATA_OFFSET];
}


/******************************************************************************
*
* This function sets the mode for the CC1101 to work in over the SPI interface.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	s selects the format for the RX and TX.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setCCMode(XSpiPs *SpiPtr, u8 s){
	ccmode = s;
	if(ccmode==1){
		CC1101_writeReg(SpiPtr, CC1101_IOCFG2,      0x06);
		CC1101_writeReg(SpiPtr, CC1101_IOCFG0,      0x06);
		CC1101_writeReg(SpiPtr, CC1101_PKTCTRL0,    0x05);
		CC1101_writeReg(SpiPtr, CC1101_MDMCFG3,     0xF8);
		CC1101_writeReg(SpiPtr, CC1101_MDMCFG4,11+m4RxBw);
	}else{
		CC1101_writeReg(SpiPtr, CC1101_IOCFG2,      0x0D);
		CC1101_writeReg(SpiPtr, CC1101_IOCFG0,      0x0D);
		CC1101_writeReg(SpiPtr, CC1101_PKTCTRL0,    0x32);
		CC1101_writeReg(SpiPtr, CC1101_MDMCFG3,     0x93);
		CC1101_writeReg(SpiPtr, CC1101_MDMCFG4, 7+m4RxBw);
	}
	CC1101_setModulation(SpiPtr, modulation);
}


/******************************************************************************
*
* This function sets the mode for the CC1101 to work in over the SPI interface.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	s selects the format for the RX and TX.
*
* @return	None.
*
* @note		0 for 2-FSK, 1 for GFSK, 2 for ASK/OOK, 3 for 4-FSK and 4 for MSK
*
******************************************************************************/
void CC1101_setModulation(XSpiPs *SpiPtr, u8 m){
	if(m>4){m=4;}
	modulation = m;
	CC1101_split_MDMCFG2(SpiPtr);
	switch(m){
	case 0: m2MODFM=0x00; frend0=0x10; break; // 2-FSK
	case 1: m2MODFM=0x10; frend0=0x10; break; // GFSK
	case 2: m2MODFM=0x30; frend0=0x11; break; // ASK
	case 3: m2MODFM=0x40; frend0=0x10; break; // 4-FSK
	case 4: m2MODFM=0x70; frend0=0x10; break; // MSK
	}
	CC1101_writeReg(SpiPtr, CC1101_MDMCFG2, m2DCOFF+m2MODFM+m2MANCH+m2SYNCM);
	CC1101_writeReg(SpiPtr, CC1101_FREND0, frend0);
	CC1101_setPA(SpiPtr, pa);
}




/******************************************************************************
*
* This function sets the base frequency of the CC1101 over the SPI interface.
* See the datasheet for the frequency ranges of the module.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	mhz is the frequency to be set.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setMHZ(XSpiPs *SpiPtr, float mhz){
	u8 freq[3] = {0,0,0};

	MHz = mhz;


	uint32_t f = ((mhz * 65536.0) / CC1101_CRYSTAL_FREQ);
	freq[0]=(f & 0xff);
	freq[1]=((f >> 8) & 0xff);
	freq[2]=((f >> 16) & 0xff);

	CC1101_writeReg(SpiPtr, CC1101_FREQ2, freq[2]);
	CC1101_writeReg(SpiPtr, CC1101_FREQ1, freq[1]);
	CC1101_writeReg(SpiPtr, CC1101_FREQ0, freq[0]);

	CC1101_calibrate(SpiPtr);

}


/******************************************************************************
*
* This function calibrates the frequency  CC1101 over the SPI interface.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_calibrate(XSpiPs *SpiPtr){
	if (MHz >= 300 && MHz <= 348){
		CC1101_writeReg(SpiPtr, CC1101_FSCTRL0, map(MHz, 300, 348, clb1[0], clb1[1]));
		if (MHz < 322.88){CC1101_writeReg(SpiPtr, CC1101_TEST0,0x0B);}
		else{
			CC1101_writeReg(SpiPtr, CC1101_TEST0,0x09);
			int s = CC1101_readStatus(SpiPtr, CC1101_FSCAL2);
			if (s<32){CC1101_writeReg(SpiPtr, CC1101_FSCAL2, s+32);}
			if (last_pa != 1){CC1101_setPA(SpiPtr, pa);}
		}
	}
	else if (MHz >= 378 && MHz <= 464){
		CC1101_writeReg(SpiPtr, CC1101_FSCTRL0, map(MHz, 378, 464, clb2[0], clb2[1]));
		if (MHz < 430.5){CC1101_writeReg(SpiPtr, CC1101_TEST0,0x0B);}
		else{
			CC1101_writeReg(SpiPtr, CC1101_TEST0,0x09);
			int s = CC1101_readStatus(SpiPtr, CC1101_FSCAL2);
			if (s<32){CC1101_writeReg(SpiPtr, CC1101_FSCAL2, s+32);}
			if (last_pa != 2){CC1101_setPA(SpiPtr, pa);}
		}
	}
	else if (MHz >= 779 && MHz <= 899.99){
		CC1101_writeReg(SpiPtr, CC1101_FSCTRL0, map(MHz, 779, 899, clb3[0], clb3[1]));
		if (MHz < 861){CC1101_writeReg(SpiPtr, CC1101_TEST0,0x0B);}
		else{
			CC1101_writeReg(SpiPtr, CC1101_TEST0,0x09);
			int s = CC1101_readStatus(SpiPtr, CC1101_FSCAL2);
			if (s<32){CC1101_writeReg(SpiPtr, CC1101_FSCAL2, s+32);}
			if (last_pa != 3){CC1101_setPA(SpiPtr, pa);}
		}
	}
	else if (MHz >= 900 && MHz <= 928){
		CC1101_writeReg(SpiPtr, CC1101_FSCTRL0, map(MHz, 900, 928, clb4[0], clb4[1]));
		CC1101_writeReg(SpiPtr, CC1101_TEST0,0x09);
		int s = CC1101_readStatus(SpiPtr, CC1101_FSCAL2);
		if (s<32){CC1101_writeReg(SpiPtr, CC1101_FSCAL2, s+32);}
		if (last_pa != 4){CC1101_setPA(SpiPtr, pa);}
	}
}


/******************************************************************************
*
* This function sets the Power Amplification on to CC1101 over the SPI interface.
* See the datasheet for all the addresses to read from.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	p is the desired power amplification in dBm.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setPA(XSpiPs *SpiPtr, int p){
	int a;
	pa = p;

	if (MHz >= 300 && MHz <= 348){
		if (pa <= -30){a = PA_TABLE_315[0];}
		else if (pa > -30 && pa <= -20){a = PA_TABLE_315[1];}
		else if (pa > -20 && pa <= -15){a = PA_TABLE_315[2];}
		else if (pa > -15 && pa <= -10){a = PA_TABLE_315[3];}
		else if (pa > -10 && pa <= 0){a = PA_TABLE_315[4];}
		else if (pa > 0 && pa <= 5){a = PA_TABLE_315[5];}
		else if (pa > 5 && pa <= 7){a = PA_TABLE_315[6];}
		else if (pa > 7){a = PA_TABLE_315[7];}
		last_pa = 1;
	}
	else if (MHz >= 378 && MHz <= 464){
		if (pa <= -30){a = PA_TABLE_433[0];}
		else if (pa > -30 && pa <= -20){a = PA_TABLE_433[1];}
		else if (pa > -20 && pa <= -15){a = PA_TABLE_433[2];}
		else if (pa > -15 && pa <= -10){a = PA_TABLE_433[3];}
		else if (pa > -10 && pa <= 0){a = PA_TABLE_433[4];}
		else if (pa > 0 && pa <= 5){a = PA_TABLE_433[5];}
		else if (pa > 5 && pa <= 7){a = PA_TABLE_433[6];}
		else if (pa > 7){a = PA_TABLE_433[7];}
		last_pa = 2;
	}
	else if (MHz >= 779 && MHz <= 899.99){
		if (pa <= -30){a = PA_TABLE_868[0];}
		else if (pa > -30 && pa <= -20){a = PA_TABLE_868[1];}
		else if (pa > -20 && pa <= -15){a = PA_TABLE_868[2];}
		else if (pa > -15 && pa <= -10){a = PA_TABLE_868[3];}
		else if (pa > -10 && pa <= -6){a = PA_TABLE_868[4];}
		else if (pa > -6 && pa <= 0){a = PA_TABLE_868[5];}
		else if (pa > 0 && pa <= 5){a = PA_TABLE_868[6];}
		else if (pa > 5 && pa <= 7){a = PA_TABLE_868[7];}
		else if (pa > 7 && pa <= 10){a = PA_TABLE_868[8];}
		else if (pa > 10){a = PA_TABLE_868[9];}
		last_pa = 3;
	}
	else if (MHz >= 900 && MHz <= 928){
		if (pa <= -30){a = PA_TABLE_915[0];}
		else if (pa > -30 && pa <= -20){a = PA_TABLE_915[1];}
		else if (pa > -20 && pa <= -15){a = PA_TABLE_915[2];}
		else if (pa > -15 && pa <= -10){a = PA_TABLE_915[3];}
		else if (pa > -10 && pa <= -6){a = PA_TABLE_915[4];}
		else if (pa > -6 && pa <= 0){a = PA_TABLE_915[5];}
		else if (pa > 0 && pa <= 5){a = PA_TABLE_915[6];}
		else if (pa > 5 && pa <= 7){a = PA_TABLE_915[7];}
		else if (pa > 7 && pa <= 10){a = PA_TABLE_915[8];}
		else if (pa > 10){a = PA_TABLE_915[9];}
		last_pa = 4;
	}
	if (modulation == 2){
		PA_TABLE[0] = 0;
		PA_TABLE[1] = a;
	}else{
		PA_TABLE[0] = a;
		PA_TABLE[1] = 0;
	}
	CC1101_writeBurstReg(SpiPtr, CC1101_PATABLE, PA_TABLE, 8);
}

/******************************************************************************
*
* This function splits the Packet Automation Control 1 into its different fieldnames.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_split_PKTCTRL1(XSpiPs* SpiPtr){
	u8 calc = CC1101_readStatus(SpiPtr, CC1101_PKTCTRL1);
	pc1PQT		= calc & 0xE0;
	pc1CRC_AF	= calc & 0x08;
	pc1APP_ST	= calc & 0x04;
	pc1ADRCHK	= calc & 0x03;
}

/******************************************************************************
*
* This function splits the Packet Automation Control 0 into its different fieldnames.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_split_PKTCTRL0(XSpiPs* SpiPtr){
	u8 calc = CC1101_readStatus(SpiPtr, CC1101_PKTCTRL0);
	pc0WDATA	= calc & 0x40;
	pc0PktForm	= calc & 0x30;
	pc0CRC_EN	= calc & 0x04;
	pc0LenConf	= calc & 0x03;

}

/******************************************************************************
*
* This function splits the Modem Configuration 4 into its different fieldnames.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_split_MDMCFG4(XSpiPs* SpiPtr){
	u8 calc = CC1101_readStatus(SpiPtr, CC1101_MDMCFG4);
	m4RxBw = calc & 0xF0;
	m4DaRa = calc & 0x0F;

}

/******************************************************************************
*
* This function splits the Modem Configuration 2 into its different fieldnames.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_split_MDMCFG2(XSpiPs* SpiPtr){
	u8 calc = CC1101_readStatus(SpiPtr, CC1101_MDMCFG2);
	m2DCOFF	= calc & 0x80;
	m2MODFM = calc & 0x70;
	m2MANCH = calc & 0x08;
	m2SYNCM = calc & 0x07;
}

/******************************************************************************
*
* This function splits the Modem Configuration 1 into its different fieldnames.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_split_MDMCFG1(XSpiPs* SpiPtr){
	u8 calc = CC1101_readStatus(SpiPtr, CC1101_MDMCFG1);
	m1FEC	= calc & 0x80;
	m1PRE	= calc & 0x70;
	m1CHSP	= calc & 0x03;
}


/******************************************************************************
*
* This function configures all necessary registers before first use.
* Look in the datasheet for the configuration options.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		This function is static.
*
******************************************************************************/
static void CC1101_regConfigSettings(XSpiPs* SpiPtr){
	CC1101_writeReg(SpiPtr, CC1101_FSCTRL1,  0x06);

	CC1101_setCCMode(SpiPtr, ccmode);
	CC1101_setMHZ(SpiPtr, MHz);

	CC1101_writeReg(SpiPtr, CC1101_MDMCFG1,  0x02);
	CC1101_writeReg(SpiPtr, CC1101_MDMCFG0,  0xF8);
	CC1101_writeReg(SpiPtr, CC1101_CHANNR,   chan);
	CC1101_writeReg(SpiPtr, CC1101_DEVIATN,  0x47);
	CC1101_writeReg(SpiPtr, CC1101_FREND1,   0x56);
	CC1101_writeReg(SpiPtr, CC1101_MCSM0 ,   0x18);
	CC1101_writeReg(SpiPtr, CC1101_FOCCFG,   0x16);
	CC1101_writeReg(SpiPtr, CC1101_BSCFG,    0x1C);
	CC1101_writeReg(SpiPtr, CC1101_AGCCTRL2, 0xC7);
	CC1101_writeReg(SpiPtr, CC1101_AGCCTRL1, 0x00);
	CC1101_writeReg(SpiPtr, CC1101_AGCCTRL0, 0xB2);
	CC1101_writeReg(SpiPtr, CC1101_FSCAL3,   0xE9);
	CC1101_writeReg(SpiPtr, CC1101_FSCAL2,   0x2A);
	CC1101_writeReg(SpiPtr, CC1101_FSCAL1,   0x00);
	CC1101_writeReg(SpiPtr, CC1101_FSCAL0,   0x1F);
	CC1101_writeReg(SpiPtr, CC1101_FSTEST,   0x59);
	CC1101_writeReg(SpiPtr, CC1101_TEST2,    0x81);
	CC1101_writeReg(SpiPtr, CC1101_TEST1,    0x35);
	CC1101_writeReg(SpiPtr, CC1101_TEST0,    0x09);
	CC1101_writeReg(SpiPtr, CC1101_PKTCTRL1, 0x04);
	CC1101_writeReg(SpiPtr, CC1101_ADDR,     0x00);
	CC1101_writeReg(SpiPtr, CC1101_FIFOTHR,	 0x0F);
}


/******************************************************************************
*
* This function gets the status of the CC1101 chip.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	status.
*
* @note		None.
*
******************************************************************************/
static u8 CC1101_getState(XSpiPs* SpiPtr){
	u8 status = 0;
	status = CC1101_writeStrobe(SpiPtr, CC1101_SNOP);
	return ((status & 0b01110000) >> 4);
}


/******************************************************************************
*
* This function sets the CC1101 into sending mode.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setTX(XSpiPs* SpiPtr){
	CC1101_writeStrobe(SpiPtr, CC1101_SIDLE);
	CC1101_writeStrobe(SpiPtr, CC1101_STX);		//Start sending state
	trxstate=1;
}


/******************************************************************************
*
* This function sets the CC1101 into receiving mode.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setRX(XSpiPs* SpiPtr){
	CC1101_writeStrobe(SpiPtr, CC1101_SIDLE);
	CC1101_writeStrobe(SpiPtr, CC1101_SRX);		//Start receiving state
	trxstate=2;
}

/******************************************************************************
*
* This function sends the data to CC1101 over the SPI interface.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	txBuffer is a pointer to the buffer that's to be transmitted.
* @param	size is the length of the buffer.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_sendData(XSpiPs* SpiPtr, u8 *txBuffer, u8 size){
	CC1101_writeReg(SpiPtr, CC1101_TXFIFO, size);
	CC1101_writeBurstReg(SpiPtr, CC1101_TXFIFO, txBuffer, size);
	CC1101_writeStrobe(SpiPtr, CC1101_SIDLE);
	CC1101_writeStrobe(SpiPtr, CC1101_STX);
	while(CC1101_getState(SpiPtr) != 0){ // waits till the data is sent before the buffer is flushed
		usleep(10);
	}
	CC1101_writeStrobe(SpiPtr, CC1101_SFTX);
}


/******************************************************************************
*
* This function receives the data from the CC1101 over the SPI interface.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	RxBuffer is a pointer to the buffer to store the transmitted data.
*
* @return	size is the length of the buffer.
*
* @note		None.
*
******************************************************************************/
u8 CC1101_receiveData(XSpiPs* SpiPtr, u8 *rxBuffer){
	u8 size;

	if(CC1101_readStatus(SpiPtr, CC1101_RXBYTES) & BYTES_IN_RXFIFO){
		//xil_printf("data in fifo\r\n");
		size = CC1101_readReg(SpiPtr, CC1101_RXFIFO);
		if (size > 61) {size = 61;}
		CC1101_readBurstReg(SpiPtr, CC1101_RXFIFO, rxBuffer, size);
		CC1101_writeStrobe(SpiPtr, CC1101_SFRX);
		usleep(10);
		CC1101_writeStrobe(SpiPtr, CC1101_SRX);
		usleep(10);
		return size;
	} else {
		//xil_printf("no data in fifo\r\n");
		CC1101_writeStrobe(SpiPtr, CC1101_SFRX);
		CC1101_writeStrobe(SpiPtr, CC1101_SRX);
		return 0;
	}
}


/******************************************************************************
*
* This function sets the sync-word qualifier mode.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	v is the desired sync-word mode.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setSyncMode(XSpiPs* SpiPtr, u8 v){
	CC1101_split_MDMCFG2(SpiPtr);
	m2SYNCM = 0;
	if (v>7){v=7;}
	m2SYNCM=v;
	CC1101_writeReg(SpiPtr, CC1101_MDMCFG2, m2DCOFF+m2MODFM+m2MANCH+m2SYNCM);
}


/******************************************************************************
*
* This function en-/disables the CRC.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	v is 1 for enable and 0 for disable.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setCRC(XSpiPs* SpiPtr, u8 v){
	CC1101_split_PKTCTRL0(SpiPtr);
	pc0CRC_EN = 0;
	if (v==1){pc0CRC_EN=4;}
	CC1101_writeReg(SpiPtr, CC1101_PKTCTRL0, pc0WDATA+pc0PktForm+pc0CRC_EN+pc0LenConf);
}


/******************************************************************************
*
* This function sets the desired datarate.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	d is the datarate in kbit/s.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
void CC1101_setDRate(XSpiPs* SpiPtr, double d) {
	CC1101_split_MDMCFG4(SpiPtr);
	double c = d;

	uint32_t xosc = 26000;
	m4DaRa = log2((c * (double )(1ULL << 20)) / xosc);
	uint32_t MDMCFG3 = round(c * ((double) (1ULL << (28 - m4DaRa)) / xosc) - 256.);

	if (MDMCFG3 == 256) {
		MDMCFG3 = 0;
		m4DaRa++;
	}

	CC1101_writeReg(SpiPtr, CC1101_MDMCFG4, m4RxBw + m4DaRa);
	CC1101_writeReg(SpiPtr, CC1101_MDMCFG3, MDMCFG3);
}


/******************************************************************************
*
* This function checks if there is incoming data.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	1 when there is data incoming and 0 when no data.
*
* @note		GDO0 should be configured to 0x06.
*
******************************************************************************/
u8	 CC1101_checkReceiveFlag(XSpiPs* SpiPtr){
	u8 receiveFlag;
	receiveFlag = CC1101_readStatus(SpiPtr, CC1101_PKTSTATUS);
	return (receiveFlag & 0b00001000) >> 3;
}


/******************************************************************************
*
* This function returns the last estimated RSSI value.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
*
* @return	1 when there is data incoming and 0 when no data.
*
* @note		None.
*
******************************************************************************/
u8 CC1101_getRssi(XSpiPs* SpiPtr) {
	u8 rssi;
	rssi = CC1101_readStatus(SpiPtr, CC1101_RSSI);
	if (rssi >= 128) {
		rssi = (rssi - 256) / 2 - 74;
	} else {
		rssi = (rssi / 2) - 74;
	}
	return rssi;
}


/****************************************************************
*
* This function sets the Forward Error Correction (FEC).
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	v is 1 for enable and 0 for disable
*
* @return	None.
*
* @note		None.
*
****************************************************************/
void CC1101_setFEC(XSpiPs* SpiPtr, u8 v){
CC1101_split_MDMCFG1(SpiPtr);
m1FEC=0;
if (v==1){m1FEC=128;}
CC1101_writeReg(SpiPtr, CC1101_MDMCFG1, m1FEC+m1PRE+m1CHSP);
}


/****************************************************************
*
* This function sets the packet length configuration.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	v is the desired packet length mode.
*
* @return	None.
*
* @note		0 for fixed packet length, 1 for variable packet length, 2 for infinite packet length, 3 is reserved
*
****************************************************************/
void CC1101_setLengthConfig(XSpiPs* SpiPtr, u8 v){
CC1101_split_PKTCTRL0(SpiPtr);
pc0LenConf = 0;
if (v>3){v=3;}
pc0LenConf = v;
CC1101_writeReg(SpiPtr, CC1101_PKTCTRL0, pc0WDATA+pc0PktForm+pc0CRC_EN+pc0LenConf);
}


/****************************************************************
*
* This function sets the (fixed) packet length.
*
* @param	SpiPtr is a pointer to the SPI driver component to use.
* @param	l is the desired packet length.
*
* @return	None.
*
* @note		None.
*
****************************************************************/
void CC1101_setPacketLength(XSpiPs* SpiPtr, u8 l){
CC1101_writeReg(SpiPtr, CC1101_PKTLEN, l);
}
