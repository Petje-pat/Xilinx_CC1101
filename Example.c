/* ---------------------------------------------------------------------------- *
 * Description :
 * Example of using the CC1101
 *
 *
 * Note :
 * The user of this demo code should NOT expect perfect code.
 * The demo code DOES have (probably) room for further improvement/refactoring.
 * Furthermore, the authors of this demo have done their best to test the code.
 * However they may have overlooked conditions in which the demo code may not
 * operate properly. If you notice an error or bug or improvement opportunity,
 * please be so kind to constructively report this to the authors of the demo code.
 *
 * Authors:
 * 		Patrick Huisman
 *
 * Date: March 2025
 * ---------------------------------------------------------------------------- */


/*****************************************************************************/
/***************************** Include Files *********************************/
/*****************************************************************************/
#include <stdio.h>
#include "platform.h"
#include "xscugic.h"
#include "xtmrctr.h"
#include "xil_printf.h"
#include "sleep.h"
#include "xtime_l.h"
#include "xgpio.h"
#include "xparameters.h"
#include "xscutimer.h"
//#include "xiicps.h"
#include "xadcps.h"
#include "xspips.h"
#ifdef SDT
#include "xinterrupt_wrap.h"
#endif
#include "xplatform_info.h"
#include "CC1101.h"



/*****************************************************************************/
/************************** Constant Definitions *****************************/
/*****************************************************************************/
// SCU timer settings
#define SCU_TIMER_VALUE		(0x1EFE920) // 100ms @ 650MHz/2 (3ns)
// Interrupt settings
// in steps of 8, lower number high priority
#define SCU_TIMER_INTR_PRI			(0xA0)
#define SCU_TIMER_INTR_TRIG			(0x03)


/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */
#ifndef SDT
#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SPI_INTR_ID		XPAR_XSPIPS_0_INTR
#endif

#define SPI_SELECT_1	0x00
#define SPI_SELECT_0	0x00
//#define XADC_DEVICE_ID XPAR_XADCPS_0_DEVICE_ID
//#define SWT_DEVICE_ID XPAR_GPIO_5_DEVICE_ID
//#define BTNS_DEVICE_ID XPAR_GPIO_2_DEVICE_ID


/*****************************************************************************/
/************************** Driver instances and pointers ********************/
/*****************************************************************************/
//static XScuTimer my_Timer;  // SCU Timer instance
//static XScuGic my_Gic ;  	// GIC instance
static XScuGic IntcInstance;
//static XAdcPs XAdcInst; 	// XADC driver instance
static XSpiPs SpiInstance;
//static XGpio switches, buttons;


/*****************************************************************************/
/************************** Prototype of Interrupt Handler (aka ISR) *********/
/*****************************************************************************/
//void my_timer_interrupt_handler(void * CallBackRef);
void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount);

/*****************************************************************************/
/************************** Prototype of API Functions ***********************/
/*****************************************************************************/
//int Init_GIC(void);
//int Init_SCUTimer(void);
//int addScuTimerToInterruptSystem(void);
//int AdcConfig();
//float ReadAdc(int channelToRead);

//u32 XGpio_DiscreteRead(XGpio *InstancePtr, unsigned Channel);

static int SpiPsSetupIntrSystem(XScuGic *IntcInstancePtr,
	      XSpiPs *SpiInstancePtr, u16 SpiIntrId);
static void SpiPsDisableIntrSystem(XScuGic *IntcInstancePtr, u16 SpiIntrId);


/*****************************************************************************/
/************************** Global variables *********************************/
/*****************************************************************************/

int Test;


/*****************************************************************************/
/************************** Main program *************************************/
/*****************************************************************************/
int main(){
	//int Status;
	//u8 *BufferPtr;
	//u8 UniqueValue;
	//u32 Count;
	u32 MaxSize = MAX_DATA;
	u32 ChipSelect = SPI_SELECT_1;
	XSpiPs_Config *SpiConfig;
	u32 Platform;
	u16 SpiIntrId = XPAR_XSPIPS_1_INTR;

	Platform = XGetPlatform_Info();
	if ((Platform == XPLAT_ZYNQ_ULTRA_MP) || (Platform == XPLAT_VERSAL)) {
		MaxSize = 1024 * 10;
		ChipSelect = SPI_SELECT_0;	/* Device is on CS 0 */
#ifndef SDT
		SpiIntrId = XPAR_XSPIPS_0_INTR;
#endif
	}

	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
#ifndef SDT
	SpiConfig = XSpiPs_LookupConfig(XPAR_XSPIPS_0_DEVICE_ID);
#else
	SpiConfig = XSpiPs_LookupConfig(BaseAddress);
#endif
	if (NULL == SpiConfig) {
		xil_printf("SPI  configuration lookup Failed \r\n");
	}

	if( XSpiPs_CfgInitialize(&SpiInstance, SpiConfig,
			SpiConfig->BaseAddress) == XST_SUCCESS){
		xil_printf("SPI config initialized \r\n");
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	if( XSpiPs_SelfTest(&SpiInstance) == XST_SUCCESS){
		xil_printf("SPI selftest succeeded \r\n");
	}else{
		xil_printf("SPI selftest failed \r\n");
	}

	/*
	 * Connect the Spi device to the interrupt subsystem such that
	 * interrupts can occur. This function is application specific
	 */
#ifndef SDT
	if( SpiPsSetupIntrSystem(&IntcInstance, &SpiInstance,
			SpiIntrId) == XST_SUCCESS) {
		xil_printf("SPI connected to interrupt \r\n");
	}
#else
	if( XSetupInterruptSystem(&SpiInstance, &XSpiPs_InterruptHandler,
			SpiConfig->IntrId,
			SpiConfig->IntrParent,
			XINTERRUPT_DEFAULT_PRIORITY) == XST_SUCCESS) {
		xil_printf("SPI connected to interrupt \r\n");
	}
#endif

	/*
	 * Setup the handler for the SPI that will be called from the
	 * interrupt context when an SPI status occurs, specify a pointer to
	 * the SPI driver instance as the callback reference so the handler is
	 * able to access the instance data
	 */
	XSpiPs_SetStatusHandler(&SpiInstance, &SpiInstance, (XSpiPs_StatusHandler) &SpiPsHandler);
	xil_printf("SPI set status handler \r\n");
	/*
	 * Set the SPI device as a master with auto start and manual
	 * chip select mode options
	 */
	XSpiPs_SetOptions(&SpiInstance, XSPIPS_MASTER_OPTION| XSPIPS_FORCE_SSELECT_OPTION);

	XSpiPs_SetClkPrescaler(&SpiInstance, XSPIPS_CLK_PRESCALE_256);
	xil_printf("SPI set options \r\n");
	/*
	 * Assert the EEPROM chip select
	 */
	XSpiPs_SetSlaveSelect(&SpiInstance, ChipSelect);
	xil_printf("SPI set slave select \r\n");

	char packetLength = 0x32;

	CC1101_writeReg(&SpiInstance, CC1101_PKTLEN, packetLength);

	sleep(1);

	if (CC1101_readReg(&SpiInstance, CC1101_PKTLEN) == packetLength){
		xil_printf("Correctly wrote and read packet length register \r\n");
	} else {
		xil_printf("Failed to correctly write and read packet length register \r\n");
	}

#ifndef SDT
	SpiPsDisableIntrSystem(&IntcInstance, SpiIntrId);
#else
	XDisconnectInterruptCntrl(SpiConfig->IntrId, SpiConfig->IntrParent);
#endif

	/*
	xil_printf("Entering main loop \r\n");
	while (1)
	{
		// Do nothing
	}*/
	xil_printf("Disabled SPI interrupt\r\n\n");
	return XST_SUCCESS;
}








/*****************************************************************************/
/************************** SPI Interrupt Handler (aka ISR) ************/
/*****************************************************************************/

/* staat in de CC1101.c file*/



/*****************************************************************************
*
* This function is the handler which performs processing for the SPI driver.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized.  It is called when a transfer of SPI data
* completes or an error occurs.
*
* This handler provides an example of how to handle SPI interrupts
* but is application specific.
*
*
* @param	CallBackRef is a reference passed to the handler.
* @param	StatusEvent is the status of the SPI .
* @param	ByteCount is the number of bytes transferred.
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event
	 */
	TransferInProgress = FALSE;

	/*
	 * If the event was not transfer done, then track it as an error
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {
		Error++;
	}
}


/*****************************************************************************/
/************************** Definitions of (API) Functions **********************/
/*****************************************************************************/

#ifndef SDT
/*****************************************************************************
*
* This function setups the interrupt system for an Spi device.
*
* @param	IntcInstancePtr is a pointer to the instance of the Intc device.
* @param	SpiInstancePtr is a pointer to the instance of the Spi device.
* @param	SpiIntrId is the interrupt Id for an SPI device.
*
* @return
*		- XST_SUCCESS if successful
*		- XST_FAILURE if not successful
*
* @note		None.
*
******************************************************************************/
static int SpiPsSetupIntrSystem(XScuGic *IntcInstancePtr,
				XSpiPs *SpiInstancePtr, u16 SpiIntrId)
{
	int Status;

	XScuGic_Config *IntcConfig; /* Instance of the interrupt controller */

	Xil_ExceptionInit();

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(XPAR_SCUGIC_SINGLE_DEVICE_ID);
	if (NULL == IntcConfig) {
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
				       IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				     (Xil_ExceptionHandler)XScuGic_InterruptHandler,
				     IntcInstancePtr);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, SpiIntrId,
				 (Xil_ExceptionHandler)XSpiPs_InterruptHandler,
				 (void *)SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	/*
	 * Enable the interrupt for the Spi device.
	 */
	XScuGic_Enable(IntcInstancePtr, SpiIntrId);

	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

/*****************************************************************************
*
* This function disables the interrupts that occur for the Spi device.
*
* @param	IntcInstancePtr is the pointer to a ScuGic driver instance.
* @param	SpiIntrId is the interrupt Id for an SPI device.
*
* @return	None.
*
* @note		None.
*
******************************************************************************/
static void SpiPsDisableIntrSystem(XScuGic *IntcInstancePtr, u16 SpiIntrId)
{
	/*
	 * Disable the interrupt for the SPI device.
	 */
	XScuGic_Disable(IntcInstancePtr, SpiIntrId);

	/*
	 * Disconnect and disable the interrupt for the Spi device.
	 */
	XScuGic_Disconnect(IntcInstancePtr, SpiIntrId);
}
#endif

