/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:		PIC18, PIC24, and PIC32 USB Microcontrollers
 Hardware:		This demo is natively intended to be used on Microchip USB demo
 				boards supported by the MCHPFSUSB stack.  See release notes for
 				support matrix.  This demo can be modified for use on other hardware
 				platforms.
 Complier:  	Microchip C18 (for PIC18), C30 (for PIC24), C32 (for PIC32)
 Company:		Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release
  2.1   Updated for simplicity and to use common
                     coding style
  2.8  Improvements to USBCBSendResume(), to make it easier to use.
	Added runtime check to avoid buffer overflow possibility if
	the USB IN data rate is somehow slower than the UART RX rate.
********************************************************************/
char uart_dtr_placeholder;

/** INCLUDES *******************************************************/
#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "HardwareProfile.h"

/** CONFIGURATION **************************************************/
#if defined(LOW_PIN_COUNT_USB_DEVELOPMENT_KIT)
        //14K50
        #pragma config CPUDIV = NOCLKDIV
        #pragma config USBDIV = OFF
        #pragma config FOSC   = HS
        #pragma config PLLEN  = ON
        #pragma config FCMEN  = OFF
        #pragma config IESO   = OFF
        #pragma config PWRTEN = OFF
        #pragma config BOREN  = OFF
        #pragma config BORV   = 30
//        #pragma config VREGEN = ON
        #pragma config WDTEN  = OFF
        #pragma config WDTPS  = 32768
        #pragma config MCLRE  = OFF
        #pragma config HFOFST = OFF
        #pragma config STVREN = ON
        #pragma config LVP    = OFF
        #pragma config XINST  = OFF
        #pragma config BBSIZ  = OFF
        #pragma config CP0    = OFF
        #pragma config CP1    = OFF
        #pragma config CPB    = OFF
        #pragma config WRT0   = OFF
        #pragma config WRT1   = OFF
        #pragma config WRTB   = OFF
        #pragma config WRTC   = OFF
        #pragma config EBTR0  = OFF
        #pragma config EBTR1  = OFF
        #pragma config EBTRB  = OFF       
#else
        #error No hardware board defined, see "HardwareProfile.h" and __FILE__
#endif
/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "USB/usb_device.h"
#include "USB/usb.h"

#include "HardwareProfile.h"
#include "eeprom.h"
#include <string.h>

/** V A R I A B L E S ********************************************************/
#pragma udata
char USB_Out_Buffer[CDC_DATA_OUT_EP_SIZE];
char RS232_Out_Data[CDC_DATA_IN_EP_SIZE];

unsigned char  NextUSBOut;
unsigned char    NextUSBOut;
//char RS232_In_Data;
unsigned char    LastRS232Out;  // Number of characters in the buffer
unsigned char    RS232cp;       // current position within the buffer
unsigned char RS232_Out_Data_Rdy = 0;
USB_HANDLE  lastTransmission;

// User variables
char message[MESSAGE_LENGTH+1];
BOOL clearRS232Buffer;
enum states {P_VERSION, P_PROMPT, P_INPUT} nextProcessState;

char length, eeprom_crc, eeprom_message[MESSAGE_LENGTH+1];

//BOOL stringPrinted;


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);
void ProcessIO(void);
void USBDeviceTasks(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
void USBCBSendResume(void);
void BlinkUSBStatus(void);
void UserInit(void);
void InitializeUSART(void);
void putcUSART(char c);
unsigned char getcUSART ();

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
	//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
	//the reset, high priority interrupt, and low priority interrupt
	//vectors.  However, the current Microchip USB bootloader 
	//examples are intended to occupy addresses 0x00-0x7FF or
	//0x00-0xFFF depending on which bootloader is used.  Therefore,
	//the bootloader code remaps these vectors to new locations
	//as indicated below.  This remapping is only necessary if you
	//wish to program the hex file generated from this project with
	//the USB bootloader.  If no bootloader is used, edit the
	//usb_config.h file and comment out the following defines:
	//#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
	//#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
	#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
	#else	
		#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
		#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
		#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
	#endif
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	extern void _startup (void);        // See c018i.c in your C18 compiler dir
	#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
	void _reset (void)
	{
	    _asm goto _startup _endasm
	}
	#endif
	#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
	void Remapped_High_ISR (void)
	{
	     _asm goto YourHighPriorityISRCode _endasm
	}
	#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
	void Remapped_Low_ISR (void)
	{
	     _asm goto YourLowPriorityISRCode _endasm
	}
	
	#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	//Note: If this project is built while one of the bootloaders has
	//been defined, but then the output hex file is not programmed with
	//the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
	//As a result, if an actual interrupt was enabled and occured, the PC would jump
	//to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
	//executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
	//(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
	//would effective reset the application.
	
	//To fix this situation, we should always deliberately place a 
	//"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
	//"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
	//hex file of this project is programmed with the bootloader, these sections do not
	//get bootloaded (as they overlap the bootloader space).  If the output hex file is not
	//programmed using the bootloader, then the below goto instructions do get programmed,
	//and the hex file still works like normal.  The below section is only required to fix this
	//scenario.
	#pragma code HIGH_INTERRUPT_VECTOR = 0x08
	void High_ISR (void)
	{
	     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#pragma code LOW_INTERRUPT_VECTOR = 0x18
	void Low_ISR (void)
	{
	     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
	}
	#endif	//end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

	#pragma code
	
	
	//These are your actual interrupt handling routines.
	#pragma interrupt YourHighPriorityISRCode
	void YourHighPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	    #if defined(USB_INTERRUPT)
	        USBDeviceTasks();
        #endif
	
	}	//This return will be a "retfie fast", since this is in a #pragma interrupt section 
	#pragma interruptlow YourLowPriorityISRCode
	void YourLowPriorityISRCode()
	{
		//Check which interrupt flag caused the interrupt.
		//Service the interrupt
		//Clear the interrupt flag
		//Etc.
	
	}	//This return will be a "retfie", since this is in a #pragma interruptlow section 

/** DECLARATIONS ***************************************************/
#pragma code

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
void main(void)
{   
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
            USBDeviceTasks();
        #endif
       	// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
        LED1 = UART_DTR;
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
 static void InitializeSystem(void)
 {
     #if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
         ADCON1 |= 0x0F;                 // Default all pins to digital
     #elif defined(__C30__)
         AD1PCFGL = 0xFFFF;
     #elif defined(__C32__)
         AD1PCFG = 0xFFFF;
     #endif

     #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM)
 	//On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
 	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
 	//This allows the device to power up at a lower initial operating frequency, which can be
 	//advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
 	//operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
 	//power up the PLL.
     {
         unsigned int pll_startup_counter = 600;
         OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
         while(pll_startup_counter--);
     }
     //Device switches over automatically to PLL output after PLL is locked and ready.
     #endif

     #if defined(PIC18F87J50_PIM)
 	//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
 	//use the ANCONx registers to control this, which is different from other devices which
 	//use the ADCON1 register for this purpose.
     WDTCONbits.ADSHR = 1;			// Select alternate SFR location to access ANCONx registers
     ANCON0 = 0xFF;                  // Default all pins to digital
     ANCON1 = 0xFF;                  // Default all pins to digital
     WDTCONbits.ADSHR = 0;			// Select normal SFR locations
     #endif

     #if defined(PIC18F46J50_PIM)
 	//Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
 	//use the ANCONx registers to control this, which is different from other devices which
 	//use the ADCON1 register for this purpose.
     ANCON0 = 0xFF;                  // Default all pins to digital
     ANCON1 = 0xFF;                  // Default all pins to digital
     #endif

    #if defined(PIC24FJ64GB004_PIM)
 	//On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
 	//by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
 	//This allows the device to power up at a lower initial operating frequency, which can be
 	//advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
 	//operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
 	//power up the PLL.
     {
         unsigned int pll_startup_counter = 600;
         CLKDIVbits.PLLEN = 1;
         while(pll_startup_counter--);
     }

     //Device switches over automatically to PLL output after PLL is locked and ready.
     #endif


 //	The USB specifications require that USB peripheral devices must never source
 //	current onto the Vbus pin.  Additionally, USB peripherals should not source
 //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
 //	When designing a self powered (as opposed to bus powered) USB peripheral
 //	device, the firmware should make sure not to turn on the USB module and D+
 //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
 //	firmware needs some means to detect when Vbus is being powered by the host.
 //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
 // 	can be used to detect when Vbus is high (host actively powering), or low
 //	(host is shut down or otherwise not supplying power).  The USB firmware
 // 	can then periodically poll this I/O pin to know when it is okay to turn on
 //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
 //	peripheral device, it is not possible to source current on D+ or D- when the
 //	host is not actively providing power on Vbus. Therefore, implementing this
 //	bus sense feature is optional.  This firmware can be made to use this bus
 //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
 //	HardwareProfile.h file.    
     #if defined(USE_USB_BUS_SENSE_IO)
     tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
     #endif

 //	If the host PC sends a GetStatus (device) request, the firmware must respond
 //	and let the host know if the USB peripheral device is currently bus powered
 //	or self powered.  See chapter 9 in the official USB specifications for details
 //	regarding this request.  If the peripheral device is capable of being both
 //	self and bus powered, it should not return a hard coded value for this request.
 //	Instead, firmware should check if it is currently self or bus powered, and
 //	respond accordingly.  If the hardware has been configured like demonstrated
 //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
 //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
 //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
 //	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
 //	to it in HardwareProfile.h.
     #if defined(USE_SELF_POWER_SENSE_IO)
     tris_self_power = INPUT_PIN;	// See HardwareProfile.h
     #endif

     UserInit();

     USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
     					//variables to known states.
 }//end InitializeSystem

/******************************************************************************
 * Function:        void ReadMessageFromEEPROM(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function will save a store message from the EEPROM 
 *                  to memory. If no valid message is on EEPROM, save a 
 *                  default one.
 *
 * Note:            
 *
 *****************************************************************************/
void ReadMessageFromEEPROM()
{    
    eeprom_crc = readEEPROM(0x00);
    length = readEEPROM(0x01);

     // Se o CRC do comprimento da mensagem for incorrecto, n‹o se faz load da mensagem
    if (crc(0,length) != eeprom_crc)
        strcpypgm2ram(message, MESSAGE_DEFAULT);
    else
    {
        LED2 = 1;
        eeprom_crc = readEEPROM(0x02);
        readStrEEPROM(0x03, length, eeprom_message);
        eeprom_message[length] = '\0';
        if (strcrc(eeprom_message) != eeprom_crc)
            strcpypgm2ram(message, MESSAGE_DEFAULT);
        else
        {
            strcpy(message, eeprom_message);
            LED3 = 1;
        }
    }    
}

/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
	unsigned char i;
    
    initLEDs();

    // 	 Initialize the arrays
	for (i=0; i<sizeof(USB_Out_Buffer); i++)
    {
		USB_Out_Buffer[i] = 0;
    }
    
    ReadMessageFromEEPROM();

	NextUSBOut = 0;
	LastRS232Out = 0;
	lastTransmission = 0;
	nextProcessState = P_VERSION;
	RS232cp = 0;

}//end UserInit

/******************************************************************************
 * Function:        void mySetLineCodingHandler(void)
 *
 * PreCondition:    USB_CDC_SET_LINE_CODING_HANDLER is defined
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function gets called when a SetLineCoding command
 *                  is sent on the bus.  This function will evaluate the request
 *                  and determine if the application should update the baudrate
 *                  or not.
 *
 * Note:            
 *
 *****************************************************************************/
#if defined(USB_CDC_SET_LINE_CODING_HANDLER)
void mySetLineCodingHandler(void)
{
    //If the request is not in a valid range
    if(cdc_notice.GetLineCoding.dwDTERate.Val > 115200)
    {
        //NOTE: There are two ways that an unsupported baud rate could be
        //handled.  The first is just to ignore the request and don't change
        //the values.  That is what is currently implemented in this function.
        //The second possible method is to stall the STATUS stage of the request.
        //STALLing the STATUS stage will cause an exception to be thrown in the 
        //requesting application.  Some programs, like HyperTerminal, handle the
        //exception properly and give a pop-up box indicating that the request
        //settings are not valid.  Any application that does not handle the
        //exception correctly will likely crash when this requiest fails.  For
        //the sake of example the code required to STALL the status stage of the
        //request is provided below.  It has been left out so that this demo
        //does not cause applications without the required exception handling
        //to crash.
        //---------------------------------------
        //USBStallEndpoint(0,1);
    }
    else
    {
        DWORD_VAL dwBaud;

        //Update the baudrate info in the CDC driver
        CDCSetBaudRate(cdc_notice.GetLineCoding.dwDTERate.Val);
        
        //Update the baudrate of the UART
        #if defined(__18CXX)
            dwBaud.Val = (DWORD)(GetSystemClock()/4)/line_coding.dwDTERate.Val-1;
            SPBRG = dwBaud.v[0];
            SPBRGH = dwBaud.v[1];
        #elif defined(__C30__)
            #if defined(__dsPIC33EP512MU810__) || defined (__PIC24EP512GU810__)
            dwBaud.Val = ((GetPeripheralClock()/(unsigned long)(16 * line_coding.dwDTERate.Val)))- 1;
            #else
            dwBaud.Val = (((GetPeripheralClock()/2)+(BRG_DIV2/2*line_coding.dwDTERate.Val))/BRG_DIV2/line_coding.dwDTERate.Val-1);
            #endif
            U2BRG = dwBaud.Val;
        #elif defined(__C32__)
            U2BRG = ((GetPeripheralClock()+(BRG_DIV2/2*line_coding.dwDTERate.Val))/BRG_DIV2/line_coding.dwDTERate.Val-1);
            //U2MODE = 0;
            U2MODEbits.BRGH = BRGH2;
            //U2STA = 0;
        #endif
    }
}
#endif

/********************************************************************
 * Function:        void ProcessMenu(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function parses the input string given
 *					by the user.
 *
 * Note:            None
 *******************************************************************/
void ProcessMenu(void)
{
	int i; // Debug
	
	// If the line is empty return.
	if(RS232cp == 0) return;
	
	// Strip the CR/NL character.
	RS232_Out_Data[RS232cp] = '\0';
	
	// Test different menu options
	if(strcmpram2pgm("help", RS232_Out_Data) == 0)
		putrsUSBUSART("\r\nAvailable commands:\r\n\r\n* help - Displays this message, recursive style!\r\n* setMessage <text> - Defines a new message to be displayed on the POV.\r\n* showMessage - Shows the current message\r\n* version - Displays firmware version.");
	else if(strcmpram2pgm("debug0", RS232_Out_Data) == 0)
	{
        sprintf(RS232_Out_Data, "\r\nCRC: 0x%X, Length: 0x%X\r\n", eeprom_crc, length);
        putsUSBUSART(RS232_Out_Data);
	}
	else if(strcmpram2pgm("debug1", RS232_Out_Data) == 0)
	{
        sprintf(RS232_Out_Data, "\r\nMessage:'%s'\r\n", eeprom_message);
        putsUSBUSART(RS232_Out_Data);
	}
	else if(strncmpram2pgm("setMessage ", RS232_Out_Data, 10) == 0)
	{				
		// Copy message
        strncpy(message, &RS232_Out_Data[11], MESSAGE_LENGTH);
        message[MESSAGE_LENGTH] = '\0';
        writeEEPROM(0x00, crc(0x00, (char) strlen(message)));
        writeEEPROM(0x01, (char) strlen(message));
        writeStrEEPROM(0x03, strlen(message), message);
        writeEEPROM(0x02, strcrc(message));

        // Display confirmation
        strcpypgm2ram(RS232_Out_Data, "\r\nYour message is now defined as:\r\n\t");
        strcat(RS232_Out_Data, message);

        putsUSBUSART(RS232_Out_Data);
	}
	else if(strcmpram2pgm("showMessage", RS232_Out_Data) == 0)
	{
		sprintf(RS232_Out_Data, "\r\nYour message is defined as:\r\n\t%s", message);
		putsUSBUSART(RS232_Out_Data);
	}
	else if(strcmpram2pgm("version", RS232_Out_Data) == 0)
		putrsUSBUSART("\r\nPOV serial configuration. Version 0.1.");
	else
		putrsUSBUSART("\r\nUnknown command. Type 'help' for the list of available commands.");
	
	clearRS232Buffer = 1;
}

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{   
    char backSpace[3] = {0x8,' ',0x8};
	
    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

	// If DTR is low no client is connected, back to default.
	if (UART_DTR == 0) { nextProcessState = P_VERSION; RS232cp = 0; }

	// Print welcome message
	if(USBUSARTIsTxTrfReady() && (UART_DTR == 1))
	{
	    // Print Version
	    if(nextProcessState == P_VERSION)
    	{
    		putrsUSBUSART("\r\n POV serial configuration. Version 0.1\r\n\r\n\ttype 'help' for the list of available commands.\r\n");
    		nextProcessState = P_PROMPT;
    	}
    	// Print Prompt
    	else if(nextProcessState == P_PROMPT)
    	{
    		putrsUSBUSART("\r\n>> ");
    		nextProcessState = P_INPUT;
    	}
    	else if (nextProcessState == P_INPUT)
    	{
    	    // In case the buffer is full, reset the input buffer and display error message.
    	    if (RS232cp == CDC_DATA_IN_EP_SIZE)
    	    {
                RS232cp = 0;
                nextProcessState = P_PROMPT;
    	    }
    	    // Process IO
    	    else
    	    {
    	        LastRS232Out = getsUSBUSART(&(RS232_Out_Data[RS232cp]), 1);
    	        if(LastRS232Out > 0)
    	            switch(RS232_Out_Data[RS232cp])
    				{
    				    // Enter/Return, process message
    					case 0x0A:	
    					case 0x0D:	ProcessMenu(); RS232cp = 0; nextProcessState = P_PROMPT; break;
    					// Backspace
    					case 0x7F:  
    					case 0x08:	if(RS232cp > 0) {putUSBUSART(backSpace,3); RS232cp--;} break;
    					// Replay character to user
    					default: 	putUSBUSART(&(RS232_Out_Data[RS232cp]), 1); RS232cp++;
    				}
    	    }
    	}
	}
		
    CDCTxService();
	// Limpa o buffer caso necess‡rio :)
	if(clearRS232Buffer == 1)
	{
		memset(RS232_Out_Data, '\0', sizeof(RS232_Out_Data));
		clearRS232Buffer = 0;
	}
}		//end ProcessIO

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
#if 0
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus
#endif

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:
	
	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
	//things to not work as intended.	
	

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *					
 *					This call back is invoked when a wakeup from USB suspend 
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the USBCBSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be 
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.
	
	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and 
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific 
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckCDCRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This 
 *					callback function should initialize the endpoints 
 *					for the device's usage according to the current 
 *					configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    CDCInitEP();
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *					
 *					The USBCBSendResume() "callback" function is used
 *					to send this special USB signalling which wakes 
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *					
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET 
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.   
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior, 
 *                  as a USB device that has not been armed to perform remote 
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *                  
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are 
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex: 
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup. 
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in 
 *                  USBCBWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager 
    //properties page for the USB device, power management tab, the 
    //"Allow this device to bring the computer out of standby." checkbox 
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) 
    {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE)
        {
            USBMaskInterrupts();
            
            //Clock switch to settings consistent with normal USB operation.
            USBCBWakeFromSuspend();
            USBSuspendControl = 0; 
            USBBusIsSuspended = FALSE;  //So we don't execute this code again, 
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at 
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;        
            do
            {
                delay_count--;
            }while(delay_count);
            
            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do
            {
                delay_count--;
            }while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}


/*******************************************************************
 * Function:        void USBCBEP0DataReceived(void)
 *
 * PreCondition:    ENABLE_EP0_DATA_RECEIVED_CALLBACK must be
 *                  defined already (in usb_config.h)
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called whenever a EP0 data
 *                  packet is received.  This gives the user (and
 *                  thus the various class examples a way to get
 *                  data that is received via the control endpoint.
 *                  This function needs to be used in conjunction
 *                  with the USBCBCheckOtherReq() function since 
 *                  the USBCBCheckOtherReq() function is the apps
 *                  method for getting the initial control transfer
 *                  before the data arrives.
 *
 * Note:            None
 *******************************************************************/
#if defined(ENABLE_EP0_DATA_RECEIVED_CALLBACK)
void USBCBEP0DataReceived(void)
{
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was 
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was 
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT 
            //      endpoints).
            break;
        default:
            break;
    }      
    return TRUE; 
}

/** EOF main.c *************************************************/
