/********************************************************************
 FileName:      HardwareProfile - Low Pin Count USB Development Kit.h
 Dependencies:  See INCLUDES section
 Processor:     PIC18 or PIC24 USB Microcontrollers
 Hardware:      Low Pin Count USB Development Kit
 Compiler:      Microchip C18
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style
  2.3   09/15/2008   Broke out each hardware platform into its own
                     "HardwareProfile - xxx.h" file
********************************************************************/

#ifndef HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H
#define HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //#define USE_SELF_POWER_SENSE_IO	
    #define tris_self_power     TRISAbits.TRISA2    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTAbits.RA2
    #else
    #define self_power          1
    #endif

    //#define USE_USB_BUS_SENSE_IO
    #define tris_usb_bus_sense  TRISAbits.TRISA1    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTAbits.RA1
    #else
    #define USB_BUS_SENSE       1
    #endif

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    // #define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER	

    /** Board definition ***********************************************/
    //These defintions will tell the main() function which board is
    //  currently selected.  This will allow the application to add
    //  the correct configuration bits as wells use the correct
    //  initialization functions for the board.  These defitions are only
    //  required in the stack provided demos.  They are not required in
    //  final application design.
    
    #define DEMO_BOARD LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
    #define LOW_PIN_COUNT_USB_DEVELOPMENT_KIT
    #define CLOCK_FREQ 48000000
    #define GetSystemClock() CLOCK_FREQ
    #define MESSAGE_LENGTH 16
    
    /** LED ************************************************************/
    #define LED1 LATCbits.LATC5
	#define LED2 LATCbits.LATC4
	#define LED3 LATCbits.LATC3
	#define LED4 LATCbits.LATC6
	#define LED5 LATCbits.LATC7
	#define LED6 LATBbits.LATB7
	#define LED7 LATBbits.LATB6

	#define LED1pin TRISCbits.TRISC5
	#define LED2pin TRISCbits.TRISC4
	#define LED3pin TRISCbits.TRISC3
	#define LED4pin TRISCbits.TRISC6
	#define LED5pin TRISCbits.TRISC7
	#define LED6pin TRISBbits.TRISB7
	#define LED7pin TRISBbits.TRISB6

	#define initLEDs() { TRISB &= 0x3F; TRISC &= 0x07; LATB &= 0x3F; LATC &= 0x07; } 

	/** RS 232 lines ****************************************************/
    extern char uart_dtr_placeholder;

    #define UART_TRISTx   TRISBbits.TRISB7
    #define UART_TRISRx   TRISBbits.TRISB5
    #define UART_Tx       PORTBbits.RB7
    #define UART_Rx       PORTBbits.RB5
    #define UART_TRISRTS  TRISBbits.TRISB4
    #define UART_RTS      PORTBbits.RB4
    #define UART_TRISDTR  TRISBbits.TRISB6
    // #define UART_DTR      PORTBbits.RB6
    #define UART_DTR      uart_dtr_placeholder
    #define UART_ENABLE RCSTAbits.SPEN

    /** I/O pin definitions ********************************************/
    #define INPUT_PIN 1
    #define OUTPUT_PIN 0

#endif  //HARDWARE_PROFILE_LOW_PIN_COUNT_USB_DEVELOPMENT_KIT_H
