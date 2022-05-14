/*
             LUFA Library
     Copyright (C) Dean Camera, 2017.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2017  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the RNDISEthernet demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "RNDISNE2000.h"
#include "8390.h"
#include <avr/cpufunc.h>

/** Message buffer for RNDIS messages processed by the RNDIS device class driver. */
static uint8_t RNDIS_Message_Buffer[4096];

static	unsigned char Frame[1518];

/** LUFA RNDIS Class driver interface configuration and state information. This structure is
 *  passed to all RNDIS Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_RNDIS_Device_t Ethernet_RNDIS_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 16,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 16,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
				.AdapterVendorDescription       = "LUFA/Teensy/NE2k",
				.AdapterMACAddress              = {ADAPTER_MAC_ADDRESS},
				.MessageBuffer                  = RNDIS_Message_Buffer,
				.MessageBufferLength            = sizeof(RNDIS_Message_Buffer),
			},
	};


void NE2KReadMem(uint16_t src, uint8_t *dst, uint16_t len);

#define	CLK()	_NOP();
#define	BLE1()	PORTE |=0x40
#define	BLE0()	PORTE &=(~0x40)

#define	IOW1()	PORTB |=0x40
#define	IOW0()	PORTB &=(~0x40)
#define	IOR1()	PORTB |=0x10
#define	IOR0()	PORTB &=(~0x10)
//#define	IOCHRDY()	{CLK(); CLK(); while ((PINE & 0x02) == 0){};CLK();CLK();}
#define	IOCHRDY()	{CLK(); uint8_t i = 0; while (((PINE & 0x02) == 0) && i < 25){i++;}; }
//#define	IOCHRDY()	CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();CLK();

#define	SBHE1()	// not used on my clone ne2000
#define	SBHE0()

#define	LED_TX1()		PORTF &= (~0x80)
#define	LED_TX0()		PORTF |= (0x80)
#define	LED_RX1()		PORTF&=(~0x40);
#define	LED_RX0()		PORTF |= 0x40;

#define	LED_RXERR1()	PORTF &= (~0x20);
#define	LED_RXERR0()	PORTF |= (0x20);
#define	LED_TXERR1()	PORTF &= (~0x10);
#define	LED_TXERR0()	PORTF |= (0x10);

#define	LED_BUSY1()		PORTF &= (~0x01);
#define	LED_BUSY0()		PORTF |= (0x01);

// inputs
#define	IOCS16()	(PINE & 0x80)

static inline void	outb(uint16_t	ioaddr, uint8_t	v)
{
	cli();
	SBHE1();
	PORTB = ((ioaddr >> 8) & 0x0f) | (PORTB & 0xf0);
	PORTD = (ioaddr & 0xff);
	BLE1();
	BLE0();
	IOW0();
	DDRC = 0xff;
	PORTC = v;
	IOCHRDY();
	IOW1();
//	PORTC = 0;
	DDRC = 0;
	sei();
}

// not used
/*void	outw(uint16_t	ioaddr, uint16_t	v)
{
	SBHE0();
	PORTB = ((ioaddr >> 8) & 0x0f) | (PORTB & 0xf0);
	PORTD = (ioaddr & 0xff);
	DDRC = 0xff;
	DDRA = 0xff;
	PORTC = v & 0xff;
	PORTA = v >> 8;
	CLK();
//	CLK();
	BLE1();
	BLE0();
	IOW0();
	IOCHRDY();
	IOW1();
	CLK();
//	CLK();
	PORTA = 0;
	PORTC = 0;
	DDRC = 0;
	DDRA = 0;
//	PORTC = 0x00;
//	PORTA = 0x00;
}*/


static inline void	outsw(uint16_t	ioaddr, uint8_t	*src, uint16_t len)
{
	cli();
	SBHE0();
	PORTB = ((ioaddr >> 8) & 0x0f) | (PORTB & 0xf0);
	PORTD = (ioaddr & 0xff);
	DDRC = 0xff;
	DDRA = 0xff;

	for (uint16_t i = 0; i < len; i++)
	{
		BLE1();
		BLE0();
		IOW0();
		PORTC = *(src++);
		PORTA = *(src++);
		IOCHRDY();
		IOW1();
	}
	PORTA = 0;
	PORTC = 0;
	DDRC = 0;
	DDRA = 0;
	sei();
}

static inline uint8_t	inb(uint16_t	ioaddr)
{
	cli();
	SBHE1();
	uint16_t	ret;
	DDRC = 0;
//	DDRA = 0;
	PORTC = 0;
//	PORTA = 0;
	PORTB = ((ioaddr >> 8) & 0x0f) | (PORTB & 0xf0);
	PORTD = (ioaddr & 0xff);
	BLE1();
	BLE0();
	IOR0();
	IOCHRDY();
	CLK();
	ret = PINC;
	IOR1();
	sei();
	return ret;
}

static inline void	insw(uint16_t	ioaddr, uint8_t *dst, uint16_t	len)
{
	cli();
	SBHE0();
	DDRC = 0;	// set inputs
	DDRA = 0;
	PORTC = 0;
	PORTA = 0;
	PORTB = ((ioaddr >> 8) & 0x0f) | (PORTB & 0xf0);
	PORTD = (ioaddr & 0xff);

	for (uint16_t	i = 0; i < len; i++)
	{
		BLE1();
		BLE0();
		IOR0();
		IOCHRDY();
		CLK();
		*(dst++) = PINC;
		*(dst++) = PINA;
		IOR1();
	}
	sei();
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	DDRA = 0;	// high bits of data bus
	DDRB = 0xff;	// B[0-3] == ADDR8-ADDR11, B4 IOR, B5 == reset, B6 = IOW, B7 = CLK
	DDRC = 0;	// data bus
	DDRD = 0xff;	// A0-A7
	DDRE = 0x41;	// E0 = aen (out), E1 = io ch rdy(in), E6 =  ALE (out), E7 IOCS16 (in)
	DDRF=0xff;	// debug out
	PORTA = 0;
	PORTB = 0xe0; //IOR, IOW high, reset high
	PORTC = 0;	
	PORTD = 0;
	PORTE = 2; // AEN low, ALE low, clock low, IORD
	PORTF = 0xff;


	TCCR0A = (1<<WGM01) | (1<<COM0A0); // Toggle OC0A pin on match
	OCR0A =  0; // output compare register, compare timer to 1
	TCCR0B = 1<<CS00; // prescaler = CLKIO

	LED_TX1();
	_delay_ms(200);
	LED_RX1();
	_delay_ms(200);
	LED_TXERR1();
	_delay_ms(200);
	LED_RXERR1();
	_delay_ms(200);

	PORTB = 0xc0; // lower reset

	_delay_ms(1000);

#define	IOBASE	0x300
	outb(IOBASE + E8390_CMD, E8390_NODMA+E8390_PAGE0);
	outb(IOBASE + 0x1F, inb(IOBASE + 0x1F));
	while ((inb(IOBASE + EN0_ISR) & ENISR_RESET) == 0);
	outb(IOBASE + EN0_ISR, 0xFF); 


	uint8_t prom[32];
	outb(IOBASE + E8390_CMD, (1 << 5) | 1);	// page 0, no DMA, stop
	outb(IOBASE + EN0_DCFG, 0x48 | ENDCFG_WTS);		// set word-wide access
	outb(IOBASE + EN0_RCNTLO, 0);		// clear the count regs
	outb(IOBASE + EN0_RCNTHI, 0);
	outb(IOBASE + EN0_IMR, ENISR_ALL);
	outb(IOBASE + EN0_ISR, 0xFF);
	outb(IOBASE + EN0_RXCR, E8390_RXOFF);		// set to monitor
	outb(IOBASE + EN0_TXCR, E8390_TXOFF);		// and loopback mode.
	outb(IOBASE + EN0_TPSR, 0);		// start the read
	NE2KReadMem(0, prom, 32);
	uint8_t i;

	for (i = 0; i < 6; i++)
		Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[i] = prom[i<<1];


	outb(IOBASE + E8390_CMD, E8390_NODMA|E8390_PAGE1 | E8390_STOP);

	for (i = 0; i < 6; i++) outb(IOBASE + EN1_PHYS_SHIFT(i), Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[i]);

	for (i = 0; i < 6; i++) Ethernet_RNDIS_Interface.Config.AdapterMACAddress.Octets[i] = inb(IOBASE + EN1_PHYS_SHIFT(i));

	// Initialize multicast address hashing registers to not accept multicasts
	for (i = 0; i < 8; i++) outb(IOBASE + EN1_MULT_SHIFT(i), 0);

#define	TX_START_PAGE	0x40
#define	RX_START_PAGE	TX_START_PAGE + 12
#define	STOP_PAGE	0x80
	outb(IOBASE + EN1_CURPAG, RX_START_PAGE);
	outb(IOBASE + E8390_CMD, E8390_NODMA | E8390_STOP);

	                  
/* Follow National Semi's recommendations for initing the DP83902. */
	outb(IOBASE + EN0_STARTPG, RX_START_PAGE);
	outb(IOBASE + EN0_BOUNDARY, STOP_PAGE-1);
	outb(IOBASE + EN0_STOPPG, STOP_PAGE);
	outb(IOBASE + EN0_TPSR, TX_START_PAGE);

//	outb(IOBASE + EN0_RXCR, E8390_RXCONFIG);
	outb(IOBASE + EN0_RXCR, 0x1c);
	outb(IOBASE + EN0_TXCR, E8390_TXCONFIG); /* xmit on. */
	outb(IOBASE + EN0_ISR, ENISR_ALL);
	outb(IOBASE + EN0_IMR, ENISR_ALL);
	outb(IOBASE + E8390_CMD, E8390_NODMA+E8390_START);

	LED_TX0();
	LED_RX0();
	LED_TXERR0();
	LED_RXERR0();
	USB_Init();

}

#define	ETH_ZLEN	60
uint8_t	NE2KTransmitBusy(void)
{
	outb(IOBASE + E8390_CMD, E8390_NODMA|E8390_PAGE0|E8390_START);
	return (inb(IOBASE + E8390_CMD) & (E8390_TRANS));
}

// 1 GND, 2 - power, 3 +v, 4 - scr, 5 num lock, 6 caps
uint8_t	bOunce = 0;
void	NE2KTransmit(uint8_t *packet, uint16_t len)
{
	LED_TX1();
	if (len < ETH_ZLEN)
		len = ETH_ZLEN;
	if (len > 1518)
		len = 1518;
	uint16_t	len1 = len;
	if (len1 & 0x1)
		len1++;
	outb(IOBASE + E8390_CMD, E8390_NODMA|E8390_PAGE0|E8390_START);

	uint8_t a = inb(IOBASE + EN0_ISR);
	if (a & ENISR_TX_ERR)
	{
		LED_TXERR1();
	}
	else
	{
		LED_TXERR0();
	}
	
	outb(IOBASE + EN0_ISR, ENISR_RDC | ENISR_TX);
	outb(IOBASE + EN0_RCNTLO, len1  & 0xff);
	outb(IOBASE + EN0_RCNTHI, len1 >> 8);
	outb(IOBASE + EN0_RSARLO, 0);		// start DMA at 0
	outb(IOBASE + EN0_RSARHI, TX_START_PAGE + bOunce);		// start DMA high
	outb(IOBASE + E8390_CMD,E8390_PAGE0|E8390_RWRITE|E8390_START);

	CLK();
	outsw(IOBASE + 0x10, packet, len1>>1);

//	while ((inb(IOBASE + EN0_ISR) & (ENISR_RDC)) == 0);

//	outb(IOBASE + EN0_ISR, ENISR_RDC);
// trigger
	outb(IOBASE + E8390_CMD, E8390_NODMA|E8390_PAGE0|E8390_START);
	outb(IOBASE + EN0_TCNTLO, len & 0xff);
	outb(IOBASE + EN0_TCNTHI, len >> 8);

	outb(IOBASE + EN0_TPSR, TX_START_PAGE + bOunce);
	outb(IOBASE + E8390_CMD, E8390_NODMA|E8390_TRANS|E8390_START);
	if (bOunce)
		bOunce = 0;
	else
		bOunce = 6;
	LED_TX0();
}


void NE2KReadMem(uint16_t src, uint8_t *dst, uint16_t len)
{
	// Word align length
	if (len & 1) len++;

	// Abort any remote DMA already in progress
	outb(IOBASE + E8390_CMD, E8390_PAGE0 | E8390_NODMA | E8390_START);
	outb(IOBASE + EN0_ISR, ENISR_RDC);

	// Setup DMA byte count
	outb(IOBASE + EN0_RCNTLO, len & 0xff);
	outb(IOBASE + EN0_RCNTHI, (len >> 8));

	// Setup NIC memory source address
	outb(IOBASE + EN0_RSARLO, src & 0xff);
	outb(IOBASE + EN0_RSARHI, src >> 8);

	// Select remote DMA read
	outb(IOBASE + E8390_CMD, E8390_RREAD | E8390_START);

	CLK();
//	while ((inb(IOBASE + EN0_ISR) & ENISR_RDC) == 0);
	
	// Read NIC memory
	insw(IOBASE + 0x10, dst, len >> 1);
	outb(IOBASE + EN0_ISR, ENISR_RDC);
}

struct recv_ring_desc {
	unsigned char rsr;                   // Receiver status
	unsigned char next_pkt;              // Pointer to next packet
	unsigned short count;                // Bytes in packet (length + 4)
};

uint8_t	NE2KReceive(uint8_t *packet, uint16_t *len)
{
	outb(IOBASE + E8390_CMD, E8390_PAGE0 | E8390_NODMA | E8390_START);
	uint8_t	a = inb(IOBASE + EN0_RSR);
	if (!(a & (ENRSR_RXOK|ENRSR_CRC|ENRSR_FAE|ENRSR_FO|ENRSR_MPA)))
		return 0;

	if (a & (ENRSR_CRC|ENRSR_FAE|ENRSR_FO|ENRSR_MPA))
		LED_RXERR1()
	else
		LED_RXERR0()

	uint8_t	packetPage = inb(IOBASE + EN0_BOUNDARY) +1;

	outb(IOBASE + E8390_CMD, E8390_PAGE1 | E8390_NODMA | E8390_START);

	uint8_t	rxingPage = inb(IOBASE + EN1_CURPAG);

	if (packetPage >= STOP_PAGE)
		packetPage = RX_START_PAGE;

	if (packetPage == rxingPage)
		return 0;

	uint8_t	ret = 0;

	LED_RX1()
	struct recv_ring_desc	packet_hdr;
	uint16_t	ptr = ((uint16_t)packetPage) << 8;
	NE2KReadMem(ptr, (uint8_t *)&packet_hdr, sizeof(struct recv_ring_desc));

	if ((packet_hdr.rsr & ENRSR_RXOK)) // && (packet_hdr.next_pkt != rxingPage))
	{

		int16_t	sLen =packet_hdr.count - sizeof(struct recv_ring_desc);
		if (sLen <= *len && sLen > 0)
		{
			NE2KReadMem(ptr + sizeof(struct recv_ring_desc), packet, sLen);
			*len = sLen;
			ret = 1;
		}
	}

	// Set page 0 registers
	outb(IOBASE + E8390_CMD, E8390_PAGE0 | E8390_NODMA | E8390_START);

	outb(IOBASE + EN0_BOUNDARY, packet_hdr.next_pkt - 1);
	outb(IOBASE + EN0_ISR, ENISR_RX|ENISR_RX_ERR);
	LED_RX0();
	return ret;
} 

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
//	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
//	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= RNDIS_Device_ConfigureEndpoints(&Ethernet_RNDIS_Interface);

//	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	RNDIS_Device_ProcessControlRequest(&Ethernet_RNDIS_Interface);
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	SetupHardware();

	GlobalInterruptEnable();

	uint16_t 	sLen;
	for (;;)
	{
		sLen = sizeof(Frame);
		if (RNDIS_Device_IsPacketReceived(&Ethernet_RNDIS_Interface) /*&& !NE2KTransmitBusy()*/ )	// 2us
		{
			RNDIS_Device_ReadPacket(&Ethernet_RNDIS_Interface, Frame, sizeof(Frame), &sLen);
			NE2KTransmit(Frame, sLen);
		}
		else
		{
			sLen = sizeof(Frame);
			LED_BUSY1();
			if (NE2KReceive(Frame, &sLen))
			{
				RNDIS_Device_SendPacket(&Ethernet_RNDIS_Interface, Frame, sLen);
			}
			else
			{
				LED_BUSY0();
			}
		}

		RNDIS_Device_USBTask(&Ethernet_RNDIS_Interface);	// 4us
		USB_USBTask();	// 2us
	}
}
