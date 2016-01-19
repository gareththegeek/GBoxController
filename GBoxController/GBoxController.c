// please see http://frank.circleofcurrent.com/index.php?page=hid_tutorial_1
// the "usb_hid_rpt_desc.hid" file needs to be viewed with the HID Descriptor Tool from USB.org

// required headers
#define F_CPU 16000000L
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>

// What the Pins do
// 1   PC6: RESET
// 2   PD0: Rotary Encoder 2 L
// 3   PD1: Rotary Encoder 2 R
// 4   PD2: USB D+
// 5   PD3: USB D-
// 6   PD4: Rotary Encoder 3 L
// 7   VCC: 5v
// 8   GND: 0v
// 9   PB6: CLK
// 10  PB7: CLK
// 11  PD5: Rotary Encoder 3 R
// 12  PD6: Rotary Encoder 4 L
// 13  PD7: Rotary Encoder 4 R
// 14  PB0: REB0 - Rotary Encoder 0 Button
// 15  PB1: REB1 - Rotary Encoder 1 Button
// 16  PB2: REB2 - Rotary Encoder 2 Button
// 17  PB3: REB3 - Rotary Encoder 3 Button
// 18  PB4: REB4 - Rotary Encoder 4 Button
// 19  PB5: Rotary Encoder 0 L
// 20  VCC: 5v
// 21 AREF: 5v 
// 22  GND: 0v
// 23  PC0: Rotary Encoder 1 R
// 24  PC1: Rotary Encoder 1 L
// 25  PC2: Rotary Encoder 0 R
// 26  PC3: Clock - Shift Register
// 27  PC4: Latch - Shift Register
// 28  PC5: Data In - Shift Register

// V-USB
#include "usbdrv/usbdrv.h"

#define CD4021BE_LATCH_PIN PINC4
#define CD4021BE_CLOCK_PIN PINC3
#define CD4021BE_DATA_PIN PINC5

#define REB0 PINB0
#define REB1 PINB1
#define REB2 PINB2
#define REB3 PINB3
#define REB4 PINB4

#define USAGE(x) 0x9, x, // Usage
#define USAGE2(x, y) 0xA, x, y, // Usage
#define USAGE_PAGE(x) 0x5, x, // Usage Page
#define USAGE_PAGE2(x, y) 0x6, x, y, // Usage Page
#define USAGE_MINIMUM(x) 0x19, x, // Usage Minimum
#define USAGE_MAXIMUM(x) 0x29, x, // Usage Maximum
#define COLLECTION(x) 0xA1, x, // Collection
#define END_COLLECTION 0xC0, // End Collection
#define REPORT_ID(x) 0x85, x, // Report
#define REPORT_SIZE(x) 0x75, x, // Report Size
#define REPORT_COUNT(x) 0x95, x, // Report Count
#define LOGICAL_MINIMUM(x) 0x15, x, // Logical Minimum
#define LOGICAL_MINIMUM2(x, y) 0x16, x, y, // Logical Minimum
#define LOGICAL_MINIMUM4(x, y, z, w) 0x17, x, y, z, w, // Logical Minimum
#define LOGICAL_MAXIMUM(x) 0x25, x, // Logical Maximum
#define LOGICAL_MAXIMUM2(x, y) 0x26, x, y, // Logical Maximum
#define LOGICAL_MAXIMUM4(x, y, z, w) 0x27, x, y, z, w, // Logical Maximum
#define PHYSICAL_MINIMUM(x) 0x35, x,
#define PHYSICAL_MAXIMUM(x) 0x45, x,
#define PHYSICAL_MAXIMUM2(x, y) 0x46, x, y,
#define PHYSICAL_MAXIMUM4(x, y, z, w) 0x47, x, y, z, w,
#define OUTPUT(x) 0x91, x, // Output
#define INPUT(x) 0x81, x, // Input
#define FEATURE(x) 0xB1, x, // Feature
#define UNIT(x, y) 0x65, x,
#define UNIT2(x, y) 0x66, x, y,
#define UNIT_EXPONENT(x) 0x55, ((x) & 0x0f),

#define GENERIC_DESKTOP 0x1
#define GAME_PAD 0x5
#define APPLICATION 0x1
#define X 0x30
#define Y 0x31
#define Z 0x32
#define Rx 0x33
#define Ry 0x34
#define Rz 0x35
#define SLIDER 0x36
#define DIAL 0x37
#define DATA_VAR_ABS 0x2
#define CNST_VAR_ABS 0x3
#define BUTTON 0x9
#define PHYSICAL 0x0

PROGMEM const char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
	USAGE_PAGE(GENERIC_DESKTOP)
	USAGE(GAME_PAD)
	COLLECTION(APPLICATION)
		COLLECTION(PHYSICAL)
			USAGE_PAGE(BUTTON)
			USAGE_MINIMUM(1)
			USAGE_MAXIMUM(31)
			LOGICAL_MINIMUM(0)
			LOGICAL_MAXIMUM(1)
			REPORT_COUNT(31)
			REPORT_SIZE(1)
			INPUT(DATA_VAR_ABS)
			REPORT_COUNT(1)
			REPORT_SIZE(1)
			INPUT(CNST_VAR_ABS)
		END_COLLECTION
	END_COLLECTION
};

typedef struct
{
	uint8_t buttons0;
	uint8_t buttons1;
	uint8_t buttons2;
	uint8_t buttons3;
	
} gamepad_report_t;

static uint8_t idle_rate = 500 / 4; // see HID1_11.pdf sect 7.2.4
static uint8_t protocol_version = 0; // see HID1_11.pdf sect 7.2.6

static volatile gamepad_report_t gamepad_report;
static volatile gamepad_report_t gamepad_report_old;

usbMsgLen_t usbFunctionSetup(uint8_t data[8])
{
	// see HID1_11.pdf sect 7.2 and http://vusb.wikidot.com/driver-api
	usbRequest_t *rq = (void *)data;

	if ((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_CLASS)
		return 0; // ignore request if it's not a class specific request

	// see HID1_11.pdf sect 7.2
	switch (rq->bRequest)
	{
		case USBRQ_HID_GET_IDLE:
			usbMsgPtr = &idle_rate; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_IDLE:
			idle_rate = rq->wValue.bytes[1]; // read in idle rate
			return 0; // send nothing
		case USBRQ_HID_GET_PROTOCOL:
			usbMsgPtr = &protocol_version; // send data starting from this byte
			return 1; // send 1 byte
		case USBRQ_HID_SET_PROTOCOL:
			protocol_version = rq->wValue.bytes[1];
			return 0; // send nothing
		case USBRQ_HID_GET_REPORT:
			usbMsgPtr = (uchar*)&gamepad_report;
			return sizeof(gamepad_report);
		case USBRQ_HID_SET_REPORT: // no "output" or "feature" implemented, so ignore
			return 0; // send nothing
		default: // do not understand data, ignore
			return 0; // send nothing
	}
}

// this function is used to guarantee that the data is sent to the computer once
void usbSendHidReport(uchar * data, uchar len)
{
	while(1)
	{
		usbPoll();
		if (usbInterruptIsReady())
		{
			usbSetInterrupt(data, len);
			break;
		}
	}
}

static void hardwareInit(void)
{
	// TODO: Not sure but I think this may be required if I implement the watchdog
	
	//uchar i = 0;
	//uchar j = 0;

	//PORTB	= 0xff;   // Activate all pull-ups
	//DDRB	= 0x00;   // All pins input
	//PORTC	= 0xff;   // Activate all pull-ups
	//DDRC	= 0x00;   // All pins input
	//PORTD	= 0xfa;   // 1111 1010 bin: activate pull-ups except on USB lines
	//DDRD	= 0x05;   // 0000 0101 bin: all pins input except USB (-> USB reset)
	//while(--j)
	//{
		//// USB Reset by device only required on Watchdog Reset
		//while(--i); // Delay >10ms for USB reset
	//}
	//DDRD = 0x00;    // 0000 0000 bin: remove USB reset condition
	
	// Configure timer 0 for a rate of 16M/(1024 * 256) = 61.04 Hz (~16ms)
	//TCCR0B = 5;      // Timer0 prescaler: 1024
	
	//DDRD |= (1<<DDD1);
	//PORTD |= (1<<PORTD1);
	
	// Digital Pins //
	
	PORTB = 0xff;	// Activate pull-ups
	DDRB = 0x00;	// All pins are input
	PORTD = 0xf3;	// Activate pull-ups (except USB pins)
	DDRD = 0x00;	// All pins are input
	
	PORTC = 0xc7;	// Activate pull-ups (except shift register)
	
	DDRC =
		(1 << CD4021BE_LATCH_PIN) |
		(1 << CD4021BE_CLOCK_PIN);	// Enable write on latch and clock pins only
		
	// Enable interrupts for rotary encoders
	PCMSK0 |= (1 << PCINT5);
	PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
	PCMSK2 |= (1 << PCINT16) | (1 << PCINT17) | (1 << PCINT20) | (1 << PCINT21) | (1 << PCINT22) | (1 << PCINT23);
	
	PCICR |= (1 << PCIE0);
	PCICR |= (1 << PCIE1);
	PCICR |= (1 << PCIE2);
}

#define CHANGE_BUFFER_LENGTH 128
static volatile gamepad_report_t changeBuffer[CHANGE_BUFFER_LENGTH];
static volatile uint8_t changeBufferIndex;

static volatile gamepad_report_t currentState;
static volatile gamepad_report_t previousState;

#define ROTARY_ENCODER_COUNT 10
static int8_t encval[ROTARY_ENCODER_COUNT];
static uint8_t old_AB[ROTARY_ENCODER_COUNT];

static const int8_t enc_states [] PROGMEM =
	{0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table

static void readEncoder(uint8_t reindex, uint8_t data, uint8_t leftValue, uint8_t rightValue)
{	
	old_AB[reindex] <<=2;  //remember previous state
	old_AB[reindex] |= data;
	encval[reindex] += pgm_read_byte(&(enc_states[( old_AB[reindex] & 0x0f )]));
	
	if (encval[reindex] > 3)
	{
		encval[reindex] = 0;
		
		if(leftValue >= 8)
		{
			leftValue -= 8;
			previousState.buttons3 &= ~(1 << leftValue);
			currentState.buttons3 |= (1 << leftValue);
		}
		else
		{
			previousState.buttons2 &= ~(1 << leftValue);
			currentState.buttons2 |= (1 << leftValue);
		}
	}
	else if (encval[reindex] < -3)
	{
		encval[reindex] = 0;
		
		if(rightValue >= 8)
		{
			rightValue -= 8;
			previousState.buttons3 &= ~(1 << rightValue);
			currentState.buttons3 |= (1 << rightValue);
		}
		else
		{
			previousState.buttons2 &= ~(1 << rightValue);
			currentState.buttons2 |= (1 << rightValue);
		}
	}
}

#define ROTARY_ENCODER0 readEncoder(0, ( (PINB & (1 << PINB5)) >> 4 ) | ( (PINC & (1 << PINC2)) >> 2 ), 5, 6)
#define ROTARY_ENCODER1 readEncoder(1, ( (PINC & ( (1 << PINC0) | (1 << PINC1) ))      ), 7, 8)
#define ROTARY_ENCODER2 readEncoder(2, ( (PIND & ( (1 << PIND0) | (1 << PIND1) ))      ), 9, 10)
#define ROTARY_ENCODER3 readEncoder(3, ( (PIND & ( (1 << PIND4) | (1 << PIND5) )) >> 4 ), 11, 12)
#define ROTARY_ENCODER4 readEncoder(4, ( (PIND & ( (1 << PIND6) | (1 << PIND7) )) >> 6 ), 13, 14)

static uint8_t previousPINB;
static uint8_t previousPINC;
static uint8_t previousPIND;

ISR(PCINT0_vect)
{
	// Figure out which pin triggered the interrupt
	
	uint8_t pinB = PINB & 0x20; /*00100000 PB5*/
	
	uint8_t changeB = previousPINB ^ pinB;
	
	previousPINB = pinB;
	
	if (changeB != 0)
	{
		// Rotary Encoder 0 on pins PB5 and PC3
		ROTARY_ENCODER0;
	}
}

ISR(PCINT1_vect)
{
	// Figure out which pin triggered the interrupt
	
	uint8_t pinC = PINC & 0x07; /*00000111 PB0-PB2*/
	
	uint8_t changeC = previousPINC ^ pinC;
	
	previousPINC = pinC;
	
	if (changeC & (1 << PINC2))
	{
		// Rotary Encoder 0 on pins PB5 and PC2
		ROTARY_ENCODER0;
	}
	
	if(changeC & ( (1 << PINC0) | (1 << PINC1) ))
	{
		// Rotary Encoder 1 on pins PC0 and PC1
		ROTARY_ENCODER1;
	}
}

ISR(PCINT2_vect)
{
	// Figure out which pin triggered the interrupt
	
	uint8_t pinD = PIND & 0xf3; /*11110011 PD0-PD1 PD4-PD7*/
	
	uint8_t changeD = previousPIND ^ pinD;
	
	previousPIND = pinD;
	
	if(changeD != 0)
	{
		if (changeD & ((1 << PIND0) | (1 << PIND1)))
		{
			// Rotary Encoder 2 on pins PD0 and PD1
			ROTARY_ENCODER2;
		}
		if (changeD & ((1 << PIND4) | (1 << PIND5)))
		{
			// Rotary Encoder 3 on pins PD4 and PD5
			ROTARY_ENCODER3;
		}
		if (changeD & ((1 << PIND6) | (1 << PIND7)))
		{
			// Rotary Encoder 4 on pins PD6 and PD7
			ROTARY_ENCODER4;
		}
	}
}

static void clockPulse(uint8_t clockPin)
{
	PINC |= (1 << clockPin);
	_delay_us(0.2d);
	PINC &= ~(1 << clockPin);
	_delay_us(0.2d);
}

static void latch(uint8_t latchPin)
{
	//Pulse the latch pins:
	//set them to 1 to collect parallel data
	PINC |= (1 << latchPin);
	//set it to 1 to collect parallel data, wait
	_delay_us(20.0d);
	//set it to 0 to transmit data serially
	PINC &= ~(1 << latchPin);
	//_delay_us(10.0d);
}

static uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin)
{	
	int8_t i;
	uint8_t temp;
	uint8_t data = 0;
	
	clockPulse(clockPin);
	
	for (i = 7; i >= 0; i--)
	{
		temp = PINC;
		_delay_us(0.2d);
		
		temp &= (1 << dataPin);
		temp >>= dataPin;
		
		data |= (temp << i);
		
		// HACK: Contrary to all tutorials and the 4021 chip's data-,
		// I seem to have to use two clock pulses instead of one.
		// Clearly I have misunderstood something here but this code
		// works so I'm going with it!
		clockPulse(clockPin);
		clockPulse(clockPin);
	}
	
	return data;
}

static void updateReport()
{	
	latch(CD4021BE_LATCH_PIN);
	
	currentState.buttons0 = shiftIn(CD4021BE_DATA_PIN, CD4021BE_CLOCK_PIN);
	currentState.buttons1 = shiftIn(CD4021BE_DATA_PIN, CD4021BE_CLOCK_PIN);
	currentState.buttons2 = ~PINB & 0x1f /*00011111 = PB0-PB4*/ | (currentState.buttons2 & 0xe0); /*11100000 = REL0 RER0 REL1*/
	// currentState.buttons3 = RER1 REL2 RER2 REL3 RER3 REL4 RER4
	
	uint8_t change0 = currentState.buttons0 ^ previousState.buttons0;
	uint8_t change1 = currentState.buttons1 ^ previousState.buttons1;
	uint8_t change2 = (currentState.buttons2 & 0x1f) | ( (currentState.buttons2 ^ previousState.buttons2) & 0xe0 );
	uint8_t change3 = currentState.buttons3 ^ previousState.buttons3;
	
	memcpy(&previousState, &currentState, sizeof(gamepad_report_t));
	
	for (uint8_t i = 0; i < CHANGE_BUFFER_LENGTH; i++)
	{
		changeBuffer[i].buttons0 |= change0;
		changeBuffer[i].buttons1 |= change1;
		changeBuffer[i].buttons2 |= change2;
		changeBuffer[i].buttons3 |= change3;
	}
	
	memcpy(&gamepad_report, &changeBuffer[changeBufferIndex], sizeof(gamepad_report_t));
	
	changeBuffer[changeBufferIndex].buttons0 = 0;
	changeBuffer[changeBufferIndex].buttons1 = 0;
	changeBuffer[changeBufferIndex].buttons2 = 0;
	changeBuffer[changeBufferIndex].buttons3 = 0;
	
	changeBufferIndex++;
	changeBufferIndex %= CHANGE_BUFFER_LENGTH;
}

int main()
{
	// TODO: watchdog
	wdt_disable(); // no watchdog, just because I'm lazy
	
	//position = 0;
	
	TCCR1B = _BV(CS12) | _BV(CS11); // timer is initialized, used to keep track of idle period
	
	hardwareInit();
	
	usbInit(); // start v-usb
    usbDeviceDisconnect(); // enforce USB re-enumeration, do this while interrupts are disabled!
	_delay_ms(250);
    usbDeviceConnect();
	
    sei(); // enable interrupts
	
	uint8_t to_send = 1; // boolean, true for first time
	
	while (1)
	{
		usbPoll();
		
		/*
		 * this area is where you should set the movement
		 * and button values of the reports using the input
		 * method of your choice
		 *
		*/
		
		updateReport();
		
		// determine whether or not the report should be sent
		if ((TCNT1 > ((4 * (F_CPU / 1024000)) * idle_rate) || TCNT1 > 0x7FFF) && idle_rate != 0)
		{// using idle rate
			to_send = 1;
		}
		else
		{// or if data has changed
			if (memcmp(&gamepad_report, &gamepad_report_old, sizeof(gamepad_report_t)) != 0)
			{
				to_send = 1;
			}
		}
		
		usbPoll();
		if (to_send != 0)
		{
			// send the data if needed
			usbSendHidReport((uchar*)&gamepad_report, sizeof(gamepad_report_t));
			TCNT1 = 0; // reset timer
		}
		
		usbPoll();
		
		memcpy(&gamepad_report_old, &gamepad_report, sizeof(gamepad_report_t));
		
		to_send = 0; // reset flag
	}
	
	return 0;
}