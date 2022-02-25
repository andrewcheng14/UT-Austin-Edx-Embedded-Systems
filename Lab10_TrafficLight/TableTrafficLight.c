// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016
// 
// Implemented by Andrew Cheng 
// December 31, 2021
//
// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"        // (for testing)
#include "tm4c123gh6pm.h" // (I/O register macros)

// ***** 2. Global Declarations Section *****
// inputs
#define SENSORS (*((volatile unsigned long*) 0x4002401C)) // PE2-PE0

// outputs
#define TRAFFIC_LIGHTS (*((volatile unsigned long*) 0x400050FC)) // PB5-PB0
#define WALK_LIGHTS (*((volatile unsigned long*) 0x40025028)) // PF3 and PF1

// states
#define GO_WEST 0
#define WAIT_WEST 1
#define GO_SOUTH 2
#define WAIT_SOUTH 3
#define GO_WALK 4
#define HURRY_UP1 5
#define HURRY_UP2 6
#define HURRY_UP3 7
#define HURRY_UP4 8
#define HURRY_UP5 9
#define HURRY_UP6 10
#define HURRY_UP7 11
#define DONT_WALK 12

// Traffic Light outputs
#define WEST_GREEN 0x0C
#define WEST_YELLOW 0x14
#define SOUTH_GREEN 0x21
#define SOUTH_YELLOW 0x22
#define BOTH_RED 0x24

// Crosswalk outputs
#define WALK_RED 0x2
#define WALK_GREEN 0x8
#define WALK_OFF 0x0

typedef struct state_st {
  unsigned long Traffic_Lights_Output; // 6-bit pattern to traffic lights output
	unsigned long Walk_Lights_Output;    // 2-bit pattern to traffic light input
  unsigned long Time;                  // delay in units of 10ms
  unsigned long Next[8];               // next state for inputs 0-7
} State;

State FSM[13] = {
	{WEST_GREEN, WALK_RED, 100, {GO_WEST, GO_WEST, WAIT_WEST, WAIT_WEST, WAIT_WEST, WAIT_WEST, WAIT_WEST, WAIT_WEST}}, // state 0
	{WEST_YELLOW, WALK_RED, 50, {GO_SOUTH, GO_SOUTH, GO_SOUTH, GO_SOUTH, GO_WALK, GO_WALK, GO_SOUTH, GO_SOUTH}},    // state 1
	{SOUTH_GREEN, WALK_RED, 100, {GO_SOUTH,	WAIT_SOUTH, GO_SOUTH, WAIT_SOUTH, WAIT_SOUTH,	WAIT_SOUTH,	WAIT_SOUTH,	WAIT_SOUTH}}, // state 2
	{SOUTH_YELLOW, WALK_RED, 50, {GO_WEST, GO_WEST, GO_WEST, GO_WEST, GO_WALK, GO_WALK, GO_WALK, GO_WALK}}, // state 3
	{BOTH_RED, WALK_GREEN, 100, {GO_WALK, HURRY_UP1, HURRY_UP1, HURRY_UP1, GO_WALK, HURRY_UP1, HURRY_UP1,	HURRY_UP1}}, // state 4
	{BOTH_RED, WALK_OFF, 50, {HURRY_UP2, HURRY_UP2, HURRY_UP2, HURRY_UP2, HURRY_UP2, HURRY_UP2, HURRY_UP2, HURRY_UP2}}, // state 5
	{BOTH_RED, WALK_RED, 50, {HURRY_UP3, HURRY_UP3, HURRY_UP3, HURRY_UP3, HURRY_UP3, HURRY_UP3, HURRY_UP3, HURRY_UP3}}, // state 6
	{BOTH_RED, WALK_OFF, 500, {HURRY_UP4, HURRY_UP4, HURRY_UP4, HURRY_UP4, HURRY_UP4, HURRY_UP4, HURRY_UP4, HURRY_UP4}}, // state 7
	{BOTH_RED, WALK_RED, 50, {HURRY_UP5, HURRY_UP5, HURRY_UP5, HURRY_UP5, HURRY_UP5, HURRY_UP5, HURRY_UP5, HURRY_UP5}}, // state 8
	{BOTH_RED, WALK_OFF, 50, {HURRY_UP6, HURRY_UP6, HURRY_UP6, HURRY_UP6, HURRY_UP6, HURRY_UP6, HURRY_UP6, HURRY_UP6}}, // state 9
	{BOTH_RED, WALK_RED, 50, {HURRY_UP7, HURRY_UP7, HURRY_UP7, HURRY_UP7, HURRY_UP7, HURRY_UP7, HURRY_UP7, HURRY_UP7}}, // state 10
	{BOTH_RED, WALK_OFF, 50, {DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK, DONT_WALK}}, // state 11
	{BOTH_RED, WALK_RED, 50, {GO_WEST, GO_WEST, GO_SOUTH, GO_WEST, GO_WEST, GO_WEST, GO_SOUTH, GO_WEST}} // state 12
};


// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

// Initializes Ports
void PortE_Init(void);
void PortF_Init(void);
void PortB_Init(void);

// Functions for initializing systick timer and creating time delays
void SysTick_Init(void);
// Delay parameter is in units of the 80 Mhz core clock (12.5 ns)
void SysTick_Wait(unsigned long delay);
void SysTick_Wait10ms(unsigned long delay);

// ***** 3. Subroutines Section *****
int main(void){ 
	unsigned long s;
	unsigned long input;
	
  	TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	
	// Initialize ports and SysTick Timer
	PortE_Init();
	PortF_Init();
	PortB_Init();
	SysTick_Init();
  
 	EnableInterrupts();
	
	// set initial state
	s = GO_WEST;
	while(1){
		// 1. Perform output
		TRAFFIC_LIGHTS = FSM[s].Traffic_Lights_Output;
		WALK_LIGHTS = FSM[s].Walk_Lights_Output;

		// 2. Wait
		SysTick_Wait10ms(FSM[s].Time);

		// 3. Input
		input = SENSORS;

		// 4. Go to next state (depends on current state and input)
		s = FSM[s].Next[input];
	}
}

void PortE_Init() { volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x10;         // activate clock for Port E
	delay = SYSCTL_RCGC2_R;					// Allow time for clock to start
	GPIO_PORTE_LOCK_R = 0x4C4F434B; // Unlock GPIO for Port E
	GPIO_PORTE_CR_R |= 0x7;         // Allow changes to PE2-PE0
	GPIO_PORTE_AMSEL_R &= ~0xA;			// Disable Analog function for PE2-PE0
	GPIO_PORTE_PCTL_R &= ~0xA;			// Regular digital function for PE2-PE0
	GPIO_PORTE_DIR_R &= ~0xA;				// Set PE2-PE0 as inputs
	GPIO_PORTE_AFSEL_R &= ~0xA;			// Disable alternate function on PE2-PE0
	GPIO_PORTE_DEN_R = 0x7;					// Enable digital I/O on PE2-PE0
}

void PortF_Init() { volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x20;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;
	GPIO_PORTF_CR_R |= 0xA;
	GPIO_PORTF_AMSEL_R &= ~0xA;
	GPIO_PORTF_PCTL_R &= ~0xA;
	GPIO_PORTF_DIR_R |= 0xA;
	GPIO_PORTF_AFSEL_R &= ~0xA;
	GPIO_PORTF_DEN_R |= 0xA;
}

void PortB_Init() { volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x2;
	delay = SYSCTL_RCGC2_R;
	GPIO_PORTB_LOCK_R = 0x4C4F434B;
	GPIO_PORTB_CR_R |= 0x3F;
	GPIO_PORTB_AMSEL_R &= ~0x3F;
	GPIO_PORTB_PCTL_R &= ~0x3F;
	GPIO_PORTB_DIR_R |= 0x3F;
	GPIO_PORTB_AFSEL_R &= ~0x3F;
	GPIO_PORTB_DEN_R |= 0x3F;
}

void SysTick_Init(void) {
	NVIC_ST_CTRL_R = 0; // disable SysTick during initialization
	NVIC_ST_RELOAD_R = 0xFFFFFF; // set RELOAD register to max value
	NVIC_ST_CURRENT_R = 0x1; // write to CURRENT register to clear counter
	NVIC_ST_CTRL_R = 0x00000005; // enable SysTick with core clock and interrupts disabled
}

void SysTick_Wait(unsigned long delay) {
  NVIC_ST_RELOAD_R = delay - 1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0x1;     // any value written to CURRENT clears the register
  while((NVIC_ST_CTRL_R & 0x00010000) == 0){ // wait for count flag
		// with 80 Mhz core clock, it takes 12.5ns for COUNT register value to decrease by 1
  }
}

void SysTick_Wait10ms(unsigned long delay) {
	// 10ms/12.5ns = 800000 counts
	int i;
	for (i = 0; i < delay; i++) {
		SysTick_Wait(800000); // wait 10ms
	}
}
