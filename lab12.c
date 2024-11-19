/*********************************************************************/
/* <Your program description here>                                   */
/* Name:  <Your name here>                                           */
/* Date:  <Date completed>                                           */
/* Class:  CMPE 250                                                  */
/* Section:  <Your section here>                                     */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 11, 2024                                      */
/*********************************************************************/
#include "MKL05Z4.h"
#include "lab12.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79)

/* LEDs */
#define POS_RED (8)
#define POS_GREEN (9)
#define POS_BLUE (10)
#define PORTB_LED_RED_MASK (1 << POS_RED)
#define PORTB_LED_GREEN_MASK (1 << POS_GREEN)
#define PORTB_LED_BLUE_MASK (1 << POS_BLUE)
#define PORTB_LEDS_MASK (PORTB_LED_RED_MASK | PORTB_LED_GREEN_MASK | PORTB_LED_BLUE_MASK)
/* Port Pin GPIO for LED */
#define PORT_PCR_MUX_SELECT_1_MASK (1 << PORT_PCR_MUX_SHIFT)
#define PORT_PCR_SET_GPIO (PORT_PCR_ISF_MASK | PORT_PCR_MUX_SELECT_1_MASK)



/*********************************************************************/

int RandomNumber;

//***********************************************
// Main function.
// Params: void
// Returns: void
//***********************************************
int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  /* Perform all device initialization here */
  /* Before unmasking interrupts            */
  InitPIT();
	InitUART0();
  __asm("CPSIE   I");  /* unmask interrupts */

	// put globals here
	
  for (;;) { /* do forever */
		
		// TODO: kill god
		
	} /* do forever */

} /* main */

char RandomLEDColor(void){
	RandomNumber -= (RandomNumber << 6);
	RandomNumber ^= (RandomNumber >> 17);
	RandomNumber -= (RandomNumber << 9);
	RandomNumber ^= (RandomNumber << 4);
	RandomNumber -= (RandomNumber << 3);
	RandomNumber ^= (RandomNumber << 10);
	RandomNumber ^= (RandomNumber >> 15);
	return RandomNumber |= 4;
}

void InitLEDGPIO(void){
	/* Enable clock for PORT B module */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	/* Select PORT B Pin 8 for GPIO to red LED */
	PORTB->PCR[POS_RED] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 9 for GPIO to green LED */
	PORTB->PCR[POS_GREEN] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 10 for GPIO to blue LED */
	PORTB->PCR[POS_BLUE] = PORT_PCR_SET_GPIO;
	
}
