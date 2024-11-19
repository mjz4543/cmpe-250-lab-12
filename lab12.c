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

/*********************************************************************/

// put functions here
int ColToInt(char c);

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

	const char *inStr = "\n\r>";
	const char *TimeOutStr = ":\tOut of time--color was \0";
	const char *WrongStr = ":\tWrong\t";
	const char *Cols[] = {"Red\0", "Green\0", "Blue\0", "White\0"};
	const int	 ColMasks[] = {PORTB_LED_RED_MASK, PORTB_LED_GREEN_MASK, 
													PORTB_LED_BLUE_MASK, PORTB_LEDS_MASK};
	
	// put globals here
	
  for (;;) { /* do forever */
		
		// TODO: kill god
		
	} /* do forever */

} /* main */

//***********************************************
// Function that maps color commands to ints
// Params: Character
// Returns: int
//***********************************************
int ColToInt(char c) {
	c = ToUpperChar(c);
	switch(c)
	{
		case 'R':
			return 0;
		case 'G':
			return 1;
		case 'B':
			return 2;
		case 'W':
			return 3;
		default:
			return -1;
	}
}
