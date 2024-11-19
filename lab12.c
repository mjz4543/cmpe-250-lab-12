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
