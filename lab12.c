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

// put globals here

int RandomNumber;
UInt8 Score = 0;

//Declare Functions
int RandomLEDColor(void);
int ColToInt(char c);
void AddScore(int TimeElapsed, int RoundNumber);
void InitLEDs(void);


//***********************************************
// Main function.
// Params: void
// Returns: void
//***********************************************
int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  /* Perform all device initialization here before unmasking interrupts */
	InitLEDs();
	InitUART0();
	InitPIT();
  __asm("CPSIE   I");  /* unmask interrupts */

	char WrongStr[10] = ":\tWrong\t\0";
	char InStr[4] = "\n\r>\0";
	char TimeOutStr[28] = ":\tOut of time--color was \0";
	char Cols[4][6] = {"Red\0", "Green\0", "Blue\0", "White\0"};
	char PlayStr[] = "\n\rPlay LED Game Guessing Game (Press Any Key): \0";
	char GStr[] = "\n\rGuess (R,G,B,W)\0";
	char ScStr[] = "\n\rFinal Score: ";
	char RStr[] = "\tCorrect--color was \0";
	int	 ColMasks[] = {PORTB_LED_RED_MASK, PORTB_LED_GREEN_MASK, 
													PORTB_LED_BLUE_MASK, PORTB_LEDS_MASK};
	const int Rounds = 10;
	const int RoundTime = 11; // (seconds)
	// put globals here
	
  for (;;) { /* do forever */
	
		//game loop start
		Score = 0;
		FPTB->PSOR = ColMasks[3];
		
		PutStringSB(PlayStr, MAX_STRING);
		while(!IsKeyPressed()) {}
			
		PutStringSB(GStr, MAX_STRING);
		for(int Round = 1; Round <= Rounds; Round++)
		{
			//print input stuff
			PutStringSB(InStr, MAX_STRING);
			
			//Turn Off LEDs and get ramdom number
			FPTB->PSOR = ColMasks[3];
			RandomNumber = RandomLEDColor();
			
			//Turn On Random LED and start timer
			FPTB->PCOR = ColMasks[RandomNumber];
			SetCount(0);
			StartTimer();
			
			while(GetCount() < ((RoundTime - Round) * 100))
			{
				char keypressed = IsKeyPressed();
				if(!keypressed){ continue; } // if no key is pressed, loop again
				
				int guess = ColToInt(Dequeue(0, GetRxQueueRecord(), 79));
				if(guess == RandomNumber)
				{
						StopTimer();
						PutStringSB(RStr, MAX_STRING);
						PutStringSB(Cols[guess], MAX_STRING);
						AddScore(GetCount(), Round);
						break;
				}else
				{
					PutStringSB(WrongStr, MAX_STRING);
					PutStringSB(InStr, MAX_STRING);
					continue;
				}
				
			} // if we reach this point, we're out of time
			StopTimer();
			SetCount(0);
			PutStringSB(TimeOutStr, MAX_STRING);
			PutStringSB(Cols[RandomNumber], MAX_STRING);
		}
		//end of game, print final score
		PutStringSB(ScStr, MAX_STRING);
		PutNumUB(Score);
		
	} /* do forever */

} /* main */

//Implement Functions

//***********************************************
// Function that gets a random number
// Params: none
// Returns: int
//***********************************************
int RandomLEDColor(void){
	RandomNumber -= (RandomNumber << 6);
	RandomNumber ^= (RandomNumber >> 17);
	RandomNumber -= (RandomNumber << 9);
	RandomNumber ^= (RandomNumber << 4);
	RandomNumber -= (RandomNumber << 3);
	RandomNumber ^= (RandomNumber << 10);
	RandomNumber ^= (RandomNumber >> 15);
	return RandomNumber %= 4;
}

//***********************************************
// Function that maps color commands to ints.
// Params: char c
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
//***********************************************
// Initializes the LEDs.
// Params: void
// Returns: void
//***********************************************
void InitLEDs(void){
	/* Enable clock for PORT B module */
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	/* Select PORT B Pin 8 for GPIO to red LED */
	PORTB->PCR[POS_RED] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 9 for GPIO to green LED */
	PORTB->PCR[POS_GREEN] = PORT_PCR_SET_GPIO;
	/* Select PORT B Pin 10 for GPIO to blue LED */
	PORTB->PCR[POS_BLUE] = PORT_PCR_SET_GPIO;
	/*Set Direction*/
	FPTB->PDDR = PORTB_LEDS_MASK;
	/*set stregnt*/
	/* Turn off red LED */
	FPTB->PSOR = PORTB_LED_RED_MASK;
	/* Turn off green LED */
	FPTB->PSOR = PORTB_LED_GREEN_MASK;
	/* Turn off blue LED */
	FPTB->PSOR = PORTB_LED_BLUE_MASK;
}

//***********************************************
// Adds a number to the player's score.
// Params: int TimeElapsed, int RoundNumber
// Returns: void
//***********************************************
void AddScore(int TimeElapsed, int RoundNumber){
	Score += RoundNumber * (((11 - RoundNumber) * 100) - TimeElapsed);
}
