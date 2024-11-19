/*********************************************************************/
/* Lab Exercise Twelve                                               */
/* Adjusts a servo to one of five positions [1, 5] using  mixed C    */
/* and assembly language.  Prompts user to enter a number from 1 to  */
/* 5, generates a voltage in the range (0, 3.3] V proportional to    */
/* the user's number, converts the voltage to a 10-bit number, and   */
/* set's the servo position [1, 5] based on the magnitude of the 10- */
/* bit digital value.                                                */
/* Name:  R. W. Melton                                               */
/* Date:  November 11, 2024                                          */
/* Class:  CMPE 250                                                  */
/* Section:  All sections                                            */
/*********************************************************************/

#define EXERCISE_12_C (1)

typedef int Int32;
typedef short int Int16;
typedef char Int8;
typedef unsigned int UInt32;
typedef unsigned short int UInt16;
typedef unsigned char UInt8;

/* assembly variables */
extern UInt32 Count;
extern UInt8 RunTimer;
extern UInt32 RxQueueRecord;

/* assembly language subroutines */
char GetChar(void);
void GetStringSB(char String[], int StringBufferCapacity);
void InitUART0(void);
void InitPIT(void);
void InitLEDs(void);
void PutChar(char Character);
void PutNumHex(UInt32);
void PutNumUB(UInt8);
void PutStringSB(char String[], int StringBufferCapacity);
char Dequeue(int null, int RxQueueRecord, int QueueCapacity);
char ToUpperChar(char c);
int ColToInt(char c);
void IsKeyPressed(void);
void AddScore(int RoundNumber, int TimeElapsed);
