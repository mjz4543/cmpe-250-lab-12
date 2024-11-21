            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL05 Assembly with Keil C startup
;R. W. Melton
;November 3, 2020
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s
            OPT  1          ;Turn on listing
;****************************************************************
;EQUates

;---------------------------------------------------------------
;Characters
BS          EQU  0x08
CR          EQU  0x0D
DEL         EQU  0x7F
ESC         EQU  0x1B
LF          EQU  0x0A
NULL        EQU  0x00
TO_UP_SUB	EQU	 'a'-'A'
;---------------------------------------------------------------
;DAC0
DAC0_BITS   EQU   12
DAC0_STEPS  EQU   4096
DAC0_0V     EQU   0x00
;---------------------------------------------------------------
;Servo
SERVO_POSITIONS  EQU  5
;---------------------------------------------------------------
PWM_DUTY_5 		  EQU  2500 ;5% duty cycle
PWM_DUTY_10 	  EQU  6500 ;10% duty cycle
PWM_DUTY_MAX 	  EQU  49970 ;(PWM_PERIOD - 1)
PWM_FREQ          EQU  50
;TPM_SOURCE_FREQ  EQU  48000000
TPM_SOURCE_FREQ   EQU  47972352
TPM_SC_PS_VAL     EQU  4
;PWM_PERIOD       EQU  ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) / 
;                       PWM_FREQ)
;PWM_DUTY_5       EQU  (PWM_PERIOD / 20)  ;  5% duty cycle
;PWM_DUTY_10      EQU  (PWM_PERIOD / 10)  ; 10% duty cycle
PWM_PERIOD        EQU  60000
;---------------------------------------------------------------
;Number output characteristics
MAX_WORD_DECIMAL_DIGITS  EQU  10
;---------------------------------------------------------------
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
XQ_BUF_SZ   EQU   80  ;Xmit queue contents
Q_REC_SZ    EQU   18  ;Queue management record
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)

;****************************************************************
;MACROs

			MACRO
			SETC
			; sets carry flag
			PUSH	{R0}
			MOVS 	R0,#1
			LSRS	R0,R0,#1
			POP		{R0}
			MEND

;---------------------------------------

			MACRO
			CLRC
			; clears carry flag
			PUSH	{R0}
			ADDS	R0,R0,#0
			POP		{R0}
			MEND

;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
		AREA    MyCode,CODE,READONLY
		EXPORT	GetChar
		EXPORT	GetStringSB
		EXPORT	InitUART0
		EXPORT	InitPIT
		EXPORT 	PutChar
		EXPORT	PutNumHex
		EXPORT	PutNumUB			
		EXPORT	PutStringSB
		EXPORT	UART0_IRQHandler
		EXPORT	PIT_IRQHandler
		EXPORT	Dequeue
		EXPORT	IsKeyPressed
		EXPORT	ToUpperChar
		EXPORT	GetCount
		EXPORT	SetCount
		EXPORT	StartTimer
		EXPORT	StopTimer
		EXPORT	GetRxQueueRecord
;>>>>> begin subroutine code <<<<<

; UART0_IRQHandler
UART0_IRQHandler	PROC	{R0-R14}
;------------------------------------------------;
; Interrupt service routine for UART0. Disables  ;
; TDRE interrupts if the transmit queue is       ;
; empty. (TDRE interrupts are re-enabled in the  ;
; GetChar subroutine.)                           ;
;------------------------------------------------;
		
		CPSID	I
		PUSH	{LR}
		LDR		R3,=UART0_BASE
		
		; is TxInt enabled?
		MOVS	R1,#UART0_C2_TIE_MASK
		LDRB	R2,[R3,#UART0_C2_OFFSET]
		ANDS	R2,R2,R1 ; check if TxInt is 0
		BEQ		TxIntDisabled ; if TxInt is disabled, check RxInt
		
		; if TxInt is enabled, check if TDRE bit set
		MOVS	R1,#UART0_S1_TDRE_MASK
		LDRB	R2,[R3,#UART0_S1_OFFSET]
		ANDS	R2,R2,R1 ; check if TDRE is 1
		BEQ		TxIntDisabled ; if TDRE is disabled, check RxInt
		
		; dequeue character from TxQueue
		LDR		R1,=TxQueueRecord
		BL		Dequeue
		BCS		DequeueFail_ISR
			; if the dequeue was succesfull, write out the character
		STRB	R0,[R3,#UART0_D_OFFSET]
		B		TxIntDisabled
			; if the dequeue failed, disable TxInt
DequeueFail_ISR 
		MOVS	R0,#UART0_C2_T_RI 
		STRB	R0,[R3,#UART0_C2_OFFSET]

TxIntDisabled	
		; if TxInt or TDRE is disabled, check RxInt
		MOVS	R1,#UART0_S1_RDRF_MASK
		LDRB	R2,[R3,#UART0_S1_OFFSET]
		ANDS	R2,R2,R1 ; check if RDRF is 1
		BEQ		EndUART0_ISR ; if RDRF is 0, exit

		; read character from UART0_D and enqueue
		LDRB	R0,[R3,#UART0_D_OFFSET]
		LDR		R1,=RxQueueRecord
		BL		Enqueue
		; ** note: character lost if RxQueueBuffer full **

EndUART0_ISR
		CPSIE	I
		POP		{PC}
		ENDP

;-------------------------------------------------------------------------------

; PIT_IRQHandler
PIT_IRQHandler		PROC	{R1-R14}
;------------------------------------------------;
; IRQ handler for the PIT. If RunTimer is set,   ;
; the counter increments, otherwise it does not. ;
; Interrupt cleared on exit.                     ;
;------------------------------------------------;

		CPSID	I
		PUSH	{LR}
		
		LDR		R0,=RunTimer
		LDRB	R0,[R0,#0]
		CMP		R0,#0
		BEQ		PIT_ISR_ClrInt ; if the RunStopwatch variable is clear, clear the interrupt
	; otherwise, increment Count
		LDR		R0,=Count
		LDR		R1,[R0,#0]
		ADDS	R1,R1,#1
		STR		R1,[R0,#0]

PIT_ISR_ClrInt
	; clear PIT Channel 0 interrupt
		LDR 	R0,=PIT_CH0_BASE
		LDR 	R1,=PIT_TFLG_TIF_MASK
		STR 	R1,[R0,#PIT_TFLG_OFFSET]
		
		CPSIE	I
		POP		{PC}
		
		ENDP

;-------------------------------------------------------------------------------

InitUART0	PROC	{R3-R14}
;------------------------------------------------;
; Initializes UART0:                             ;
;  - Sets MCGFLLCLK as UART0 clock source        ;
;  - Sets UART0 to external connection           ;
;  - Enables the UART0 clock                     ;
;  - Enables the PORT B clock                    ;
;  - Bridges PORT B pins 1/2 to UART0 Tx/Rx      ;
;  - Disables UART0 receive and transmit         ;
;  - Sets UART0 for 8N1 @ 9600 baud              ;
;  - Re-enables UART0 receive and transmit       ;
; Returns nothing                                ;
;------------------------------------------------;
		
		PUSH	{LR,R0-R3} ; push R0-R3 to stack

	; Select MCGFLLCLK as UART0 clock source
		LDR		R0,=SIM_SOPT2
		LDR 	R1,=SIM_SOPT2_UART0SRC_MASK
		LDR 	R2,[R0,#0]
		BICS 	R2,R2,R1
		LDR 	R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
		ORRS 	R2,R2,R1
		STR 	R2,[R0,#0]
	; Set UART0 for external connection
		LDR 	R0,=SIM_SOPT5
		LDR 	R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
		LDR 	R2,[R0,#0]
		BICS 	R2,R2,R1
		STR 	R2,[R0,#0]
	; Enable UART0 module clock
		LDR 	R0,=SIM_SCGC4
		LDR 	R1,=SIM_SCGC4_UART0_MASK
		LDR 	R2,[R0,#0]
		ORRS 	R2,R2,R1
		STR 	R2,[R0,#0]
	; Enable PORT B module clock
		LDR 	R0,=SIM_SCGC5
		LDR 	R1,=SIM_SCGC5_PORTB_MASK
		LDR 	R2,[R0,#0]
		ORRS 	R2,R2,R1
		STR 	R2,[R0,#0]
	; Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
		LDR 	R0,=PORTB_PCR2
		LDR 	R1,=PORT_PCR_SET_PTB2_UART0_RX
		STR 	R1,[R0,#0]
	; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
		LDR 	R0,=PORTB_PCR1
		LDR 	R1,=PORT_PCR_SET_PTB1_UART0_TX
		STR 	R1,[R0,#0]
	; Set UART0 IRQ priority
		LDR		R0,=UART0_IPR
		LDR		R2,=NVIC_IPR_UART0_PRI_3
		LDR		R3,[R0,#0]
		ORRS	R3,R3,R2
		STR		R3,[R0,#0]
	; Clear pending interrupts for UART0
		LDR		R0,=NVIC_ICPR
		LDR		R1,=NVIC_ICPR_UART0_MASK
		STR		R1,[R0,#0]
	; Unmask UART0 interrupt
		LDR		R0,=NVIC_ISER
		LDR		R1,=NVIC_ISER_UART0_MASK
		STR		R1,[R0,#0]
	; Disable UART0 receiver and transmitter
		LDR 	R0,=UART0_BASE
		MOVS 	R1,#UART0_C2_T_R
		LDRB 	R2,[R0,#UART0_C2_OFFSET]
		BICS 	R2,R2,R1
		STRB 	R2,[R0,#UART0_C2_OFFSET]
	; Set UART0 for 9600 baud, 8N1 protocol
		MOVS 	R1,#UART0_BDH_9600
		STRB 	R1,[R0,#UART0_BDH_OFFSET]
		MOVS 	R1,#UART0_BDL_9600
		STRB 	R1,[R0,#UART0_BDL_OFFSET]
		MOVS 	R1,#UART0_C1_8N1
		STRB 	R1,[R0,#UART0_C1_OFFSET]
		MOVS 	R1,#UART0_C3_NO_TXINV
		STRB 	R1,[R0,#UART0_C3_OFFSET]
		MOVS 	R1,#UART0_C4_NO_MATCH_OSR_16
		STRB 	R1,[R0,#UART0_C4_OFFSET]
		MOVS 	R1,#UART0_C5_NO_DMA_SSR_SYNC
		STRB 	R1,[R0,#UART0_C5_OFFSET]
		MOVS 	R1,#UART0_S1_CLEAR_FLAGS
		STRB 	R1,[R0,#UART0_S1_OFFSET]
		MOVS 	R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
		STRB 	R1,[R0,#UART0_S2_OFFSET]
	; Enable UART0 receiver and transmitter
		MOVS 	R1,#UART0_C2_T_R
		STRB 	R1,[R0,#UART0_C2_OFFSET]
	; Initialize transmit/receive queue structures
		LDR		R0,=TxQueueBuffer
		LDR		R1,=TxQueueRecord
		MOVS	R2,#80
		BL		InitQueue
		LDR		R0,=RxQueueBuffer
		LDR		R1,=RxQueueRecord
		MOVS	R2,#80
		BL		InitQueue
		
		POP	 	{PC,R0-R3} ; yank R0-R3 off the stack

		ENDP
	
;-------------------------------------------------------------------------------

InitPIT		PROC	{R0-R14}
;------------------------------------------------;
; Initializes the PIT to use ISRs, as discussed  ;
; in lecture.                                    ;
;------------------------------------------------;

		PUSH	{R0-R3}
		
	; enable module clock for PIT
		LDR 	R0,=SIM_SCGC6
		LDR 	R1,=SIM_SCGC6_PIT_MASK
		LDR 	R2,[R0,#0] ;current SIM_SCGC6 value
		ORRS 	R2,R2,R1 ;only PIT bit set
		STR 	R2,[R0,#0] ;update SIM_SCGC6
	; disable PIT timer 0 (PIT_TCTRL0) 
		LDR 	R0,=PIT_CH0_BASE
		LDR 	R1,=PIT_TCTRL_TEN_MASK 
		LDR 	R2,[R0,#PIT_TCTRL_OFFSET] 
		BICS	R2,R2,R1
	; set PIT interrupt priority
		LDR 	R0,=PIT_IPR
		LDR 	R1,=(NVIC_IPR_PIT_MASK)
		LDR 	R3,[R0,#0]
		BICS 	R3,R3,R1
		STR 	R3,[R0,#0]
	; clear PIT channel 0 interrupt
		LDR 	R0,=PIT_CH0_BASE
		LDR 	R1,=PIT_TFLG_TIF_MASK
		STR 	R1,[R0,#PIT_TFLG_OFFSET]
	; unmask PIT interrupts
		LDR 	R0,=NVIC_ISER
		LDR 	R1,=PIT_IRQ_MASK
		STR 	R1,[R0,#0]
	; enable PIT module
		LDR 	R0,=PIT_BASE
		LDR 	R1,=PIT_MCR_EN_FRZ
		STR 	R1,[R0,#PIT_MCR_OFFSET]
	; set PIT timer 0 for 0.01 sec.
		LDR 	R0,=PIT_CH0_BASE
		LDR 	R1,=PIT_LDVAL_10ms
		STR 	R1,[R0,#PIT_LDVAL_OFFSET]
	; Enable PIT timer 0 with interrupt
		LDR 	R0,=PIT_CH0_BASE
		MOVS 	R1,#PIT_TCTRL_CH_IE
		STR 	R1,[R0,#PIT_TCTRL_OFFSET]

		POP		{R0-R3}
		BX		LR
		ENDP

;-------------------------------------------------------------------------------

GetChar		PROC	{R1-R14}
;------------------------------------------------;
; An ISR-driven receive: returns a character     ;
; from the terminal if the RDRF (receive data    ;
; register full) flag in UART0_S1 is set.        ;
; R0: character received                         ;
;------------------------------------------------;

		PUSH	{LR,R1}
		
		LDR		R1,=RxQueueRecord

GetCharDequeueLoop
		CPSID	I		; ** CRITICAL SECTION **
		BL		Dequeue	; ** CRITICAL SECTION **
		CPSIE	I		; ** CRITICAL SECTION **
		BCS		GetCharDequeueLoop

		POP		{PC,R1}
		ENDP

;-------------------------------------------------------------------------------

IsKeyPressed	PROC	{R1-R14}
;------------------------------------------------;
; Checks if a keyboard key is pressed. If it is, ;
; returns a non-zero boolean value.              ;
;------------------------------------------------;
		
		LDR		R0,=RxQueueRecord
		LDRB	R0,[R0,#NUM_ENQD]
		CMP		R0,#0
		BGT		QueueNotEmpty
QueueEmpty
		MOVS	R0,#0x00
		B		IsKeyPressedEnd
QueueNotEmpty
		MOVS	R0,#0xFF
IsKeyPressedEnd
		BX		LR
		
		ENDP

;-------------------------------------------------------------------------------

PutChar		PROC	{R0-R14}
;------------------------------------------------;
; An ISR-driven transmit: returns a character    ;
; from the terminal if the TDRE (transmit data   ;
; register empty) flag in UART0_S1 is set.       ;
; Returns nothing                                ;
;------------------------------------------------;

		PUSH	{LR,R0-R1}
		
		LDR		R1,=TxQueueRecord

PutCharEnqueueLoop
		CPSID	I		; ** CRITICAL SECTION **
		BL		Enqueue	; ** CRITICAL SECTION **
		CPSIE	I		; ** CRITICAL SECTION **
		BCS		PutCharEnqueueLoop
	; enable TxInt again
		LDR		R0,=UART0_BASE
		MOVS	R1,#UART0_C2_TI_RI
		STRB 	R1,[R0,#UART0_C2_OFFSET]

		POP		{PC,R0-R1}
		ENDP

;-------------------------------------------------------------------------------
	
InitQueue	PROC	{R1-R14}
;------------------------------------------------;
; Initializes queue record structure with record ;
; address in R1, buffer start address in R0, and ;
; size in R2.                                    ;
; Inputs:                                        ;
;	R0 - memory address of queue buffer          ;
; 	R1 - memory address of queue record          ;
;	R2 - size of queue buffer                    ;
; Outputs: None                                  ;
;------------------------------------------------;

		PUSH  	{R0-R2}
		
		STR   	R0,[R1,#IN_PTR]		; store starting address of queue buffer in IN_PTR  
		STR   	R0,[R1,#OUT_PTR] 	; store starting address of queue buffer in OUT_PTR  
		STR   	R0,[R1,#BUF_STRT] 	; store starting address of queue buffer in BUF_STRT 
		ADDS  	R0,R0,R2 			; add queue capacity to R0
		STR   	R0,[R1,#BUF_PAST] 	; store first address past end of queue buffer in BUF_PAST
		STRB  	R2,[R1,#BUF_SIZE] 	; store queue capacity in BUF_SIZE
		MOVS  	R0,#0
		STRB  	R0,[R1,#NUM_ENQD] 	; store that nothing is enqueued in NUM_ENQD
		
		POP		{R0-R2}
		BX		LR
		
		ENDP

;-------------------------------------------------------------------------------
	
Dequeue		PROC	{R1-R14}
;------------------------------------------------;
; Attempts to get a character from the queue     ;
; with record structure address in R1. If the    ;
; queue isn't empty, a single character is       ;
; dequeued into R0 and C is cleared, otherwise C ;
; is set.                                        ;
; Inputs:                                        ;
;	R1 - memory address of queue record          ;
;	R2 - size of queue buffer                    ;
; Outputs:                                       ;
;	R0 - character dequeued                      ;
; 	C set on faliure/cleared on success          ; 
;------------------------------------------------;

		PUSH 	{R1-R6}
		LDR		R2,[R1,#OUT_PTR]  ; R2 <- OUT_PTR
		LDRB	R3,[R2,#0] 		  ; R3 <- *OUT_PTR
		LDR		R4,[R1,#BUF_STRT] ; R4 <- BUF_STRT
		LDR		R5,[R1,#BUF_PAST] ; R5 <- BUF_PAST
		LDRB	R6,[R1,#NUM_ENQD] ; R6 <- NUM_ENQD

		CMP		R6,#0 ; if NUM_ENQD <= 0
		BLE		DequeueSetCarry ; there's nothing in the queue, set C and exit
		
		MOVS	R0,R3 ; get queue item at *OUT_PTR
		SUBS	R6,R6,#1 ; decrement NUM_ENQD
		ADDS	R2,R2,#1 ; increment OUT_PTR
		
		CMP		R2,R5 ; if OUT_PTR >= BUF_PAST
		BGE		DequeueAdjustOutPtr ; adjust OUT_PTR to be BUF_STRT
		B		DequeueClearCarry ; reflect success in C bit
		
DequeueAdjustOutPtr
		MOVS	R2,R4 ; adjust OUT_PTR to be BUF_STRT
		B		DequeueClearCarry ; reflect success in C bit

DequeueSetCarry
		SETC
		B		DequeueEnd
		
DequeueClearCarry
		CLRC
		B		DequeueEnd

DequeueEnd
		STR		R2,[R1,#OUT_PTR]  ; OUT_PTR <- R2
		STR		R4,[R1,#BUF_STRT] ; BUF_STRT <- R4
		STR		R5,[R1,#BUF_PAST] ; BUF_PAST <- R5
		STRB	R6,[R1,#NUM_ENQD] ; NUM_ENQD <- R6
		
		POP		{R1-R6}
		BX		LR

		ENDP

;-------------------------------------------------------------------------------
	
Enqueue		PROC	{R0-R14}
;------------------------------------------------;
; Attempts to put a character in R0 into the     ;
; queue with record structure in R1. If the      ;
; queue isn't full, a single character is        ;
; enqueued and C is cleared, otherwise C is set. ;
; Inputs:                                        ;
;	R1 - memory address of queue record          ;
;	R2 - size of queue buffer                    ;
; Outputs:                                       ;                      ;
; 	C set on faliure/cleared on success          ; 
;------------------------------------------------;

		PUSH 	{R0-R7}
		LDR		R2,[R1,#IN_PTR]   ; R2 <- IN_PTR
		LDRB	R3,[R2,#0] 		  ; R3 <- *IN_PTR
		LDR		R4,[R1,#BUF_STRT] ; R4 <- BUF_STRT
		LDR		R5,[R1,#BUF_PAST] ; R5 <- BUF_PAST
		LDRB	R6,[R1,#NUM_ENQD] ; R6 <- NUM_ENQD
		LDRB	R7,[R1,#BUF_SIZE] ; R7 <- BUF_SIZE

		CMP		R6,R7 ; if NUM_ENQD >= BUF_SIZE
		BGE		EnqueueSetCarry ; the queue is full, set C and exit
		
		MOVS	R3,R0 ; put new queue item into R3
		STRB	R3,[R2,#0] ; *IN_PTR <- R3
		ADDS	R6,R6,#1 ; increment NUM_ENQD
		ADDS	R2,R2,#1 ; increment IN_PTR
		
		CMP		R2,R5 ; if IN_PTR >= BUF_PAST
		BGE		EnqueueAdjustInPtr ; adjust OUT_PTR to be BUF_STRT
		B		EnqueueClearCarry ; reflect success in C bit
		
EnqueueAdjustInPtr
		MOVS	R2,R4 ; adjust OUT_PTR to be BUF_STRT
		B		EnqueueClearCarry ; reflect success in C bit

EnqueueSetCarry
		SETC
		B		EnqueueEnd
		
EnqueueClearCarry
		CLRC
		B		EnqueueEnd

EnqueueEnd
		STR		R2,[R1,#IN_PTR]   ; IN_PTR <- R2
		STR		R4,[R1,#BUF_STRT] ; BUF_STRT <- R4
		STR		R5,[R1,#BUF_PAST] ; BUF_PAST <- R5
		STRB	R6,[R1,#NUM_ENQD] ; NUM_ENQD <- R6
		
		POP		{R0-R7}
		BX		LR

		ENDP

;-------------------------------------------------------------------------------
	
PutNumHex	PROC	{R1-R14}
;------------------------------------------------;
; Prints the hexidecimal number in R0 to the     ;
; terminal with leading zeroes.                  ;
; Inputs:                                        ;
; 	- R0: hexidecimal number to print            ;
; Output: None                                   ;
;------------------------------------------------;

		PUSH	{LR,R0-R7}

		MOVS	R2,R0 ; back up R0 in R2
		MOVS	R3,#16 ; sentinel value
		PUSH	{R3} ; push 16 as sentinel value
		MOVS	R4,#8 ; loop counter
		MOVS	R1,#2_1111 ; move low nibble mask to R1
		
		
PutNumHexPushLoop
		CMP		R4,#0
		BEQ		PutNumHexPrintLoop ; we're done, start printing

		
		ANDS	R0,R0,R1 ; store low nibble of R0 in R0
		PUSH	{R0} ; push low nibble to stack
		LSRS	R2,R2,#4 ; shift R2 right by a nibble
		MOVS	R0,R2 ; move shifted copy of R0 back into R0
		
		SUBS	R4,R4,#1
		B	PutNumHexPushLoop
		
		
PutNumHexPrintLoop
		POP		{R0} ; pop result to R0
		CMP		R0,#16 ; did we reach the end?
		BEQ		PutNumHexEnd ; if we did, terminate
		
		CMP		R0,#10 ; if we're greater than 9...
		BGE		PutNumHexCorrectAF ; add a corrective term

PutNumHexMakeChar
		ADDS	R0,R0,#0x30 ; add 0x30 to make ASCII out of a number 0-9
		BL		PutChar ; print ASCII representation of the number
		B		PutNumHexPrintLoop ; loop again

PutNumHexCorrectAF
		ADDS	R0,R0,#0x7 ; correct for numbers greater than 10
		B	PutNumHexMakeChar

PutNumHexEnd
		POP		{PC,R0-R7}
		
		ENDP

;-------------------------------------------------------------------------------
	
PutNumUB	PROC	{R1-R14}
;------------------------------------------------;
; Prints the unsigned decimal byte in R0 to      ;
; the terminal with leading zeroes.              ;
; Inputs:                                        ;
; 	- R0: unsigned decimal byte to print         ;
; Output: None                                   ;
;------------------------------------------------;

		PUSH	{LR,R0-R1}			
		
		MOVS	R1,#2_00001111	; put mask into R1
		ANDS	R0,R0,R1		; AND with mask to preserve only LSB
		BL		PutNumU			; print with PutNumU
		
		POP		{PC,R0-R1}				

		ENDP

;-------------------------------------------------------------------------------
	
GetStringSB		PROC	{R1-R14}
;------------------------------------------------;
; Input a string from the terminal keyboard into ;
; the memory address specified in R0, preventing ;
; buffer overrun of the capacity specified in    ;
; R1.                                            ;
; Inputs:                                        ;
; 	R0 - memory address of string                ;
;	R1 - buffer capacity                         ;
; Outputs:                                       ;
;	R0 - memory address of string                ;
;------------------------------------------------;

		PUSH 	{LR,R0-R5}

		MOVS 	R2,R0 ; back up string address in R2, since GetChar uses R0
		SUBS 	R3,R1,#1 ; move actual length (R1-1) into R3
		MOVS 	R4,#0 ; zero R4 to hold number of characters typed

GetStringGetCharLoop
		BL 		GetChar ; get character in R0
		MOVS 	R5,R4 ; store current length-1 in R5 for storage offset
		ADDS 	R4,R4,#1 ; increment number of characters in the string

		CMP 	R0,#CR ; did we press enter?
		BEQ 	GetStringPressedEnter

		CMP 	R4,R3 ; did we type more characters than buffer size-1?
		BGT 	GetStringDecNumChars ; If R4 > buffer size-1, decrement number of characters in the string

		; if we didn’t overrun the buffer...
		BL 		PutChar ; output character to terminal
		STRB 	R0,[R2,R5] ; store character to string address, indexed by (current length-1)
		B 		GetStringGetCharLoop ; again!

GetStringDecNumChars
		SUBS 	R4,R4,#1 ; undo the incrementing of the number of characters in the string
		B 		GetStringGetCharLoop ; branch back

GetStringPressedEnter
		MOVS 	R0,#NULL ; store null in R0
		SUBS 	R4,R4,#1 ; subtract 1 to get total number of chararcters, not including the enter key
		STRB 	R0,[R2,R5] ; store null terminator

		POP 	{PC,R0-R5}

		ENDP
			
;-------------------------------------------------------------------------------
	
PutStringSB		PROC	{R4-R14}
;------------------------------------------------;
; Displays a null-terminated string in memory to ;
; the terminal screen.                           ;
; Inputs:                                        ;
;	R0 - memory address of string                ;
;	R1 - buffer capacity                         ;
; Outputs: none                                  ;
;------------------------------------------------;

		PUSH	{LR,R0-R3}
		
		MOVS	R2,#0 ; counts number of characters printed
		MOVS	R3,R0 ; copy string memory address to R3
		
PutStringLoop
		LDRB	R0,[R3,R2] ; load R0 with character in string to print, indexed by R2
		CMP		R0,#NULL ; is R0 null?
		BEQ		PutStringEnd ; bail out!
		BL		PutChar ; print character in R0 if it isn't null
		
		ADDS	R2,R2,#1 ; increment R2
		CMP		R2,R1 ; is the current number of printed character less than or equal to the buffer capacity?
		BLE		PutStringLoop ; if so, branch back, otherwise fall through and exit

PutStringEnd		
		POP		{PC,R0-R3}
		BX		LR
		
		ENDP
			
;-------------------------------------------------------------------------------
	
PutNumU		PROC	{}
;------------------------------------------------;
; Displays the text decimal representation of    ;
; the unsigned word value in R0 to the terminal  ;
; screen. There are no leading zeroes.           ;
; Inputs:                                        ;
;	R0 - unsigned number to output               ;                     ;
; Outputs: none                                  ;
;------------------------------------------------;

		PUSH	{LR,R0-R2}
		
		MOVS	R1,R0 ; move dividend to R1
		MOVS	R0,#10
		PUSH	{R0} ; push 10 as a sentinel value to signal the end of the list of remainders on the stack
		
PutNumConvertLoop
		MOVS	R0,#10 ; move 10 to divisor
		BL		DIVU
		PUSH	{R1} ; push remainder to stack
		MOVS	R1,R0 ; move quotient into R1 to become the dividend next time
		
		CMP		R0,#0 ; if the quotient is zero, we're done
		BNE		PutNumConvertLoop ; otherwise loop

PutNumPrintLoop
		POP		{R0} ; pop result to R0
		CMP		R0,#10 ; did we reach the end?
		BEQ		PutNumEnd ; if we did, terminate
		ADDS	R0,R0,#0x30 ; add 0x30 to make ASCII out of a number 0-9
		BL		PutChar ; print ASCII representation of the number
		B		PutNumPrintLoop ; loop again

PutNumEnd		
		POP		{PC,R0-R2}

		ENDP

;-------------------------------------------------------------------------------

DIVU	PROC	{R2-R14}
;------------------------------------------------;
; Input: R0 (divisor), R1 (dividend)             ;
; Output: R0 (quotient), R1 (remainder)          ;
; C flag clear for invalid result (R1 or R0 = 0) ;
; C flag set for valid result                    ;
;------------------------------------------------;

			PUSH	{R2,R3,R4} ; store R2-R4 on stack for later retrieval

			CMP		R0,#0
			BEQ		DvsrZero ; divisor = 0: set carry and end
			CMP		R1,#0
			BEQ		DvdndZero ; dividend = 0: set R0 and sR1 to 0, clear carry and end

			MOVS	R2,#0 ; zero R2 (quotient)

DivLoop		CMP		R1,R0 ; dividend >= divisor
			BLO		EndDivLoop
			SUBS	R1,R1,R0 ; dividend = dividend - divisor
			ADDS	R2,R2,#1 ; quotient = quotient + 1
			B		DivLoop
EndDivLoop	MOVS	R0,R2 ; move quotient to R0 (remainder is still in R1)	
			B		ClearCarry ; clear carry flag and end


DvdndZero	MOVS	R0,#0 ; zero divisor
			MOVS	R1,#0 ; zero dividend
			B		ClearCarry

DvsrZero	; set carry flag and end
			MRS		R3,APSR
			MOVS	R4,#0x20
			LSLS	R4,R4,#24
			ORRS	R3,R3,R4
			MSR		APSR,R3
			B		DivEnd

ClearCarry	; clear carry flag and end
			MRS		R3,APSR
			MOVS	R4,#0x20
			LSLS	R4,R4,#24
			BICS	R3,R3,R4
			MSR		APSR,R3
			; fall through

DivEnd		POP		{R2,R3,R4} ; pop R2-R4 from stack
			BX		LR
			
			ENDP

;-------------------------------------------------------------------------------

ToUpperChar		PROC	{R0,R2-R14}
;**********************************************************
; If a character is contained in R1, then it is made
; uppercase if it is not already
;	Inputs:
;		R1, The character to make uppercase
;	Outputs:
;		R1, The uppercasse character
;**********************************************************
				CMP		R1,#'a'
				BLO		ToUpEnd				;not a lowercase character
				CMP		R1,#'z'
				BHI		ToUpEnd				;not a lowercase character
				SUBS	R1,R1,#TO_UP_SUB	;make character uppercase

ToUpEnd			BX		LR
				ENDP
						
;-------------------------------------------------------------------------------

GetCount		PROC	{R1-R14}
;**********************************************************
; Gets the Count variable.
;**********************************************************
				LDR		R0,=Count
				LDR		R0,[R0,#0]
				BX		LR
				ENDP

;-------------------------------------------------------------------------------

SetCount		PROC	{R1-R14}
;**********************************************************
; Sets the Count variable to the value in R0.
;**********************************************************
				PUSH	{R1}
				LDR		R1,=Count
				STR		R0,[R1,#0]
				POP		{R1}
				BX		LR
				ENDP

;-------------------------------------------------------------------------------

StartTimer		PROC	{R1-R14}
;**********************************************************
; Sets the RunTimer variable to 0xFF.
;**********************************************************
				PUSH	{R0-R1}
				LDR		R1,=RunTimer
				MOVS	R0,#0
				STRB	R0,[R1,#0]
				POP		{R0-R1}
				BX		LR
				ENDP

;-------------------------------------------------------------------------------

StopTimer		PROC	{R1-R14}
;**********************************************************
; Sets the RunTimer variable to 0x0.
;**********************************************************
				PUSH	{R0-R1}
				LDR		R1,=RunTimer
				MOVS	R0,#0xFF
				STRB	R0,[R1,#0]
				POP		{R0-R1}
				BX		LR
				ENDP
					
;-------------------------------------------------------------------------------

GetRxQueueRecord		PROC	{R1-R14}
;**********************************************************
; Sets the address of the RxQueueRecord variable.
;**********************************************************
				LDR		R0,=RxQueueRecord
				BX		LR
				ENDP

;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
			
			
			
;>>>>>   end constants here <<<<<
			ALIGN
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
			EXPORT	RxQueueRecord	
			EXPORT	RunTimer
			EXPORT	Count
;>>>>> begin variables here <<<<<

TxQueueRecord		SPACE	18
	ALIGN
TxQueueBuffer		SPACE	80
RxQueueRecord		SPACE 	18
	ALIGN
RxQueueBuffer		SPACE	80

RunTimer			SPACE	1
	ALIGN
Count				SPACE	4

;>>>>>   end variables here <<<<<
            END