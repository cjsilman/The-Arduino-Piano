;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Final Project Buttons/Notes Test.asm
; Polling Inputs, Playing Notes			
; Created: 2/12/2021 1:00 PM     
; Author : Chris Silman and Trevor Spurbeck                
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; KEEP THIS CODE FOR ALL LABS UNLESS INSTRUCTED TO DO OTHERWISE;
;                                                              ;
.ORG 0x0000           ; the next instruction has to be         ;
                      ; written to address 0x0000              ;
;rjmp START           ; the reset vector: jump to "main"       ;
START:                                                         ;
ldi r16, low(RAMEND)  ; set up the stack pointer to point at   ;
out SPL, r16          ; the bottom of RAM (standard method)    ;
ldi r16, high(RAMEND)                                          ;
out SPH, r16                                                   ;
;                                                              ;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; Actual Program Starts Here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
SBI DDRB, 2	; make pin 10 on Uno an output pin for Buzzer		;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

; This is the format that the "sound" of our notes are in. Essentially it is
; two $XX $XX hex numbers stored as one large number. The higher number
; represents the inner part of a loop, while the lower one represents the outer loop
.EQU A4NOTE = $3F5D ; Loop length of approx. 2273 MicroSeconds
.EQU B4NOTE = $3F53 ; Loop length of approx. 2024 MicroSeconds
.EQU C4NOTE = $804F	; Loop length of approx. 1909 MicroSeconds
.EQU D4NOTE = $7F47 ; Loop length of approx. 1701 MicroSeconds
.EQU E4NOTE = $7F3F ; Loop length of approx. 1515 MicroSeconds
.EQU F4NOTE = $803B ; Loop length of approx. 1433 MicroSeconds
.EQU G4NOTE = $7F35 ; Loop length of approx. 1276 MicroSeconds

; Values to set up our ADC
.EQU ADCsetup = 0x87		; ADC on, clk/128 (high accuracy)
.EQU ADMUXsetup = 0x41		; Vref=5V,right-justified, pin A1

LDI R24, $00
LDI R25, $01
LDI R26, $01


; Look up table 
USARTchords:
;;;;"0123456789ABCDEF"
.DB "C-MAJOR "
.DB "D-MINOR "
.DB "E-MINOR "
.DB "C-E 3RD "
.DB "D-F 3RD "
.DB "E-G 3RD "
.DB "F-A 3RD "
.DB "G-B 3RD "


;---------------------------- PROGRAM STARTS HERE ----------------------------;
Setup:
	RCALL InitUSART		; Sets up USART (57600 baud)
	RCALL InitADC		; Sets up ADC
	RCALL Delay100Milli ; Delay to make sure it all sets up

; Loop to continuously input from Port D and Poll button presses
Loop:
	RCALL ReadADC		; Reads Potentiometer Value
	RCALL AnalogToThird ; Converts our potentiometer value (0 to 1023 to range 0 to 2) 
						; Stores in $202
	RCALL OctaveToASCII ; Convert Octave range to ASCII and stores in $203
	IN R16, PIND		; Reads Port D
	ANDI R16, 252		; Bitmask R16
	IN R17, PINB		; Reads Pin B
	ANDI R17, 2			; Bitmask R17
	ADD R16, R17


;------------- SINGLE NOTES BEGIN -------------;


; Case 0000010X, PortD pin 2, C
CasePin2:
	LDI R19, HIGH(C4NOTE)	; Preload the note value so we do not need to check later
	LDI R18, LOW(C4NOTE)	; This just gets overwritten later if we do not press this button
	LDI R23, $43			;ASCII C
	CPI R16, $04
	BRNE CasePin3
	RCALL PlayNote			; If the button on pin 2 is pressed, go to play note

; Case 0000100X, PortD pin 3, D
CasePin3:
	LDI R19, HIGH(D4NOTE)
	LDI R18, LOW(D4NOTE)
	LDI R23, $44			;ASCII D
	CPI r16, $08
	BRNE CasePin4
	RCALL PlayNote

; Case 0001000X, PortD pin 4, E
CasePin4:
	LDI R19, HIGH(E4NOTE)
	LDI R18, LOW(E4NOTE)
	LDI R23, $45			;ASCII E
	CPI r16, $10
	BRNE CasePin5
	RJMP PlayNote

; Case 0010000X, PortD pin 5, F
CasePin5:
	LDI R19, HIGH(F4NOTE)
	LDI R18, LOW(F4NOTE)
	LDI R23, $46			;ASCII F
	CPI r16, $20
	BRNE CasePin6
	RJMP PlayNote

; Case 0100000X, PortD pin 6, G
CasePin6:
	LDI R19, HIGH(G4NOTE)
	LDI R18, LOW(G4NOTE)
	LDI R23, $47			;ASCII G
	CPI r16, $40
	BRNE CaseA
	RJMP PlayNote

; Case 1000000X, PortD Pin 7, A
CaseA:	
	LDI R19, HIGH(A4NOTE)
	LDI R18, LOW(A4NOTE)
	LDI R23, $41			;ASCII A
	CPI R16, $80
	BRNE CaseB
	RJMP PlayNote

; Case X000001X, PortB Pin 9, B
CaseB:	
	LDI R19, HIGH(B4NOTE)
	LDI R18, LOW(B4NOTE)
	LDI R23, $42			;ASCII B
	CPI R16, $02
	BRNE CaseCmaj
	RJMP PlayNote


;---------- TRIADS BEGIN (Three Note Cycles) ------------;


; Case 0101010X, PortD pin 2, 4, and 6 C major
CaseCmaj:		  ; Cycles through C, E, and G
LDI R23, $48	  ; ASCII Cmaj
CPI R16, $54
BRNE CaseDmin   
LDI R27, 0x07
Loop2:						; Loops are used to allow our ears to register each note individually
	LDI R19, HIGH(C4NOTE)
	LDI R18, LOW(C4NOTE)	
	RCALL PlayNote2			; If the button on pin 2 is pressed, go to play note
	Dec R27
	BRNE Loop2   
LDI R27, 0x09
Loop3:
	LDI R19, HIGH(E4NOTE)
	LDI R18, LOW(E4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop3   
LDI R27, 0x0A
Loop4: 
	LDI R19, HIGH(G4NOTE)
	LDI R18, LOW(G4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop4
CPI R16, $54		; The code will cycle through three notes until the user
BREQ CaseCmaj		; stops clicking all three buttons. This cycle resembles
RJMP Loop   	    ; a chord while only using a single audio output.


; Case 1010100X, PortD pin 3, 5, 7, D minor
CaseDmin:			; Cycles through D, F, and A
LDI R23, $49		; ASCII Dmin
CPI R16, $A8
BRNE CaseEmin   
LDI R27, 0x07
Loop5:
	LDI R19, HIGH(D4NOTE)
	LDI R18, LOW(D4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop5   
LDI R27, 0x09
Loop6: 
	LDI R19, HIGH(F4NOTE)
	LDI R18, LOW(F4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop6   
LDI R27, 0x0A
Loop7: 
	LDI R19, HIGH(A4NOTE)
	LDI R18, LOW(A4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop7
CPI R16, $A8
BREQ CaseDmin
RJMP Loop


; Case 0101001X, PortD pin 3, 5, 7, E minor
CaseEmin:	    ; Cycles through E, G, and B
LDI R23, $4A	; ASCII Emin
CPI R16, $52
BRNE CaseCE   
LDI R27, 0x07
Loop8: 
	LDI R19, HIGH(E4NOTE)
	LDI R18, LOW(E4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop8   
LDI R27, 0x09
Loop9: 
	LDI R19, HIGH(G4NOTE)
	LDI R18, LOW(G4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop9   
LDI R27, 0x0A
Loop10: 
	LDI R19, HIGH(B4NOTE)
	LDI R18, LOW(B4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop10
CPI R16, $52
BREQ CaseEmin
RJMP Loop


;--------- THIRDS BEGIN (2 Note Cycles) ----------;


; Case 0001010X, PortD pin 2, 4 CE major third
CaseCE:			      ; Cycles through C and E
LDI R23, $4B		  ; ASCII CE
CPI R16, $14
BRNE CaseDF   
LDI R27, 0x07
Loop11: 
	LDI R19, HIGH(C4NOTE)
	LDI R18, LOW(C4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop11   
LDI R27, 0x09
Loop12: 
	LDI R19, HIGH(E4NOTE)
	LDI R18, LOW(E4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop12   
CPI R16, $14			; The code will cycle through two notes until the user stops
BREQ CaseCE				; clicking both buttons. This cycle resembles an interval in
RJMP Loop   			; music called a third while only using a single audio output.


; Case 0010100X, PortD pin 3, 5 DF minor third
CaseDF:				  ; Cycles through D and F
LDI R23, $4C		  ; ASCII DF
CPI R16, $28
BRNE CaseEG   
LDI R27, 0x07
Loop13: 
	LDI R19, HIGH(D4NOTE)
	LDI R18, LOW(D4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop13   
LDI R27, 0x09
Loop14: 
	LDI R19, HIGH(F4NOTE)
	LDI R18, LOW(F4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop14   
CPI R16, $28
BREQ CaseDF
RJMP Loop

; Case 0101000X, PortD pin 4, 6 EG minor third
CaseEG:			   	  ; Cycles through E and G
LDI R23, $4D		  ; ASCII EG
CPI R16, $50
BRNE CaseFA 
LDI R27, 0x07
Loop15: 
	LDI R19, HIGH(E4NOTE)
	LDI R18, LOW(E4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop15   
LDI R27, 0x09
Loop16: 
	LDI R19, HIGH(G4NOTE)
	LDI R18, LOW(G4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop16   
CPI R16, $50
BREQ CaseEG
RJMP Loop


; Case 1010000X, PortD pin 5, 7 FA major third
CaseFA:					; Cycles through F and A
LDI R23, $4E			; ASCII FA
CPI R16, $A0
BRNE CaseGB   
LDI R27, 0x07
Loop17: 
	LDI R19, HIGH(F4NOTE)
	LDI R18, LOW(F4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop17  
LDI R27, 0x09
Loop18: 
	LDI R19, HIGH(A4NOTE)
	LDI R18, LOW(A4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop18 
CPI R16, $A0
BREQ CaseFA
RJMP Loop


; Case 0100001X, PortD pin 6, 1 GB major third
CaseGB:					; Cycles through G and B
LDI R23, $4F			; ASCII GB
CPI R16, $42
BRNE CaseDefault   
LDI R27, 0x07
Loop19: 
	LDI R19, HIGH(G4NOTE)
	LDI R18, LOW(G4NOTE)	
	RCALL PlayNote2
	Dec R27
	BRNE Loop19  
LDI R27, 0x09
Loop20: 
	LDI R19, HIGH(B4NOTE)
	LDI R18, LOW(B4NOTE)
	RCALL PlayNote2
	Dec R27
	BRNE Loop20 
CPI R16, $42
BREQ CaseGB
RJMP Loop

; Case 0000000X, no button pressed
CaseDefault:
	
	LDI R26, $01
	LDI R24, $00
	CPI R16, $01
	BRNE SafeGuard
	NOP
;	RJMP PrintUSART
	RJMP Loop

;Safeguard incase we have no specific case or it gets here, jump back to the top
SafeGuard:
	RJMP Loop

;---------------------------- MAIN LOOP ENDS HERE ----------------------------;


;---------------------------- BUZZER SUBROUTINES -----------------------------;
; Play Note - Assumes R19 contains HIGH of Note and R18 contains LOW of Note
PlayNote:
	RCALL OctaveCheck		; Note values are pre-loaded when polling button presses.
	CPI R17, $02			; Must check what octave we are in and make adjustments
	BREQ ConvertHighOctave
	CPI R17, $00
	BREQ ConvertLowOctave
ProduceSound:
	SBI PORTB, 2    ; Turn on Pin 10
	RCALL Delay    
	CBI PORTB, 2    ; Turn off Pin 10
	RCALL Delay   
	RJMP Loop

PlayNote2:
	RCALL OctaveCheck		
	CPI R17, $02			
	BREQ ConvertHighOctave
	CPI R17, $00
	BREQ ConvertLowOctave
ProduceSound2:
	SBI PORTB, 2    
	RCALL Delay    
	CBI PORTB, 2    
	RCALL Delay   
	RET

; Checks what octave we are in by taking our ADC value and comparing it to
; 2, 1, and 0. It returns R17 which tells us what octave we are in
OctaveCheck:			; This portion of the octave check is used in
	CPI R26, $01		; USART communication. R26 works as a check to see
	BRNE OctaveCheck2	; whether or not we have already sent to the serial monitor
	ADD R24, R25		; If we have, it just skips this and checks the octave.
	CPI R24, $16
	BREQ PrintUSART
OctaveCheck2:			; Compares our ADC read value with 2, 1, 0 and stores in R17
	LDS R16, $202
	LDI R17, $02
	SUB R16, R17
	BREQ Return ; Octave 2 (5)
	LDS R16, $202
	LDI R17, $01
	SUB R16, R17
	BREQ Return ; Octave 1 (4)
	LDI R17, $00
	RJMP Return ; Octave 0 (3)

Return:
	RET

PrintUSART:				; Jumps to PrintInfo to print to our USART
	RJMP PrintInfo		; This was required as PrintInfo is outside of
	RET					; BREQ range

; Converts our predetermined X4 note value to X3
; Assumes HIGH of note value stored in R19, Low in R18
ConvertLowOctave:
	SEC					; Sets carry bit, this is necessary to convert to lower octave
	ROL R18				; A close approximation is left shifting our note value
	DEC R18				; Minor Tuning Adjustment
	RJMP ProduceSound

	
; Converts our predetermined X4 note value to X5
ConvertHighOctave:
	CLC					; Clears carry bit, this is necessary to convert to higher octave
	ROR R18				; A close approximation is right shifting our note value
	INC R19				; Minor Tuning Adjustment
	RJMP ProduceSound

; Delay - loops value of note that needs to be played
; Assumes HIGH stored in R19, LOW stored in R18
Delay:		
	PUSH R20		; Reserves R20 and R21 incase used elsewhere
	PUSH R21				
	MOV R20, R18	    ; Counter 2
Loop2b:
	MOV R21, R19		; Counter 1
Loop1a:	
	DEC R21
	BRNE Loop1a         
	DEC R20
	BRNE Loop2b
	POP R21
	POP R20
	RET  

;------------------------------ ADC SUBROUTINES ------------------------------;
InitADC:
	LDI R16, ADCsetup		; Set bits in ADC control register to
	STS ADCSRA, R16			; turn it on & use clk/128 since we'll
							; be looking at 10 bits
	LDI R16, ADMUXsetup		; Set bits in ADC MUX register for
	STS ADMUX, R16			; Vref = 5 volts, left-justify output, use pin A1
	RET

ReadADC:
	LDI R16, ADCsetup | (1<<ADSC)
	STS ADCSRA, R16
ReadADCLoop:
	LDS R16, ADCSRA
	SBRS R16, ADIF
	RJMP ReadADCLoop
	LDI R16, ADCsetup|(1<ADIF)
	STS ADCSRA, R16
	LDS R16, ADCL			; Stores low bytes of ADC in R16
	LDS R17, ADCH			; Stores high bytes of ADC in R17
	STS $200, R16
	STS $201, R17
	RET

; Converts our ADC read into 0, 1, and 2
; MultipliesADC by ten, divides by 5100 decimal
; Maybe it is unnecessary to multiply by 10??
AnalogToThird:
	LDS R16, $200
	LDS R17, $201
	LDI R18, $0A
	MUL R16, R18
	MOV R16, R0
	MOV R19, R1
	MUL R17, R18
	ADD R19, R0
	RCALL DivideBy5100
	STS $202, R18		; Stores possible values 0 to 2 in $202

DivideBy5100:			; Divides by 5100
	CLR R18			
	LDI R20, $13
	LDI R17, $EC				
 DivLoop5100:
    INC R18
	SUB R16, R17
	SBC R19, R20
	BRGE DivLoop5100		
	DEC R18				
	RET

OctaveToASCII:			; Converts to ASCII by adding $33 to it
	PUSH R16			; Easy solution as value is only 0, 1, 2
	PUSH R17
	LDS R16, $202
	LDI R17, $33
	ADD R16, R17
	STS $204, R16		; Stores Octave ASCII value in $204
	POP R17
	POP R16

;------------------------ USART AND PRINT SUBROUTINES ------------------------;
InitUSART:
	LDI R16, (1<<TXEN0)					; enable transmit on UNO TX (pin1)
	STS UCSR0B, R16    
	LDI R16, (1<<UCSZ01)|(1<<UCSZ00)	; 8-bit chars, 1 stop-bit
	STS UCSR0C, R16						; and no parity bit
	LDI R16, $10						; 57600 baud
	STS UBRR0L, R16
	RET

PrintChar:				;serial print a single ASCII char
	LDS R16, UCSR0A		; read the USART control register
	SBRS R16, UDRE0		; Poll the transmit buffer flag to see
	RJMP PrintChar		; if it is a 1 (empty)
	STS UDR0, R21		; if empty, put an ASCII char in the tx buffer
	RET

PrintInfo:
	CPI R23, $48
	BRCC PrintLUT
	MOV R21, R23	  ; Prints note as defined in polling loop
	RCALL PrintChar
	LDS R21, $204	  ; Prints octave value stored in $204
	RCALL PrintChar
	LDI R21, $0A      ; $0A is ASCII Line Feed
	RCALL PrintChar
	LDI R21, $0D      ; $0D is ASCII Carriage Return (returns to start of line)
	RCALL PrintChar
	LDI R24, $00
	LDI R26, $00
	RJMP OctaveCheck2 

PrintLUT:
	SUBI R23, $48
	LDI R27, $08
	LDI R28, $07
	MUL R23, R27
	LDI ZL, LOW(USARTchords<<1)
	LDI ZH, HIGH(USARTchords<<1)
	ADD ZL, R0
LUTLoop:
	LPM R21, Z+
	RCALL PrintChar
	DEC R28
	BREQ Exit
	RJMP LUTloop
Exit:
	LDI R21, $0A      ; $0A is ASCII Line Feed
	RCALL PrintChar
	LDI R21, $0D      ; $0D is ASCII Carriage Return (returns to start of line)
	RCALL PrintChar
	LDI R24, $00
	LDI R26, $00
	RJMP OctaveCheck2 

;----------------------------- DELAY SUBROUTINES -----------------------------;
Delay100Milli:
	LDI R16, $07
	Loop103:
	LDI R17, $FF
	Loop102:
	LDI R18, $FF
	Loop101:
	DEC R18
	BRNE Loop101
	DEC R17
	BRNE Loop102
	Dec R16
	BRNE Loop103
	RET