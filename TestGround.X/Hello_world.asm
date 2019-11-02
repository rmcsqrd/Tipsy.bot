; Hello World program 

    
;;;;;;; CONFIG ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    list  P=PIC18F2553, F=INHX32, C=160, N=0, ST=OFF, MM=OFF, R=DEC, X=OFF
    #include p18f2553.inc
; Not sure if I need these configs for my dumb chip
	    CONFIG	FOSC = HS
	    
;	    CONFIG	PWRTEN = ON, BOREN = ON, BORV = 1, WDTEN = OFF
;	    CONFIG	CCP2MX = PORTC, XINST = OFF

    
;;;;;;; VARIABLES ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	cblock  0x000
        endc		    
	
        org  0x0000         ; Reset vector
        nop		    ; No operation, wastes one instruction cycle
        goto  MAINLINE	    ; Send program to Mainline code

        org  0x0008         ; High priority interrupt vector
        goto  $             ; $ returns code to the current program counter

        org  0x0018	    ; Low priority interrupt vector
        goto  $   

	
;;;;;;; MACROS ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
MOVLF   macro  literal,dest		; Move literal to file macro
        movlw  literal
        movwf  dest
        endm
	
	
;;;;;;; CODE ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

MAINLINE
	rcall	Initial

Loop
	BTG 	LATC, 0
	BTG 	LATC, 1
	rcall	Wait1s
	bra	Loop
	
	
Initial
	clrf	TRISA
	clrf	TRISB
	clrf	TRISC
	return
	
TimerLoop
	btfss	INTCON, TMR0IF, 0	; check the TMR0IF flag on INTCON within
					; access memory. If flag is not raised, 
					; check indefinitely until it is
	bra	TimerLoop
	bcf	T0CON, 7, 0		; turn timer off by setting bit 7 = 0
	bcf	INTCON, TMR0IF, 0	; turn off timer flag for next time
	return				; return back to main loop
	
	
;oneSec_num equ	65536-62500
oneSec_num equ	65536-62500
Wait1s
	movlw	B'00000011'
;	movlw	B'00000111'		; Timer0 settings (x64 prescaler)
					; 1s/250ns = 4e6cycles/65536 = 61 prescale
					; round up to 64. 4e6/64 = 62500 cycles
	movwf	T0CON,0
	MOVLF	high oneSec_num, TMR0H	; Write high and low byte to TMR0
	MOVLF	low oneSec_num, TMR0L
	bsf	T0CON,7,0		; start the timer
	rcall	TimerLoop
	return
	
	end

