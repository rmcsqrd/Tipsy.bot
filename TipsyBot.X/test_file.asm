;;;;;;; Lab 4 Rio McMahon Code for ASEN 4519/5519 ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;; Program hierarchy ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; 
;;;;;;; Assembler directives ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Copied from lab4_example.asm and previous lab examples
	
        list  P=PIC18F2553, F=INHX32, C=160, N=0, ST=OFF, MM=OFF, R=DEC, X=OFF
        #include p18f87k22.inc
;		After MPLAB X all configuration bits are set in the code
;		Use mplab help to understand what these directives mean
		CONFIG	FOSC = HS1
		CONFIG	PWRTEN = ON, BOREN = ON, BORV = 1, WDTEN = OFF
		CONFIG	CCP2MX = PORTC, XINST = OFF

;;;;;;; Variables ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        cblock  0x000       
        endc		    

;;;;;;; Macro definitions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; All macro definitions taken from lab4_example.asm
	
MOVLF   macro  literal,dest		; Move literal to file macro
        movlw  literal
        movwf  dest
        endm
;; POINT taken from Reference: Peatman CH 7 LCD
POINT   macro  stringname		; Load a string into table pointer
        MOVLF  high stringname, TBLPTRH	; Used to put values in program memory
        MOVLF  low stringname, TBLPTRL
        endm

DISPLAY macro  register         ; Displays a given register in binary on LCD
        movff  register,BYTE
        call  ByteDisplay
        endm


;;;;;;; Vectors ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

        org  0x0000         ; Reset vector
        nop		    ; No operation, wastes one instruction cycle
        goto  MAINLINE	    ; Send program to Mainline code

        org  0x0008         ; High priority interrupt vector
        goto  $             ; $ returns code to the current program counter

        org  0x0018	    ; Low priority interrupt vector
        goto  $             ; Returns. Only here to show code structure.

;;;;;;; Mainline program ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

MAINLINE


	
	end

