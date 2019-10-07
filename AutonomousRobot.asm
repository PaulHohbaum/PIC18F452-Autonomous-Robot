;;;Assembler directives
  LIST  P=PIC18F452, F=INHX32, C=160, N=0, ST=OFF, MM=OFF, X=ON
  #include  P18F452.INC
  CONFIG  OSC=HS      ;select HS oscillator
  CONFIG  PWRT=ON, BOR=ON, BORV=42
  CONFIG  WDT=OFF, LVP=OFF  ;disable watchdog timer
          ; and low voltage power
;-----------------------------------------------------------------------------------
;;;Constants Section
  constant LEFT_MOTOR  = d'5'    ;Green
  constant RIGHT_MOTOR = d'4'    ;Red

  constant WHEEL_FORWARD_PULSE_WIDTH = d'2500'*d'21'/d'10'  ;2.1*2500 = 2.1 ms delay
  constant WHEEL_BACKWARDS_PULSE_WIDTH = d'2500'*d'71'/d'50' ;1.42*2500 = 1.42 ms delay   ;1.3 for straight

  constant WHEEL_SLOW_FORWARD_PULSE_WIDTH = d'2500'*d'17'/d'10'  ;1.7*2500 = 1.7 ms delay


  constant PULSE_WAIT_TIME = d'2500'*d'20'-WHEEL_FORWARD_PULSE_WIDTH-WHEEL_BACKWARDS_PULSE_WIDTH ;Used to wait out the rest of the 20ms for Forward,backwards
  constant TURN_PULSE_WAIT_TIME = d'2500'*d'20'-WHEEL_FORWARD_PULSE_WIDTH ;Used to wait out the rest of the 20ms for Turning

  constant RIGHT_DIST_ECHO = d'3'  ; Right distance sensor echo pin
  constant FRONT_DIST_ECHO = d'2'  ; Front distance sensor echo pin
  constant RIGHT_DIST_TRIG = d'1'  ; Right distance sensor trigger pin
  constant FRONT_DIST_TRIG = d'0'  ; Front distance sensor trigger pin

  constant INFRARED_RECEIVER = d'4' ;Infrared sensor is on pin RC4

  constant NEGATIVE = d'4'
  constant OVERFLOW = d'3'
  constant ZERO     = d'2'

  constant CLOSE_WALL   = d'7'
  constant CLOSE_WALLR  = d'4'
  constant MAX_DISTANCE = d'8'

  constant NINETY_DEGREE_LEFT  = d'90'
  constant NINETY_DEGREE_RIGHT = d'90'
  constant CORRECTION_DEGREE = d'20'

  constant MAX_FREQUENCY = d'50'
;-----------------------------------------------------------------------------------
;__      __  ______    _____   _______    ____    _____     _____
; \ \    / / |  ____|  / ____| |__   __|  / __ \  |  __ \   / ____|
;  \ \  / /  | |__    | |         | |    | |  | | | |__) | | (___
;   \ \/ /   |  __|   | |         | |    | |  | | |  _  /   \___ \
;    \  /    | |____  | |____     | |    | |__| | | | \ \   ____) |
;     \/     |______|  \_____|    |_|     \____/  |_|  \_\ |_____/
;
;-----------------------------------------------------------------------------------
        org   0x0000                   ; Reset vector.
        nop
        goto  Main

        org   0x0008                   ; High priority interrupt vector.
        goto  $                        ; If interrupt is generated, go into this infinite loop.

        org   0x0018                   ; Low priority interrupt vector.
        goto  $                        ; If interrupt is generated, go into this infinite loop.

;-----------------------------------------------------------------------------------
;;;Variables Section
  cblock  0x000
  front_dist
  right_dist
  IR_sensor
  multiplyValue
  max_count
  middle_count
  max_loops

  max_countD
  middle_countD
  max_loopsD

  endc
;-----------------------------------------------------------------------------------
;  __  __               _____   _____     ____     _____
; |  \/  |     /\      / ____| |  __ \   / __ \   / ____|
; | \  / |    /  \    | |      | |__) | | |  | | | (___
; | |\/| |   / /\ \   | |      |  _  /  | |  | |  \___ \
; | |  | |  / ____ \  | |____  | | \ \  | |__| |  ____) |
; |_|  |_| /_/    \_\  \_____| |_|  \_\  \____/  |_____/
;
;   - MULWL
;   - MOVLF_HIGH
;   - MOVLF_LOW
;   - MOVLF
;-----------------------------------------------------------------------------------
MULWL macro literal;Multiplys a literal with the working register
  MOVLF literal,multiplyValue
  MULWF multiplyValue
  endm
;-----------------------------------------------------------------------------------
; Moves the high byte into a register
MOVLF_HIGH   macro  literal,dest
  movlw  high literal
  movwf  dest
  endm
;-----------------------------------------------------------------------------------
; Moves the high byte into a register
MOVLF_LOW   macro  literal,dest
  movlw  low  literal
  movwf  dest
  endm
;-----------------------------------------------------------------------------------
; Lets the programmer store a literal value in a RAM location directly.
MOVLF   macro  literal,dest
  movlw  literal
  movwf  dest
  endm
;-----------------------------------------------------------------------------------
; __  __              _____   _   _
;|  \/  |     /\     |_   _| | \ | |
;| \  / |    /  \      | |   |  \| |
;| |\/| |   / /\ \     | |   | . ` |
;| |  | |  / ____ \   _| |_  | |\  |
;|_|  |_| /_/    \_\ |_____| |_| \_|
;
;-----------------------------------------------------------------------------------
Main
  rcall INITIAL

; This function makes the robot drive
drive  rcall short_forward  ;robot moves forward
       btfsc PORTC,INFRARED_RECEIVER
       rcall beaconDetected

driveF bra checkFrontDist ;robot checks the front distance sensor
       btfsc PORTC,INFRARED_RECEIVER
       rcall beaconDetected

driveR bra checkRightDist ;checks the right distance sensor
       btfsc PORTC,INFRARED_RECEIVER
       rcall beaconDetected
       bra drive       ;loops


; This function checks the distance to the front sensor
checkFrontDist  rcall  frontDistance ;gets the distance, puts it in front_dist
          movlw  CLOSE_WALL    ;moves the "close to wall" constant into WREG
         cpfslt  front_dist    ;is it close to a wall?
            bra  driveR        ;  no -> go back to driving
      bra  objectF       ;  yes -> theres an object! go to objectF

; This function checks the distance to the front sensor
checkRightDist  rcall rightDistance ;gets the distance, puts it in right_dist
          movlw CLOSE_WALLR    ;moves the "close to wall" constant into WREG
         cpfslt right_dist    ;is it close to a wall?
      bra drive         ;  no -> go back to driving
      bra objectR       ;  yes -> theres an object! go to objectR


; This function checks the front distance sensor


; This function tells the robot what to do when it sees the beacon, right now the robot just stops

; This function corrects the robot's course, if it got here the right distance
; sensor is close to something but the front sensor is not
objectR rcall left_turn_correct
          bra drive

; There is an object in front of the front sensor
objectF rcall  rightDistance     ;Checks the right sensor
  movlw  CLOSE_WALL
  CPFSGT right_dist  ;Checks if there is an object close to the right sensor
  bra    objectFR    ; if there is turn left
  rcall  left_turn_ninety  ; if there isn't turn right
  bra    driveR

; There is an object in front of the front sensor and right sensor, so turn left
objectFR rcall left_turn_ninety
   bra driveR
;-------------------------------------------------------------------------------------------------
;   _____   _    _   ____    _____     ____    _    _   _______   _____   _   _   ______    _____
;  / ____| | |  | | |  _ \  |  __ \   / __ \  | |  | | |__   __| |_   _| | \ | | |  ____|  / ____|
; | (___   | |  | | | |_) | | |__) | | |  | | | |  | |    | |      | |   |  \| | | |__    | (___
;  \___ \  | |  | | |  _ <  |  _  /  | |  | | | |  | |    | |      | |   | . ` | |  __|    \___ \
;  ____) | | |__| | | |_) | | | \ \  | |__| | | |__| |    | |     _| |_  | |\  | | |____   ____) |
; |_____/   \____/  |____/  |_|  \_\  \____/   \____/     |_|    |_____| |_| \_| |______| |_____/
;
;   0) INITIAL
;   1) Functions with return values
;   2) Functions for the wheels
;   3) Delay Subroutines
;-------------------------------------------------------------------------------------------------
INITIAL
        MOVLF B'11000000', SSPSTAT     ;  to use with a 2.5 MHz clock.
        MOVLF B'10000001', T1CON       ; continue setting up T1 to CCP1.
        MOVLF B'00001011', CCP1CON     ; Set up to trigger special event so that PIR1, CCP1IF will be set.
  CLRF PORTB ; Initialize PORTB by
  CLRF PORTC ; clearing portb

  CLRF LATB ; Clear data latches
  CLRF LATC

  MOVLW b'00010000' ;Set RC4 to input for infrared reciever
  MOVWF TRISC

  MOVLW b'11001100' ; Value used to initialize datadirection ;0 Output ;1 Input
  MOVWF TRISB

  ; initialize PORT A registers for leds

  MOVLF   b'01100001',TRISA ;set data direction
  MOVLF   b'10001110',ADCON1  ;set digital/analog functions
  clrf  PORTA     ;initialize L, C, R LEDs to off
  return
;-----------------------------------------------------------------------------------
;; Functions with return values
;   - frontDistance
;   - rightDistance
;-----------------------------------------------------------------------------------
frontDistance     bsf LATB,FRONT_DIST_TRIG  ;Send short trigger pulse
        rcall triggerDelay
        bcf LATB,FRONT_DIST_TRIG
        clrf  front_dist

frontDistanceLoop   btfss PORTB,FRONT_DIST_ECHO ;Count until echo is recieved
        bra   frontDistanceLoop
        movlw d'0'

frontDistanceLoop2  addlw d'1' ; basicaly, count how many times it overflows, then increment the counter
        btfsc STATUS,OVERFLOW
        bra   incFront
        btfsc PORTB,FRONT_DIST_ECHO ;Pulse counted, store value
        bra   frontDistanceLoop2  ;1
        bcf   PORTB,FRONT_DIST_ECHO
        bra   frontDistanceRet

incFront      incf   front_dist
        movlw  MAX_DISTANCE
        cpfseq front_dist
        bra    frontDistanceLoop2
        bra    frontDistanceRet

frontDistanceRet    return
;-------------------------------------------------------------------------------
rightDistance     bsf LATB,RIGHT_DIST_TRIG  ;Send short trigger pulse
        rcall triggerDelay
        bcf LATB,RIGHT_DIST_TRIG
        clrf  right_dist

rightDistanceLoop   btfss PORTB,RIGHT_DIST_ECHO ;Count until echo is recieved
        bra   rightDistanceLoop
        movlw d'0'

rightDistanceLoop2  addlw d'1' ; basicaly, count how many times it overflows, then increment the counter
        btfsc STATUS,OVERFLOW
        bra   incRight
        btfsc PORTB,RIGHT_DIST_ECHO ;Pulse counted, store value
        bra   rightDistanceLoop2  ;1
        bcf   PORTB,RIGHT_DIST_ECHO
        bra   rightDistanceRet

incRight      incf  right_dist
        movlw  MAX_DISTANCE
        cpfseq right_dist
        bra    rightDistanceLoop2
        bra    rightDistanceRet

rightDistanceRet    return
;-----------------------------------------------------------------------------------------------------
beaconDetected btfss PORTC,INFRARED_RECEIVER  ;We want to start the frequency counter as soon as the pulse is low
         nop
         bra displayFrequency

         bra sensorLow      ;If the wave starts low, wait until high and then low again to start counter
         bra sensorHigh     ;If the wave starts high, wait until low then start counter

sensorHigh     btfsc PORTC,INFRARED_RECEIVER
         bra sensorHigh
         bra startCount

sensorLow      btfss PORTC,INFRARED_RECEIVER
         bra sensorLow
         bra sensorHigh

startCount     addlw d'1' ; basicaly, count how many times it overflows, then increment the counter
         btfsc STATUS,OVERFLOW
               bra   IRoverflow
         btfss PORTC,INFRARED_RECEIVER  ;If signal is still on keep counting
         bra   startCount
         bra   IRsensorRet    ;Else return w register contains counter
         MOVWF  IR_sensor   ; I want to move w reg into IR_sensor is this how I do this?
         bra  displayFrequency   ;Turns on leds based on frequcy

;Display frequency Looks at IR_sensor and turns on leds based on the value
displayFrequency
         BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
                 BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
                 BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
                 BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
                 BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
                 BSF  LATA,4      ;turn left LED on
         rcall    fiveSecDelay
         BTG  LATA,4      ;breakpoints
         rcall  fiveSecDelay
         return

IRoverflow     bra IRsensorRet  ;If it overflows the its been too long for an IR signal

IRsensorRet    return   ;Returns to the main program
;-----------------------------------------------------------------------------------
;;Functions for the wheels
;   1) Main wheel functions
; - short_forward
; - right_turn_ninety
; - left_turn_ninety
; - left_turn_correct
;   2) Helper wheel functions
; - leftTurn
; - rightTurn
; - fastForward
; - fastBackward
;-----------------------------------------------------------------------------------
short_forward   MOVLF d'1',max_loops ;moves the roboto a short distance forward
del_out_forward   MOVLF d'50',max_count ;initialize loop counter
del_in_forward    rcall fastForward     ;kill some time

       btfsc PORTC,INFRARED_RECEIVER
       rcall beaconDetected

       decfsz max_count   ;decrement count
       bra    del_in_forward  ;loop if not zero
       decfsz max_loops   ;   else decrement loop number
       bra    del_out_forward  ;loop if not zero
       return     ;   else return
;-----------------------------------------------------------------------------------
;;; Right Turn 90 Degrees Subroutine
right_turn_ninety  MOVLF d'1',max_loops ;turns the robot 90 degrees to the right
del_out_ninetyR   MOVLF NINETY_DEGREE_RIGHT,max_count ;initialize loop counter
del_in_ninetyR    rcall rightTurn     ;kill some time
       decfsz max_count   ;decrement count
       bra    del_in_ninetyR    ;loop if not zero
       decfsz max_loops   ;   else decrement loop number
       bra    del_out_ninetyR  ;loop if not zero
       return     ;   else return
;-----------------------------------------------------------------------------------
;;; Left Turn 90 Degrees Subroutine
left_turn_ninety  MOVLF d'1',max_loops ;turns the robot 90 degrees to the right
del_out_ninetyL   MOVLF NINETY_DEGREE_LEFT,max_count ;initialize loop counter
del_in_ninetyL    rcall leftTurn  ;kill some time

    btfsc PORTC,INFRARED_RECEIVER
    rcall beaconDetected
           decfsz max_count ;decrement count
              bra del_in_ninetyL  ;loop if not zero
           decfsz max_loops ;   else decrement loop number
              bra del_out_ninetyL ;loop if not zero
           return           ;   else return
;-----------------------------------------------------------------------------------
;;; Left Turn Correct Subroutine
left_turn_correct MOVLF d'1',max_loops
del_out_correct   MOVLF CORRECTION_DEGREE,max_count ;initialize loop counter
del_in_correct    rcall leftTurn  ;kill some time
    btfsc PORTC,INFRARED_RECEIVER
    rcall beaconDetected
     decfsz max_count ;decrement count
              bra del_in_correct  ;loop if not zero
           decfsz max_loops ;   else decrement loop number
              bra del_out_correct ;loop if not zero
           return
;-----------------------------------------------------------------------------------
; Helper wheel functions
;-----------------------------------------------------------------------------------
leftTurn
  ;20ms wheel cycle
  MOVLF_HIGH WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1H  ;Set loop to right wheel pulse width
  MOVLF_LOW  WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1L
  BSF LATB,RIGHT_MOTOR  ;Set right wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,RIGHT_MOTOR  ;Set right wheel pin low

  MOVLF_HIGH TURN_PULSE_WAIT_TIME, CCPR1H ;Wait out the rest of the time
  MOVLF_LOW  TURN_PULSE_WAIT_TIME, CCPR1L
  rcall LoopTime    ;wait for remainder of 20ms

  return
;-------------------------------------------------------------------------------
rightTurn
  ;20ms wheel cycle
  MOVLF_HIGH WHEEL_FORWARD_PULSE_WIDTH , CCPR1H ;Set the left wheel pulse width
  MOVLF_LOW  WHEEL_FORWARD_PULSE_WIDTH , CCPR1L
  BSF LATB,LEFT_MOTOR ;Set left wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,LEFT_MOTOR ;Set left wheel pin low

  MOVLF_HIGH TURN_PULSE_WAIT_TIME, CCPR1H ;Wait out the rest of the time
  MOVLF_LOW  TURN_PULSE_WAIT_TIME, CCPR1L
  rcall LoopTime    ;wait for remainder of 20ms

  return
;-------------------------------------------------------------------------------
fastForward
  ;20ms wheel cycle
  MOVLF_HIGH WHEEL_FORWARD_PULSE_WIDTH , CCPR1H  ;Set loop to left wheel pulse width
  MOVLF_LOW  WHEEL_FORWARD_PULSE_WIDTH , CCPR1L
  BSF LATB,LEFT_MOTOR ;Set left wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,LEFT_MOTOR ;Set left wheel pin low

  MOVLF_HIGH WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1H ;Set the right wheel pulse width
  MOVLF_LOW  WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1L
  BSF LATB,RIGHT_MOTOR  ;Set right wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,RIGHT_MOTOR  ;Set right wheel pin low

  MOVLF_HIGH PULSE_WAIT_TIME, CCPR1H ;Wait out the rest of the time
  MOVLF_LOW  PULSE_WAIT_TIME, CCPR1L
  rcall LoopTime    ;wait for remainder of 20ms

  return
;-------------------------------------------------------------------------------
fastBackward
  ;20ms wheel cycle
  MOVLF_HIGH WHEEL_FORWARD_PULSE_WIDTH , CCPR1H  ;Set loop to right wheel pulse width
  MOVLF_LOW  WHEEL_FORWARD_PULSE_WIDTH , CCPR1L
  BSF LATB,RIGHT_MOTOR  ;Set right wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,RIGHT_MOTOR  ;Set right wheel pin low

  MOVLF_HIGH WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1H ;Set the left wheel pulse width
  MOVLF_LOW  WHEEL_BACKWARDS_PULSE_WIDTH , CCPR1L
  BSF LATB,LEFT_MOTOR ;Set left wheel pin high
  rcall LoopTime    ;Wait
  BCF LATB,LEFT_MOTOR ;Set left wheel pin low

  MOVLF_HIGH PULSE_WAIT_TIME, CCPR1H ;Wait out the rest of the time
  MOVLF_LOW  PULSE_WAIT_TIME, CCPR1L
  rcall LoopTime    ;wait for remainder of 20ms

  return
;-----------------------------------------------------------------------------------
;;Delay Subroutines
;   - LoopTime
;   - triggerDelay
;   - fiveSecDelay
;-----------------------------------------------------------------------------------

;;;;;;; LoopTime subroutine ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CCP1 Loop
LoopTime

L46
        ;UNTIL_ PIR1, CCP1IF == 1
        btfss PIR1,CCP1IF
        bra L46
RL46
        bcf    PIR1, CCP1IF            ; When it has, reset it to start over.
        return
;-------------------------------------------------------------------------------
;;; Trigger delay subroutine
triggerDelay MOVLF d'1',max_loops ;initialize number of loops
del_out_trig MOVLF d'5',max_count ;initialize loop counter
del_in_trig  nop      ;kill some time
       decfsz max_count   ;decrement count
       bra    del_in_trig   ;loop if not zero
       decfsz max_loops   ;   else decrement loop number
       bra    del_out_trig  ;loop if not zero
       return     ;   else return
;-------------------------------------------------------------------------------
;;; 5 second delay subroutine
fiveSecDelay MOVLF h'FF',max_loopsD ;initialize number of loops
del_out_five MOVLF h'FF',max_countD ;initialize loop counter
del_in_five  nop      ;kill some time
       decfsz max_countD    ;decrement count
       bra    del_in_five   ;loop if not zero
       decfsz max_loopsD    ;   else decrement loop number
       bra    del_out_five  ;loop if not zero
       return     ;   else return
    end