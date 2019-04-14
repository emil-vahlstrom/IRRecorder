
/*
 * waits.s
 *
 * Created: 2016-12-15 23:04:20
 *  Author: Emil
 */ 

 #include <avr/io.h>
 .global wait
 .global wait100
 .global wait_long
 .global no_operation

 wait:
        NOP                     ; NOP 1 cycle
        DEC     R24             ; DEC 1 cycle
        BRNE    wait            ; BRNE 2 cycles
        RET                     ; RET 4 cycles (also true for call)

 wait100:                       ;wait 100us
        LDI     R16, 199        ;199 to compensate for CALL and RET ; LDI 1 cycle
 decrement: 
        NOP
        NOP
        NOP
        NOP
        NOP
        DEC     R16
        BRNE    decrement
        RET

wait_long:
        CALL    wait100     ;this lines incl. CALL takes 100µs
        DEC     R24
        BRNE    wait_long
        RET

no_operation:
        RET