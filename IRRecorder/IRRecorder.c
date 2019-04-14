/*
* IRRecorder.c
*
* Created: 2016-12-22 23:54:48
* Author : Emil
*/

//Functions needed to control an LCD display
//and a keyboard
//121117
//Jonas F
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//#define F_CPU 1000000UL  // 1 MHz
//#include <util/delay.h>
//#include <limits.h>
#include "IRRecorder.h"

#define     COL1     0b11100000 // pd3
#define     COL2     0b10110000 // pd6
#define     COL3     0b01110000 // pd7

#define     COL1_PORTD_BIT  4
#define     COL2_PORTD_BIT  6
#define     COL3_PORTD_BIT  7
#define     IR_RECV_PWR_PORTD_BIT   1

#define     ROW1_PORTB_BIT 4
#define     ROW2_PORTB_BIT 5
#define     ROW3_PORTB_BIT 6
#define     ROW4_PORTB_BIT 7

#define BUTTON_ROWS 1 << PCINT7 | 1 << PCINT6 | 1 << PCINT5 | 1 << PCINT4

#define PORTC_IRLED 1 << 7

#define PB6_ASTERIX 1 << 6
#define PB4_ONE 1 << 4
#define IR_RECV 1 << PCINT7
#define IR_RECV2 1 << PCINT2 // pb2
//#define PC6 6
//#define IR_RECV 1 << 6

#define SEND_BUTTON PB4_ONE
#define RECORD_BUTTON PB6_ASTERIX
  
#define     RS     0
#define     RW     1
#define     E     7

#if defined(__AVR_ATmega32U4__)
#define IS_ATMEGA_32U4 1
#define CPU_SPEED 16000000
#else
#define IS_ATMEGA_32U4 0
#define CPU_SPEED 0
#endif

#define CHAR_BUFFER_SIZE 70

volatile uint32_t timer1_interrupt_count = 0;
volatile uint64_t actual_time_in_micros = 0;
volatile uint32_t actual_time_in_millis = 0;

volatile uint32_t last_millis = 0;
volatile uint32_t millis_step = 1000;
volatile const uint16_t leet = 1337;
char buffer[CHAR_BUFFER_SIZE] = {0};

volatile uint32_t timer3_num_periods = 0;
volatile uint8_t led_status = 1;

volatile uint8_t current_command_index = 0;

typedef struct timing_struct
{
    int16_t usecs_left;
    struct timing_struct *next;
} timer;

timer* t1;

#define TIMERPTR timer*
#define MALLOCTIMER (timer*) malloc ( sizeof(timer*) )

// RC5 is explained here: https://en.wikipedia.org/wiki/RC-5
#define RC5_START_FIELD_CONTROL 889, 889, 889+889, 889
#define RC5_ADDRESS_TV_LOWERFIELD 889, 889, 889, 889, 889, 889, 889, 889+889, 889
#define RC5_COMMAND_DIGIT_1 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889, 889, 889, 889, 889+889, 889, -1
#define RC5_COMMAND_DIGIT_2 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889, 889, 889+889, 889+889, 889, -1
#define RC5_COMMAND_DIGIT_3 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889, 889, 889+889, 889, 889, 889, 889, -1
#define RC5_COMMAND_DIGIT_4 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889+889, 889+889, 889, 889, 889, -1
#define RC5_COMMAND_DIGIT_5 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889+889, 889+889, 889+889, 889, 889, -1
#define RC5_COMMAND_DIGIT_6 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889+889, 889, 889, 889+889, 889, -1
#define RC5_COMMAND_DIGIT_7 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889+889, 889, 889, 889, 889, 889, 889, -1
#define RC5_COMMAND_DIGIT_8 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889+889, 889+889, 889, 889, 889, 889, 889, -1
#define RC5_COMMAND_DIGIT_9 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889+889, 889+889, 889, 889, 889+889, 889, 889, -1
#define RC5_COMMAND_DIGIT_0 RC5_START_FIELD_CONTROL, RC5_ADDRESS_TV_LOWERFIELD+889, 889, 889, 889, 889, 889, 889, 889, 889, 889, 889, 889, -1

volatile int16_t recorded_command[50] = { -1 };
//889, 889, 1778, 889, 889, 889, 889, 889, 889, 889, 889, 1778, 1778, 889, 889, 889, 889, 889, 889, 889, 889, 1778, 889, -1
//volatile int16_t command_btn_1[50] = { 917, 887, 1832, 892, 942, 860, 937, 865, 914, 888, 941, 1743, 1890, 836, 934, 868, 914, 890, 941, 858, 942, 1746, 941, -1 };

#define MAX_COMMAND_SIZE 50

volatile int16_t all_commands[10][MAX_COMMAND_SIZE] =
{
    {RC5_COMMAND_DIGIT_1}, {RC5_COMMAND_DIGIT_2}, {RC5_COMMAND_DIGIT_3}, {RC5_COMMAND_DIGIT_4}, {RC5_COMMAND_DIGIT_5},
    {RC5_COMMAND_DIGIT_6}, {RC5_COMMAND_DIGIT_7}, {RC5_COMMAND_DIGIT_8}, {RC5_COMMAND_DIGIT_9}, {RC5_COMMAND_DIGIT_0}
};

#define COMMAND_1_INDEX 0
#define COMMAND_2_INDEX 1
#define COMMAND_3_INDEX 2
#define COMMAND_4_INDEX 3
#define COMMAND_5_INDEX 4
#define COMMAND_6_INDEX 5
#define COMMAND_7_INDEX 6
#define COMMAND_8_INDEX 7
#define COMMAND_9_INDEX 8
#define COMMAND_0_INDEX 9

volatile int16_t test_command_02[50] = { 917, 887, 1832, -1};
volatile int16_t test_command_03[50] = { -1};
volatile int16_t test_time = 0;
volatile uint8_t test_index = 0;

volatile uint8_t portc_irled_status = 0;
volatile timer timer_local;
#define PORTC_IRLED_ON PORTC_IRLED
#define PORTC_IRLED_OFF 0

#define BUTTON_BLINK_INTERVAL_1 1000
#define BUTTON_BLINK_INTERVAL_2 500
#define BUTTON_BLINK_INTERVAL_3 250
#define BUTTON_BLINK_INTERVAL_4 125

#define STATUS_LED_PORTD 0x01

uint32_t status_led_last_transition = 0;

volatile uint8_t test;

uint8_t charcodes[13] = { '?', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', '0', '#' };
    
volatile uint8_t start_rec = 0;
volatile uint32_t usecs_passed = 0;
volatile uint8_t iindex = 0;
volatile uint8_t last_signal = 0;
volatile uint8_t current_signal = 0;

volatile unsigned char i = 1;
volatile unsigned char j = 1;

volatile uint32_t last_millis_interrupt = 0;
volatile uint32_t interrupt_delay_ms = 500;
volatile uint8_t send = 0;
volatile uint8_t in_record_mode = 0;
volatile uint8_t in_assign_mode = 0;

volatile uint8_t duty_cycle;
volatile uint16_t frequency;
volatile uint16_t clock_cycles_per_period;
volatile int8_t usecs_per_period;

void init_pins(void)
{
    DDRD = 0xFF;    //PD4-PD7 outputs
    PORTD = 0xFE;   //PD1-PD7 on by default, PD0 off(connected to status led)
    DDRF = 0x0F;    //PF4-PF7 inputs, PF0-PF1 outputs
    PORTF = 0x00;
    DDRC = 0b10000000;    //PC6 input and PC7 output
    DDRB = 0b00001011;    //PB2, PB4-PB7 inputs
    //PORTB = 0xF0;    //connect pullup resistors
    PORTB = 0x00; // external 10k pullup resistors
    //PORTC = 0x00;

    PORTC = 0b00000000;
}


void init_USART(void)
{
    /** Pinout: (grön) RTS, RX(yellow) - INPUT, TX(orange) - OUTPUT, 5V, CTS, GND (svart)*/
    // To know the value for UBRR1L, follow this formula:
    // fOsc / (16BAUD) - 1
    // example 1: (16.000.000 / 16 * 9600 ) - 1 = 103
    // example 2: (16.000.000 / 16 * 19200 ) - 1 = 51
    UBRR1L = 51; //baud 19200
    //UCSR1B, USART Control and Status Register B
    UCSR1B = 0b00011000; //activate the USART, sender is bit 3, receiver is bit 4
}

void USART_send_char(char c)
{
    //while UDREI1 (bit 5) is 0 the sender is busy
    //USCR1A = USART Control and Status Register 1
    //more in the study book at p.215
    while (!(UCSR1A & 0b00100000));
    //UDR1, USART I/O Data Register 1
    //send 0x48 (H) as in Hello world
    UDR1 = c;
}

void USART_write(char *c)
{
    while (*c != '\0')
    {
        USART_send_char(*c);
        c++;
    }
}

void USART_writeline(char *c)
{
    USART_write(c);
    USART_send_char('\r');
    USART_send_char('\n');
}

void init_timer1()
{
    asm("CLI"); //Global Interrupt Disable
    /*
    * Timer/Counter1 Control Register A – TCCR1A 
    * Bit 7 6 5 4 3 2 1 0
    * COM1A1 COM1A0 COM1B1 COM1B0 COM1C1 COM1C0 WGM11 WGM10 
    *
    * Timer/Counter1 Control Register B – TCCR1B
    * Bit 7 6 5 4 3 2 1 0
    * ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
    */
    /*
    * CSn2 CSn1 CSn0 Description
    * 0 0 0 No clock source (Timer/Counter stopped)
    * 0 0 1 clkI/O/(No prescaling)
    * 0 1 0 clkI/O/8 (From prescaler)
    * 0 1 1 clkI/O/64 (From prescaler)
    * 1 0 0 clkI/O/256 (From prescaler)
    * 1 0 1 clkI/O/1024 (From prescaler)
    */
    // 16MHz / 8 = 2MHz. 8/16MHz=0,5µs period time.
    // Setting bit 0 to 0 and bit 1 to 1 is equal to dividing frequency by 8
    // bit 3 is WGM12. Setting it to one whilst leaving WGM10 and WGM11 to 0
    // is equal to setting mod4. In mode 4 the counter resets when the same as OCR1B
    //TCCR1B = 0b00001010;
    // match every 50µs (20kHz)
    //OCR1A = 100; 

    // 16MHz / 64 = 0.25MHz. 64/16MHz=4µs period time. 
    TCCR1B = 0b00001011;
    // want interrupt every 100ms
    // (100*10^-3) / 4*10^-6 = 25000
    // 100ms / 4µs = 25000
    OCR1A = 25000;
    OCR1B = 25000;

    // TCNT1 (Timer/Counter1 - Counter Register) is the counter, when it matches TCCR1B
    // it will set Timer/Counter1, Output Compare B Match Flag, OCF1B
    // if interrupts are enabled, an interrupt will happen. 

    /*
    * TIMSK1 - Timer/Counter1 Interrupt Mask Register
    *     Bit|7     |6      |5      |4      |3      |2      |1      |0
    *        |-     |–      |ICIE1  |–      |OCIE1C |OCIE1B |OCIE1A |TOIE1
    * Matches|-     |-      |-      |-      |OCR1C  |OCR1B  |OCR1A  |-
    * OCIE1A = Timer/Counter, Output Compare A Match Interrupt Enable
    */
    
    TIMSK1 = 0b00000110;

    //14.10.19Timer/Counter1 Interrupt Flag Register – TIFR1
    //Name Bit 7 Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1 Bit 0 Page
    //TIFR1 - - ICF1 - OCF1C OCF1B OCF1A TOV1

    //OCF1B is set to 1 when a match occurs and is automatically cleared in the ISR.

    asm("SEI"); //Global Interrupt Enable
}

ISR(TIMER1_COMPA_vect)
{
    //timer1_interrupt_count++;
    //actual_time_in_micros += 50;
    actual_time_in_micros += 100000;
    actual_time_in_millis += 100;
}

ISR(TIMER1_COMPB_vect)
{
    if (1)
    {
        i=1;
        
        if (j == 4)
        j=1;
        
        switch (j)
        {
            case 1:
            {
                PORTD = (PORTD & 0x0F) | 1 << COL1_PORTD_BIT; //activate COL1
                break;
            }
            case 2:
            {
                PORTD = (PORTD & 0x0F) | 1 << COL2_PORTD_BIT; //activate COL2
                break;
            }
            case 3:
            {
                PORTD = (PORTD & 0x0F) | 1 << COL3_PORTD_BIT; //activate COL3;
                break;
            }
        }
        j++;
    }
    i++;
}    

void init_timer3()
{
    asm("CLI");
    TCCR3B = 0b00001001;
    //16MHz/1 = 16MHz
    //period time = 1/16MHz = 0.0625µsec
    //Desired frequency is 38kHz
    //(1/38000)/(1/16E6) = 421 (easier way to calculate: 16MHz / 38kHz = 421)
    //response time it at least 5 ticks according to manual, so set OCR3A to 421-5~=416
    OCR3A = 416;
    TIMSK3 = 0b00000010;
    asm("SEI");
}

ISR(TIMER3_COMPA_vect)
{
    timer3_num_periods++;

    //if (led_status == 1)
    //{
        //PORTC ^= PORTC_IRLED;
    //}
    
    //PORTC |= PORTC_IRLED;
    //wait(32); 
    //PORTC &= ~PORTC_IRLED;
}

uint64_t get_micros()
{
    return actual_time_in_micros;
}

uint32_t get_millis()
{
    return get_micros() / 1000;
}

void init_external_interrupts()
{
    //Global Interrupt Disable
    asm("CLI");
    
    //turn off USB-interrupts
    USBCON = 0x00;
    
    // PCMSK0 - Pin Change Mask Register 0
    // Bit  |7      |6      |5      |4      |3      |2      |1      |0
    //      |PCINT7 |PCINT6 |PCINT5 |PCINT4 |PCINT3 |PCINT2 |PCINT1 |PCINT0
    // Enable interrupt for PCINT4-PCINT7
    PCMSK0 = BUTTON_ROWS;
    
    // Pin Change Interrupt - only bit 1 is defined on atmega 32u4
    // Enable interrupt on PCINT0-7
    PCICR = 0b00000001;
    
    //Global Interrupt Enable
    asm("SEI");
}

void init_PCINT0(void)
{
    PCICR = 0b00000001;    //activate PCINT
    PCMSK0 = BUTTON_ROWS;    //activate for PB4-PB7
    //PCMSK0 = 0b10000000;    //activate for PB7
    asm("SEI");        //set global interrupt bit
}

void wait_ms(uint16_t millis)
{
    for (uint16_t i = 0; i < millis; i++)
    {
        wait_long(250);
        wait_long(250);
        wait_long(250);
        wait_long(250);
    }    
}

void writeSecondsPassed()
{
    if (get_millis() > (last_millis + millis_step))
    {
        last_millis = get_millis();
        //snprintf(buffer, 50, "Milliseconds passed: %lu", last_millis);
        //USART_writeline(buffer);
        snprintf(buffer, CHAR_BUFFER_SIZE, "Seconds passed: %lu - approx frequency for timer3: %u", (last_millis / 1000), timer3_num_periods);
        USART_writeline(buffer);
        timer3_num_periods = 0;
    }
}

void init_ir()
{
    duty_cycle = 50;
    frequency = 38000;
    usecs_per_period = 1000000 / frequency;
    clock_cycles_per_period = CPU_SPEED / frequency;
}


void set_current_command_index(char charcode)
{
    switch (charcode)
    {
        case '0': current_command_index = COMMAND_0_INDEX; break;
        case '1': current_command_index = COMMAND_1_INDEX; break;
        case '2': current_command_index = COMMAND_2_INDEX; break;
        case '3': current_command_index = COMMAND_3_INDEX; break;
        case '4': current_command_index = COMMAND_4_INDEX; break;
        case '5': current_command_index = COMMAND_5_INDEX; break;
        case '6': current_command_index = COMMAND_6_INDEX; break;
        case '7': current_command_index = COMMAND_7_INDEX; break;
        case '8': current_command_index = COMMAND_8_INDEX; break;
        case '9': current_command_index = COMMAND_9_INDEX; break;
    }
}

char get_current_command_charcode()
{
    char charcode = '?';
    switch (current_command_index)
    {
        case COMMAND_0_INDEX: charcode = '0'; break;
        case COMMAND_1_INDEX: charcode = '1'; break;
        case COMMAND_2_INDEX: charcode = '2'; break;
        case COMMAND_3_INDEX: charcode = '3'; break;
        case COMMAND_4_INDEX: charcode = '4'; break;
        case COMMAND_5_INDEX: charcode = '5'; break;
        case COMMAND_6_INDEX: charcode = '6'; break;
        case COMMAND_7_INDEX: charcode = '7'; break;
        case COMMAND_8_INDEX: charcode = '8'; break;
        case COMMAND_9_INDEX: charcode = '9'; break;
    }
    return charcode;
}

void send_ir()
{
    unsigned int modified_cycles = clock_cycles_per_period - (clock_cycles_per_period % 4);
    unsigned char num_periods_of_4_cycles = modified_cycles / 4;
    unsigned char time_on = (float)duty_cycle / 100 * num_periods_of_4_cycles;
    unsigned char time_off = num_periods_of_4_cycles - time_on;
    time_on -= 2;
    time_off -= 12;
    
    portc_irled_status = PORTC_IRLED_ON;
    //timer_local.usecs_left = t1->usecs_left;
    timer_local.usecs_left = 917;
    timer_local.next = NULL;
    
    test_index = 0;
    test_time = all_commands[current_command_index][test_index];
    
    snprintf(buffer, CHAR_BUFFER_SIZE, "Sending IR-code for: %c", get_current_command_charcode());
    USART_writeline(buffer);

    while (1)
    {
        PORTC |= portc_irled_status; // setting this seems to take 2 cycles
        wait(time_on); //time to call and ret is 8 cycles
        PORTC &= ~PORTC_IRLED; // setting this seems to take 2 cycles
        wait(time_off); //time to call and ret is 8 cycles
        //looping probably takes 2 cycles
        
        test_time -= 26;
        if (test_time <= 0)
        {
            test_index++;
            if (all_commands[current_command_index][test_index] == -1)
            {
                break;
            }
            else
            {
                test_time = all_commands[current_command_index][test_index];
                if (portc_irled_status & PORTC_IRLED_ON)
                {
                    portc_irled_status = PORTC_IRLED_OFF;
                }
                else
                {
                    portc_irled_status = PORTC_IRLED_ON;
                }
            }
        }
        
        // how long does this take for every case, how to compensate?
        //timer_local->usecs_left -= 26;
        //if (timer_local->usecs_left <= 0)
        //{
            //if (timer_local->next == NULL)
            //{
                //send = 0;
            //}
            //else
            //{
                //timer_local = timer_local->next;
                //if (portc_irled_status & PORTC_IRLED_ON)
                //{
                    //portc_irled_status = PORTC_IRLED_OFF;
                //}
                //else
                //{
                    //portc_irled_status = PORTC_IRLED_ON;
                //}
            //}
        //}
    }
    
}

uint8_t recv_ir()
{
    // read input
    asm("NOP");
    asm("NOP");
    uint8_t key = PINB;
    
    // if signal is low, which means start
    if (!(key & IR_RECV2))
    {
        last_signal = 0;
        current_signal = 0;
        while (1)
        {
            if (usecs_passed > 50000)
            {
                in_record_mode = 0;
                PCMSK0 = 0b00000000;
                wait(38);
                
                char indexmsg[30] = {0};
                                
                snprintf(indexmsg, 30, "index: %u", iindex);
                USART_writeline(indexmsg);
                
                recorded_command[iindex] = -1;
                
                iindex = 0;
                
                while (1)
                {
                    uint16_t val = recorded_command[iindex];
                    if (val == -1)
                    {
                        USART_writeline("val is -1");
                        iindex = 0;
                        break;
                    }
                    char buffed[30] = {0};
                    
                    snprintf(buffed, 30, "usecs passed: %u, index: %u", val, iindex);
                    USART_writeline(buffed);
                    iindex++;
                }
                
                iindex = 0;
                usecs_passed = 0;
                
                return 1;
            }
            
            //wait_long(1);
            
            // in parameter every 1 counts as 4 cycles. 16 cycles equals one usec
            // waiting 50 usec is 50*16 = 800. divide by four cycles: 800 / 4 = 200
            // compensate for call and ret, which is 8 cycles, i.e corresponds to 2 = about 200 - 2 = 198. 
            // In reality the delay is longer because code before and after also takes time to execute
            // if waiting to short intervals, the precision will be lower because of overhead in the code.
            wait(198);
            //wait100();
            usecs_passed += 50;
            last_signal = current_signal;
        }
    }
    
    return 0;
}



char decode_input(uint8_t col_data, uint8_t row_data)
{
    uint8_t col_no = 0;
    uint8_t row_no = 0;
    uint8_t index_for_charcode = 0;
    uint8_t col_length = 3;
    
    if (col_data & 1 << COL1_PORTD_BIT)
    {
        col_no = 1;
    }
    else if (col_data & 1 << COL2_PORTD_BIT)
    {
        col_no = 2;
    }
    else if (col_data & 1 << COL3_PORTD_BIT)
    {
        col_no = 3;
    }
    
    if (row_data & 1 << ROW1_PORTB_BIT)
    {
        row_no = 1;
    }
    else if (row_data & 1 << ROW2_PORTB_BIT)
    {
        row_no = 2;
    }
    else if (row_data & 1 << ROW3_PORTB_BIT)
    {
        row_no = 3;
    }
    else if (row_data & 1 << ROW4_PORTB_BIT)
    {
        row_no = 4;
    }
    
    index_for_charcode = (col_no + ((row_no - 1) * col_length));
    
    return charcodes[index_for_charcode];
}

ISR(PCINT0_vect)
{
    //to avoid bouncing button, make sure a certain time passes before executing code
    if (get_millis() > last_millis_interrupt + interrupt_delay_ms)
    {
        if (in_record_mode == 0 || in_assign_mode == 1)
        {
            // read input
            asm("NOP");
            asm("NOP");
            uint8_t col_data = PIND;
            uint8_t row_data = PINB;
            char character = decode_input(col_data, row_data);
            
            last_millis_interrupt = get_millis();
            
            // if between the ASCII-values for '0'-'9'
            if (character >= 48 && character <= 57)
            {
                set_current_command_index(character);
                
                if (in_assign_mode == 1)
                {
                    for (uint8_t i = 0; i < MAX_COMMAND_SIZE; i++)
                    {
                        all_commands[current_command_index][i] = recorded_command[i];
                    }
                    in_assign_mode = 0;
                    
                    snprintf(buffer, CHAR_BUFFER_SIZE, "Assigned new command to button: %c", get_current_command_charcode());
                    USART_writeline(buffer);
                }
                else
                {
                    send = 1;
                }
            }

            else if (character == '*')
            {
                USART_writeline("RECORD BUTTON pressed ");
                in_record_mode = 1;
                // activate interrupt for PB2/PCINT2
                PCMSK0 = 0b00000100;
            }
        }
        else if (in_record_mode == 1)
        {
            if (usecs_passed != 0)
            {
                recorded_command[iindex] = usecs_passed;
                usecs_passed = 0;
                iindex++;
            }
        }
    }        
    
    //SREG &= 0b01111111;
    
    //wait to avoid a bouncing button
    //wait_ms(250);
    
    
    //SREG = SREG_temp;
}

void update_led()
{
    int16_t interval = 0;
    if (in_record_mode == 1)
    {
        interval = BUTTON_BLINK_INTERVAL_2;
    }
    else if (in_assign_mode)
    {
        interval = BUTTON_BLINK_INTERVAL_4;
    }
    else
    {
        // status led off
        PORTD &= ~STATUS_LED_PORTD;
        return;
    }
    
    if (status_led_last_transition + interval < get_millis())
    {
        //toggle status led
        PORTD ^= STATUS_LED_PORTD;
        status_led_last_transition = get_millis();
    }
}

int main(void)
{
    SPL = 0xFF;
    SPH = 0x0A;
    USBCON = 0x00;
    init_pins();

    init_USART();
    init_external_interrupts();
    init_PCINT0();
    
    init_timer1();
    //init_timer3();
    
    init_ir();

    PORTB |= 0x04;
    
    
    usart_init_message();
    
    while(1)
    {
        if (send)
        {
            send_ir();
            //_delay_ms(10);
            send = 0;    
        }
        
        if (in_record_mode == 1)
        {
            if (recv_ir() == 1)
            {
                in_record_mode = 0;
                in_assign_mode = 1;
                
                USART_writeline("Assign command by pressing a button");
                // restore interrupt
                PCMSK0 = BUTTON_ROWS; 
            }  
        }
        
        update_led();
        //writeSecondsPassed();

        //usart_io_method();
    }
    
}

void usart_init_message()
{
    USART_writeline("Init complete.");

    snprintf(buffer, CHAR_BUFFER_SIZE, "hello %d", leet);
    USART_writeline(buffer);
    
    if (IS_ATMEGA_32U4)
    {
        USART_writeline("Detected an MCU with Atmega32u4");
        snprintf(buffer, CHAR_BUFFER_SIZE, "Running at clock speed: %li", CPU_SPEED);
        USART_writeline(buffer);
    }
    else
    {
        USART_writeline("Not sure what MCU you are using, there might be unforeseen consequences");
    } 
}

void usart_io_method()
{
    char received;
    // receive from computer | UCSR1A, USART Control and Status Register A
    if (UCSR1A & 0b10000000)  //if a byte have been received, RXC1, bit 7 in USCR1A is set
    {
        USART_writeline("Byte has been received.");
        //UDR1, USART I/O Data Register 1
        received = UDR1; //works well with Optimization: O0, O2 & O3
        
        //play a tone if not default-case (works best with Optimization: O0)
        switch(received)
        {
            case 'c':
            {
                USART_writeline("Playing tone C!");
                PORTB |= 0x04; // LED on
                //OCR3A = 477; //262Hz, C-tone
                break;
            }
            case 'd':
            {
                USART_writeline("Playing tone D!");
                PORTB |= 0x04; // LED on
                //OCR3A = 425; //294Hz, D-tone
                break;
            }
            default:
            {
                PORTB &= 0xFB; // LED off
                //OCR3A = 0; //No tone
                break;
            }
        }
        
        //send_character(received); //send character to screen
    }
}