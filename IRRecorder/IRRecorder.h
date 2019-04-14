/*
 * IRRecorder.h
 *
 * Created: 2019-01-27 23:02:12
 *  Author: Emil
 */ 

#ifndef IR_RECORDER_H_
#define IR_RECORDER_H_

extern void wait(char);
extern void wait_long(char);
extern void no_operation(void);
extern void wait100(void);
void init_pins(void);
void wait_busyflag(void);
void send_nibble(char);
void send_instruction(char);
void send_character(char);
void init_LCD(void);
void init_PCINT0(void);
char convert(char);
void init_timer1(void);
void init_USART(void);
void init_tones(void);
void usart_io_method();
void usart_init_message();

#endif /* IR_RECORDER_H_ */