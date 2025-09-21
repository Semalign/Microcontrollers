#include "LPC214x.h"
#include <string.h>

// --- PWM0 Registers for PWM channel 3 ---
#define PWM0MR0   (*(volatile unsigned long *)0xE0010040)
#define PWM0MR3   (*(volatile unsigned long *)0xE0010050)
#define PWM0PCR   (*(volatile unsigned long *)0xE001004C)
#define PWM0MCR   (*(volatile unsigned long *)0xE0010014)
#define PWM0TCR   (*(volatile unsigned long *)0xE0010004)
#define PWM0LER   (*(volatile unsigned long *)0xE001005C)
#define PWM0PR    (*(volatile unsigned long *)0xE001000C)

// --- Constants ---
#define PCLK 60000000UL
#define UART_BAUD 9600
#define PWM_FREQ 1000          // 1kHz PWM frequency
#define PWM_PERIOD (PCLK / PWM_FREQ)

// --- LCD Pins ---
#define LCD_RS (1 << 16)
#define LCD_EN (1 << 17)
#define LCD_D4 (1 << 18)
#define LCD_D5 (1 << 19)
#define LCD_D6 (1 << 20)
#define LCD_D7 (1 << 21)
#define LCD_DATA (LCD_D4 | LCD_D5 | LCD_D6 | LCD_D7)  // DEFINITIONS THAT ARE NOT INCLUDED IN ORIGINAL HEADER FILE 

// --- Function Prototypes ---
void delay_ms(unsigned int ms);
void uart0_init(void);
void uart0_send_char(char ch);
void uart0_send_string(const char* str);
char uart0_receive_char(void);

void lcd_init(void);
void lcd_cmd(unsigned char cmd);
void lcd_data(unsigned char data);
void lcd_print(const char *str);
void lcd_clear(void);

void PWM_init(void);
void pwm_set_duty(unsigned char duty);  // FUNCTION DECLARATIONS

// --- Main Function ---
int main(void) {
    VPBDIV = 0x01;          // Set PCLK = CCLK = 60MHz

    uart0_init();
    PWM_init();
    lcd_init();

    lcd_clear();
    lcd_print("System Ready");
    uart0_send_string("\r\nSpeed Control System Ready\r\n");
    uart0_send_string("Select Zone:\r\nS - School\r\nC - City\r\nH - Highway\r\n");

    while (1) {
        char input = uart0_receive_char();
        uart0_send_char(input);  // Echo input
        unsigned char duty = 0;

        switch (input) {
            case 'S':
            case 's':
                duty = 30;
                lcd_clear();
                lcd_print("School Zone");
                uart0_send_string("\r\nZone: School (30%)\r\n");
                break;

            case 'C':
            case 'c':
                duty = 60;
                lcd_clear();
                lcd_print("City Zone");
                uart0_send_string("\r\nZone: City (60%)\r\n");
                break;

            case 'H':
            case 'h':
                duty = 90;
                lcd_clear();
                lcd_print("Highway Zone");
                uart0_send_string("\r\nZone: Highway (90%)\r\n");
                break;

            default:
                duty = 0;
                lcd_clear();
                lcd_print("Invalid Input");
                uart0_send_string("\r\nInvalid Zone Input\r\n");
                break;
        }

        pwm_set_duty(duty);
    }
}                                                                       // MAIN FUNCTION WHERE WE CALL ALL THE IMPORTANT FUNCTIONS

// --- UART Functions ---
void uart0_init(void) {
    PINSEL0 |= 0x00000005;   // P0.0 = TXD0, P0.1 = RXD0
    U0LCR = 0x83;
    U0DLL = 78;              // For 9600 baud with PCLK = 60MHz
    U0DLM = 0x00;
    U0LCR = 0x03;            // 8-bit, 1 stop bit, no parity
}

void uart0_send_char(char ch) {
    while (!(U0LSR & 0x20));
    U0THR = ch;
}

void uart0_send_string(const char* str) {
    while (*str)
        uart0_send_char(*str++);
}

char uart0_receive_char(void) {
    while (!(U0LSR & 0x01));
    return U0RBR;
}
                                                          // UART FUNCTIONS 
// --- LCD Functions ---
void lcd_init(void) {
    IODIR1 |= LCD_RS | LCD_EN | LCD_DATA;
    delay_ms(20);
    lcd_cmd(0x33); lcd_cmd(0x32);  // 4-bit init
    lcd_cmd(0x28); lcd_cmd(0x0C);  // 2 line, display ON
    lcd_cmd(0x06); lcd_clear();
}

void lcd_cmd(unsigned char cmd) {
    IOCLR1 = LCD_RS | LCD_DATA;
    IOSET1 = ((cmd >> 4) << 18);
    IOSET1 |= LCD_EN; delay_ms(2); IOCLR1 = LCD_EN;

    IOCLR1 = LCD_DATA;
    IOSET1 = ((cmd & 0x0F) << 18);
    IOSET1 |= LCD_EN; delay_ms(2); IOCLR1 = LCD_EN;
}

void lcd_data(unsigned char data) {
    IOSET1 = LCD_RS;
    IOCLR1 = LCD_DATA;

    IOSET1 = ((data >> 4) << 18);
    IOSET1 |= LCD_EN; delay_ms(2); IOCLR1 = LCD_EN;

    IOCLR1 = LCD_DATA;
    IOSET1 = ((data & 0x0F) << 18);
    IOSET1 |= LCD_EN; delay_ms(2); IOCLR1 = LCD_EN;
}

void lcd_print(const char *str) {
    while (*str) lcd_data(*str++);
}

void lcd_clear(void) {
    lcd_cmd(0x01);
    delay_ms(5);
}
                                // LCD FUNCTIONS 
// --- PWM Initialization and Control ---
void PWM_init(void) {
    // Set P0.8 to PWM1 output (PWM channel 3)
    PINSEL0 &= ~(3 << 16);  // Clear bits 17:16
    PINSEL0 |=  (1 << 16);  // Set bits for PWM1 (P0.8)

    // PWM Prescaler: divide PCLK by (PWM0PR+1)
    PWM0PR = 59;  // 60MHz / (59+1) = 1 MHz timer frequency (1us resolution)

    // Set PWM period for 1 kHz PWM frequency:
    PWM0MR0 = 1000;   // Period = 1000 us = 1 ms

    // Initial duty cycle 0
    PWM0MR3 = 0;

    // Reset on PWM0MR0 match
    PWM0MCR = (1 << 1);

    // Enable PWM output for channel 3 (bit 9)
    PWM0PCR = (1 << 9);

    // Latch PWM0MR0 and PWM0MR3
    PWM0LER = (1 << 0) | (1 << 3);

    // Enable counter and PWM mode
    PWM0TCR = (1 << 0) | (1 << 3);
}

void pwm_set_duty(unsigned char duty_percent) {
    if(duty_percent > 100) duty_percent = 100;  // limit to 100%
    unsigned int duty_value = (PWM0MR0 * duty_percent) / 100;
    PWM0MR3 = duty_value;
    PWM0LER |= (1 << 3);  // Latch new MR3 value                        // PWM FUNCTION TO CONTROL MOTOR SPEED , PULSE WIDTH MODULATED 
}

// --- Delay ---
void delay_ms(unsigned int ms) {
    unsigned int i, j;
    for (i = 0; i < ms; i++)
        for (j = 0; j < 6000; j++);
}
                                                            // THE LAST FUNCTION HERE IS THE DELAY FUNCTION



