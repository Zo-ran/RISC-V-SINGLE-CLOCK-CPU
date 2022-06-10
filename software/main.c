#include "sys.h"
#include "utils.h"
#include "func.h"
#include "arithmetic.h"

#define UNKNOWN 0
#define HELLO 1
#define TIME 2
#define FIB 3
#define CLEAR 4
#define CALC 5
#define OVERFLOW 6
#define LIGHT 7

int main();

char mh[] = "Hello World!\n";

void entry() {
    asm("lui sp, 0x00120");
    asm("addi sp, sp, -4");
    main();
}

size_t decoder(char *instr) {
    if (my_strcmp(instr, "hello") == 0)
        return HELLO;
    if (my_strcmp(instr, "time") == 0)
        return TIME;
    if (my_strcmp(instr, "clear") == 0)
        return CLEAR;
    char t = instr[4];
    instr[4] = 0;
    if (my_strcmp(instr, "fib ") == 0) {
        instr[4] = t;
        return FIB;
    }
    if (my_strcmp(instr, "LED ") == 0) {
        instr[4] = t;
        return LIGHT;
    }
    instr[4] = t;
    if (check(instr))
        return CALC;
    if (instr[98] != 0)
        return OVERFLOW;
    return UNKNOWN;
}

int main() {
    vga_init();
    char instr[100];
    my_memset(instr, 100, 0);
    putstr(mh);
    while (1) {
        my_memset(instr, 100, 0);
        handle_kbd(instr);
        switch(decoder(instr)) {
            case HELLO: {
                putstr("Hello World!\n");
                break;
            } case TIME: {
                size_t clk = *(size_t *)(0x500000);
                char hour[5];
                char min[5];
                char sec[5];
                my_memset(hour, 5, 0);
                my_memset(min, 5, 0);
                my_memset(sec, 10, 0);
                my_itoa(__udivsi3(clk, 3600), hour, 10);
                my_itoa(__udivsi3(__umodsi3(clk, 3600), 60), min, 10);
                my_itoa(__umodsi3(clk, 60), sec, 10);
                handle_time(hour);
                handle_time(min);
                handle_time(sec);
                putstr(hour);
                putch(':');
                putstr(min);
                putch(':');
                putstr(sec);
                putch('\n');
                break;
            } case FIB: {
                int num = my_atoi(instr + 4);
                unsigned int res = fibonacci(num);
                char res_char[10];
                my_memset(res_char, 10, 0);
                my_itoa(res, res_char, 10);
                putstr(res_char);
                putch('\n');
                break;
            } case CLEAR: {
                vga_init();
                break;
            } case CALC: {
                char res_char[10];
                my_memset(res_char, 10, 0);
                eval(instr, res_char);
                putstr(res_char);
                putch('\n');
                break;
            } case OVERFLOW: {
                putstr("KeyBuffer Overflow!\n");
                break;
            } case LIGHT: {
                int num = my_atoi(instr + 4);
                *(char *)(0x600000 + num) = 0;
                break;
            } case UNKNOWN: {
                putstr("Unknown Command!\n");
                break;
            }
        }
    }
}