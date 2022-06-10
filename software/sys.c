#include "sys.h"
#include "arithmetic.h"

// VGA
char *vga_start = (char *)VGA_START;
unsigned int *vga_reg = (unsigned int *)VGA_REG;
size_t vga_line = 0;
size_t vga_ch = 0;
size_t vga_scroll = 0;
size_t judge = 0;

// KBD
char *kbd_start = (char *)KBD_START;

void vga_init() {
    vga_line = 0;
    vga_ch = 0;
    *vga_reg = 0;
    vga_scroll = 0;
    for (size_t i = 0; i < VGA_MAXLINE; ++i)
        for (size_t j = 0; j < VGA_MAXCOL; ++j)
            vga_start[(i << 6) + j] = 0;
}

void putch(const char ch) {
    if (ch == ENTER || ch == '\n') {
        vga_ch = 0;
        if (vga_line == 63)
            vga_line = 0;
        else 
            vga_line += 1;
        if (*vga_reg == 63)
            *vga_reg = 0;
        else if (vga_scroll != 0)
            *vga_reg = *vga_reg + 1;
    } else if (ch == BACKSPACE) {
        if (vga_ch == 0) {
            if (judge != 0) {
                vga_ch = 63;
                if (*vga_reg == 0 && vga_scroll != 0)
                    *vga_reg = 63;
                else if (vga_scroll != 0)
                    *vga_reg = *vga_reg - 1;
                if (vga_line == 0)
                    vga_line = 63;
                else 
                    vga_line -= 1;
            }
        } else {
            vga_ch -= 1;
        }
        vga_start[(vga_line << 6) + vga_ch] = 0;
    } else {
        vga_start[(vga_line << 6) + vga_ch] = ch;
        if (vga_ch == 63) {
            vga_ch = 0;
            if (vga_line == 63)
                vga_line = 0;
            else 
                vga_line += 1;
            if (*vga_reg == 63)
                *vga_reg = 0;
            else if (vga_scroll)
                *vga_reg = *vga_reg + 1;
        } else 
            vga_ch += 1;
    }
    if (vga_line == 29)
        vga_scroll = 1;
}

void putstr(const char *str) {
    for (size_t i = 0; str[i] != 0; ++i)
        putch(str[i]);
}

void handle_kbd(char *instr) {
    size_t i = 0;
    while (1) {
        char temp = *kbd_start;
        if (temp == ENTER)
            break;
        else if (temp != 0) {
            if (i >= 64)
                judge = 1;
            else 
                judge = 0;                
            putch(temp);
            if (temp != BACKSPACE) {
                instr[i] = temp;
                ++i;
            } else {
                if (i > 0)
                    --i;
                instr[i] = 0;
            }
        }
        if (i == 99)
            break;
    }
    putch(ENTER);
    instr[i] = 0;
}