#ifndef SYS_H
#define SYS_H

// VGA
#define VGA_START 0x00200000
#define VGA_REG 0x00400000
#define VGA_MAXLINE 64
#define VGA_MAXCOL 64

// KBD
#define KBD_START 0x00300000
#define BACKSPACE 0x8
#define ENTER 0xd

typedef unsigned int size_t;


void putstr(const char *);
void putch(const char);
void vga_init(void);
void handle_kbd(char *);

#endif