#include "func.h"
#include "utils.h"
#include "arithmetic.h"

unsigned int fibonacci(unsigned int index) {
    if (index == 1 || index == 2)
        return 1;
    unsigned int last = 1, lastlast = 1, i = 3, res;
    while (i <= index) {
        res = last + lastlast;
        lastlast = last;
        last = res;
        ++i;
    }
    return res;
}

void handle_time(char *input) {
    if (input[1] == 0) {
        input[1] = input[0];
        input[0] = '0';
    }
    input[2] = 0;
}


unsigned int Is_Operator(const char str) {
    return str == '+' || str == '-' || str == '/' || str == '*';
}

unsigned int Is_Num(const char s) {
    return s >= '0' && s <= '9';
}

unsigned int check(const char *express)
{
    char bracket[20];
    my_memset(bracket, 20, 0);
    int cnt = 0, top = -1;
    for (int i = 0; express[i + 1] != '\0'; i++)
        if ((Is_Operator(express[i]) && express[i + 1] == ')') || (express[i] == '(' && Is_Operator(express[i + 1])) || (Is_Operator(express[i]) && Is_Operator(express[i + 1])) || (!Is_Operator(express[i]) && !Is_Num(express[i]) && (express[i] != '(' && express[i] != ')')))
            return 0;
    if (!Is_Num(express[0]) && express[0] != '(')
        return 0;
    for (int i = 0; express[i] != '\0'; i++)
        if (express[i] == '(' || express[i] == ')')
            bracket[cnt++] = express[i];

    char stack[20];
    my_memset(stack, 20, 0);
    for (int i = 0; bracket[i] != '\0'; i++) {
        if (top == -1)
            stack[++top] = bracket[i];
        else {
            if (bracket[i] == ')') {
                if (stack[top] == '(')
                    top--;
                else
                    return 0;
            }
            else
                stack[++top] = bracket[i];
        }
    }
    if (top == -1)
        return 1;
    else
        return 0;
}

void eval(char *express, char *output)
{
    char stack[20];
    int top = -1, cnt = 0;
    char result[20];
    my_memset(result, 20, 0);
    for (int i = 0; express[i] != '\0'; i++) {
        if (Is_Num(express[i])) {
            while (Is_Num(express[i]) || express[i] == '.')
                result[cnt++] = express[i++];
            i--;
            result[cnt++] = ' ';
        } else if (Is_Operator(express[i]) || express[i] == '(' || express[i] == ')') {
            if (top == -1)
                stack[++top] = express[i];
            else {
                if (express[i] == '+' || express[i] == '-') {
                    while (top >= 0 && stack[top] != '(') {    
                        result[cnt++] = stack[top--];
                        result[cnt++] = ' ';
                    }
                    stack[++top] = express[i];
                } else if (express[i] == '*' || express[i] == '/') {
                    while ((stack[top] == '*' || stack[top] == '/') && stack[top] != '(') { 
                        result[cnt++] = stack[top--];
                        result[cnt++] = ' ';
                    }
                    stack[++top] = express[i];
                } else if (express[i] == '(') {
                    stack[++top] = express[i];
                } else if (express[i] == ')') {
                    while (stack[top] != '(') { 
                        result[cnt++] = stack[top--];
                        result[cnt++] = ' ';
                    }
                    top--;
                }
            }
        }
    }

    while (top >= 0) { 
        result[cnt++] = stack[top--];
        result[cnt++] = ' ';
    }
    top = -1;
    char stack1[20][20];
    my_memset((char *)stack1, 400, 0);   
    for (int i = 0; result[i] != '\0'; i++) {
        if (Is_Operator(result[i])) {
            int a = my_atoi(stack1[top - 1]);
            int b = my_atoi(stack1[top]);
            char x[30];
            my_memset(x, 30, 0);
            if (result[i] == '*') 
                my_itoa(__mulsi3(a, b), stack1[--top], 10);
            else if (result[i] == '/')
                my_itoa(__udivsi3(a, b), stack1[--top], 10);
            else if (result[i] == '+') 
                my_itoa(a + b, stack1[--top], 10);
            else
                my_itoa(a - b, stack1[--top], 10);
        } else if (result[i] != ' ') {
            int cnt = 0;
            ++top;
            while (result[i] != ' ') 
                stack1[top][cnt++] = result[i++];
            stack1[top][cnt] = 0;
        }
    }
    for (int i = 0; stack1[0][i] != 0; ++i)
        output[i] = stack1[0][i];
}