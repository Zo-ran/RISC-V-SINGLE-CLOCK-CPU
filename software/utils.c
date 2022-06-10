#include "utils.h"
#include "arithmetic.h"

int my_strcmp(const char *str1, const char* str2) {
	if (str1 == NULL || str2 == NULL)
		return 1;
	while (*str1 == *str2) {
		if (*str1 == '\0')
			return 0;
		str1++;
		str2++;
	}
	return *str1 > *str2 ? 1 : -1;
}

int my_memset(char *buffer, int size, char value) {
	if (buffer == NULL)
		return -1;
	for (int i = 0; i < size ; ++i)
		buffer[i] = value;
	return 0;
}

int my_atoi(char *arr) {
    if (arr == NULL)
        return -1;
    int index = 0;
    int num = 0;
    int flag = 1;
        
    if(arr[index] == '-')
        flag = -1; 
    if(arr[index] == '-' || arr[index] == '+')
        index++;
    while(arr[index] >= '0' && arr[index] <= '9') {
        num = num*10 + arr[index] - '0';
        index++;
    }
    return flag == 1 ? num : -num;
}

char *my_itoa(int num, char *str, unsigned int radix) {
    char index[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
    unsigned int unum;
    int i = 0, j, k;
    if (radix == 10 && num < 0) {
        unum = (unsigned) - num;
        str[i++] = '-';
    }
    else unum = (unsigned)num;
    do {
        str[i++] = index[__umodsi3(unum, radix)];
        unum /= radix;
    } while(unum);
    str[i] = '\0';
    if (str[0] == '-') k = 1;
    else k = 0;
    char temp;
    for (j = k; j <= (i - 1) / 2; ++j) {
        temp = str[j];
        str[j] = str[i - 1 + k - j];
        str[i - 1 + k - j] = temp;
    }
    return str;
}