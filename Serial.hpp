#include <iostream>

using namespace std;

void pi_uart_write(char *ch, int len)
{
    int i;
    char *p = ch;
    for (i = 0; i < len; i++)
    {
        printf("%c\n", *p);
        p++;
    }

    return;
}