#include <stdio.h>
int main()
{
    char c;
    printf("Enter a character: ");

    // Reads character input from the user
    scanf("%c", &c);  
    
    // %d displays the integer value of a character
    // %c displays the actual character
    printf("ASCII value of '%c' = %d[dec] %#02x[hex] %#03o[oct]\n", c, c, c, c);
    return 0;
}
