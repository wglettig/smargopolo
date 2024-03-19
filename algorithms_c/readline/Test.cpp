#include <stdio.h>

int main (int argc, char* argv[])
{
    FILE * pSource = fopen("params.txt", "r");
    while (!feof(pSource))
    {
        char arrLine[101];
        fgets(arrLine, 100, pSource);
        printf( "%s", arrLine);
    }
    fclose(pSource);

    //getchar();
    return 0;
}

