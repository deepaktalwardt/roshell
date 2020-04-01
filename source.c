#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/wait.h>
#include "source.h"
#include "const.h"
#include "command.h"

//------------------------------------------------------------------------------
//  sourceCommand()
//
//  Basic Functionality is to read a text file with list of shell commands
//  Improvement is needed to support shell scripting
//
//------------------------------------------------------------------------------
int sourceCommand(char** input)
{
    FILE  *sourceFile;
    char sourceLine[MAX_COMM_SIZE+1] = {0x0};

    sourceFile = fopen(input[1], "r");
    while(fgets(sourceLine, MAX_COMM_SIZE, sourceFile) != NULL)
    {
        executeCommand(sourceLine);
    }

    fclose(sourceFile);

    return 0;
}
