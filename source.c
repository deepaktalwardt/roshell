#include "source.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "command.h"
#include "const.h"

//------------------------------------------------------------------------------
//  sourceCommand()
//
//  Basic Functionality is to read a text file with list of shell commands
//  Improvement is needed to support shell scripting
//
//------------------------------------------------------------------------------
int sourceCommand(char** input) {
  FILE* sourceFile;
  char sourceLine[MAX_COMM_SIZE + 1] = {0x0};

  sourceFile = fopen(input[1], "r");
  if (!sourceFile) {
    printf("File cannot open or does not exist, %s\n", input[1]);
    return -1;
  }

  while (fgets(sourceLine, MAX_COMM_SIZE, sourceFile) != NULL) {
    executeCommand(sourceLine);
  }

  fclose(sourceFile);

  return 0;
}
