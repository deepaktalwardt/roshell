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
  int c;

  sourceFile = fopen(input[1], "r");
  if (!sourceFile) {
    printf("File cannot open or does not exist, %s\n", input[1]);
    return -1;
  }

  // Search any non ascii char in the file to determine if binary file
  //  This may not be perfect to catch all non-text file.
  while ((c=fgetc(sourceFile)) != EOF) {
    if (c > 0x7B) {
      printf("%s may be binary\n", input[1]);
      fclose(sourceFile);
      return -1;
    }
  }

  // reset file pointer
  fseek(sourceFile, 0, SEEK_SET);

  while (fgets(sourceLine, MAX_COMM_SIZE, sourceFile) != NULL) {
    executeCommand(sourceLine);
  }

  fclose(sourceFile);

  return 0;
}
