#include "cd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include "variable.h"

//------------------------------------------------------------------------------
//  changeDirectory()
//
//  Function: Change the current directory
//              and update PWD in environment variables
//
//------------------------------------------------------------------------------
void changeDirectory(char* input) {
  // change current directory
  chdir(input);
  // get current directory string
  char* pwd = getcwd(NULL, 0);
  char* pwd_to_add = (char*)malloc(strlen(pwd) + 5);
  //change format to add or replace PWD variable.
  sprintf(pwd_to_add, "PWD=%s", pwd);

  addVariable(pwd_to_add, 1);
  
  free(pwd);
  free(pwd_to_add);
}
