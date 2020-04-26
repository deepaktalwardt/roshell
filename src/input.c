#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <dirent.h>
#include <string.h>
#include "input.h"
#include "const.h"

//------------------------------------------------------------------------------
//  readInput()
//
//  Function: Read input in non-canonical mode to  handle it while typing
//
//------------------------------------------------------------------------------
int readInput(char* input, int size, char* path_str) {

  struct termios old_in_t, new_in_t;
  int ch;
  int oldf;
  char last_input[MAX_COMM_SIZE+1] = {0x0};
  int index=0;
  int last_index=0;

  // original code from https://stackoverflow.com/questions/32390617/get-keyboard-interrupt-in-c
  //   Change the input to non-cannonical mode

  // store current setting
  tcgetattr(STDIN_FILENO, &old_in_t);
  new_in_t = old_in_t;
  // non-canonical mode
  new_in_t.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &new_in_t);
  //oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  //fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  while (1) {

    ch = getchar();

    // handle only the accepted ascii char.
    if (isalnum(ch) || (ch > 31 && ch <127) || ch == '\n') {
      input[index++] = ch;
      last_input[last_index++] = ch;
      last_input[last_index] = '\0';
      printf("%c", ch);
    }

    if (ch == 0x20) {
      last_index = 0;
      last_input[last_index] = '\0';
    }

    // Remove char for backspace or del (for Mac) char
    if(ch == '\b' || ch == 0x7f) {
      if (index > 0) {

        // remove one character in the input string
        input[--index] = '\0';
        last_input[--last_index] = '\0';

        // remove one character and move curser back in stdout
        putchar('\b');
        putchar(0x20);
        putchar('\b');
      }
    }

    // For Tab, autocomplete command or file name
    if(ch == '\t' && last_input[0] != '\0') {
      // TODO: auto autocomplete
      listDirectory(last_input);
      // Print again if multiple selections are available
      printf("\n%s%s", path_str, input);
    }
    if(ch == '\n') break;

  }

  // restore input flag
  tcsetattr(STDIN_FILENO, TCSANOW, &old_in_t);
  //fcntl(STDIN_FILENO, F_SETFL, oldf);

  return 0;
}


//------------------------------------------------------------------------------
//  readInput()
//
//  Function: Read input in non-canonical mode to  handle it while typing
//
//------------------------------------------------------------------------------
void listDirectory(char* input) {

  DIR *d;
  struct dirent *dir;
  d = opendir(".");

  printf("\n");
  if (d)
  {
    while ((dir = readdir(d)) != NULL)
    {
      if (strstr(dir->d_name, input) != NULL)
        printf("%s\n", dir->d_name);
    }
    closedir(d);
  }
}
