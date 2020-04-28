#include "input.h"
#include <ctype.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
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
  int index = 0;

  // original code from
  // https://stackoverflow.com/questions/32390617/get-keyboard-interrupt-in-c
  //   Change the input to non-cannonical mode

  // store current setting
  tcgetattr(STDIN_FILENO, &old_in_t);
  new_in_t = old_in_t;

  // non-canonical mode
  new_in_t.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &new_in_t);

  while (1) {
    ch = getchar();

    // handle only the accepted ascii char.
    if (isalnum(ch) || (ch > 31 && ch < 127) || ch == '\n') {
      input[index++] = ch;
      printf("%c", ch);
    }

    // Remove char for backspace or del (for Mac) char
    if (ch == '\b' || ch == 0x7f) {
      if (index > 0) {
        // remove one character in the input string
        input[--index] = '\0';

        // remove one character and move curser back in stdout
        putchar('\b');
        putchar(0x20);
        putchar('\b');
      }
    }

    // For Tab, autocomplete command or file name
    if (ch == '\t') {
      // TODO: auto autocomplete
      int num_byte = findInDirectory(input);
      // Print again if multiple selections are available
      if (num_byte > 0) {
        for (int i = num_byte; i > 0; i--) {
          putchar(input[strlen(input) - i]);
          index++;
        }
      }
    }
    if (ch == '\n') break;
  }

  // restore input flag
  tcsetattr(STDIN_FILENO, TCSANOW, &old_in_t);

  return 0;
}

//------------------------------------------------------------------------------
//  findInDirectory()
//
//  Function: Search file name in current directory and update input
//  Return: Number of bytes added to the file name in input
//
//------------------------------------------------------------------------------
int findInDirectory(char* input) {
  DIR* d;
  struct dirent* dir;
  char* last_input = input;
  char file_found[MAX_COMM_SIZE] = {0x0};
  int num_occur = 0;
  int added_byte = 0;

  // search the last word in input
  for (int i = 0; input[i] != '\0'; i++) {
    if (input[i] == ' ') {
      last_input = &input[i + 1];
    }
  }

  // exit if empty string
  if (last_input[0] == '\0') return 0;

  d = opendir(".");
  if (d) {
    while ((dir = readdir(d)) != NULL) {
      // Find matching file name
      if (strncmp(dir->d_name, last_input, strlen(last_input)) == 0) {
        strncpy(file_found, dir->d_name, strlen(dir->d_name));
        num_occur++;
      }
    }
    // update input only if one occurrence
    if (num_occur == 1) {
      added_byte = strlen(file_found) - strlen(last_input);
      strncpy(last_input, file_found, strlen(file_found));
    }
    closedir(d);
  }
  return added_byte;
}
