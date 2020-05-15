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
#include "history.h"
#include "variable.h"
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
    } else if (ch == '\t') {  // For Tab, autocomplete command or file name
      // TODO: auto autocomplete
      int num_byte = findInDirectory(input);

      // printf("\n%d: %s%s", num_byte, path_str, input);
      // Print again if multiple selections are available
      if (num_byte > 0) {
        for (int i = num_byte; i > 0; i--) {
          putchar(input[strlen(input) - i]);
          index++;
        }
      } else if (num_byte < 0) {
        printf("%s%s", path_str, input);
      }
    } else if (ch == 0x1b) {  // Arrow buttons
      // Do nothing for arrow buttons for now
      int ch2 = getchar();
      int ch3 = getchar();
      int ch4, ch5, ch6;

      switch (ch3) {
        case 0x41:  // Up button
          // Clear input
          while (index > 0) {
            input[--index] = '\0';
            putchar('\b');
            putchar(0x20);
            putchar('\b');
          }
          // Get previous history as input and display
          HIST_ENTRY* prev_hist = previous_history();
          if (prev_hist != NULL) {
            char* prev_val = prev_hist->line;
            int len = strlen(prev_val);
            for (int i = 0; i < len; i++) {
              input[i] = prev_val[i];
              putchar(prev_val[i]);
              index++;
            }
          }
          break;
        case 0x42:  // Down button

          // Clear input
          while ((index > 0)) {
            input[--index] = '\0';
            putchar('\b');
            putchar(0x20);
            putchar('\b');
          }
          // Get next history as input and display
          HIST_ENTRY* next_hist = next_history();
          if (next_hist != NULL) {
            char* next_val = next_hist->line;
            int len = strlen(next_val);

            for (int i = 0; i < len; i++) {
              input[i] = next_val[i];
              putchar(next_val[i]);
              index++;
            }
          }
          break;
        case 0x43:  // Right button
        case 0x44:  // Left button
          // TODO: support moving curser left or right
          break;
        case 0x31:  // shift arrow button
          ch4 = getchar();
          ch5 = getchar();
          ch6 = getchar();
          switch (ch6) {
            case 0x41:  // Up button
              while (index > 0) {
                input[--index] = '\0';
                putchar('\b');
                putchar(0x20);
                putchar('\b');
              }
              // Get previous history as input and display
              HIST_ENTRY* prev_hist = previous_history();
              if (prev_hist != NULL) {
                char* prev_val = prev_hist->line;
                int len = strlen(prev_val);
                for (int i = 0; i < len; i++) {
                  input[i] = prev_val[i];
                  putchar(prev_val[i]);
                  index++;
                }
              }
              break;
            case 0x42:  // Down button
              // Clear input
              while ((index > 0)) {
                input[--index] = '\0';
                putchar('\b');
                putchar(0x20);
                putchar('\b');
              }
              // Get next history as input and display
              HIST_ENTRY* next_hist = next_history();
              if (next_hist != NULL) {
                char* next_val = next_hist->line;
                int len = strlen(next_val);

                for (int i = 0; i < len; i++) {
                  input[i] = next_val[i];
                  putchar(next_val[i]);
                  index++;
                }
              }
              break;
            case 0x43:  // Right button
            case 0x44:  // Left button
              // TODO: support moving curser left or right
              break;
            default:
              printf("invalid 0x%x%x%x %x%x%x\n", ch, ch2, ch3, ch4, ch5, ch6);
          }
          break;
        default:
          printf("invalid 0x%x%x%x\n", ch, ch2, ch3);
      }
    } else {
      input[index++] = ch;
      printf("%c", ch);
      if (ch == '\n') break;
    }
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
  static char saved_input[MAX_COMM_SIZE] = {0x0};  // to detect second tab
  int num_occur = 0;
  int added_byte = 0;
  int isCommand = 0;

  // search the last word in input
  for (int i = 0; input[i] != '\0'; i++) {
    if (input[i] == ' ') {
      last_input = &input[i + 1];
    }
  }

  // exit if empty string
  if (last_input[0] == '\0') return 0;

  // if command, it will search full path list
  if (last_input == input) isCommand = 1;

  char* path_list = searchVariable("PATH");
  char* path_list_copy = NULL;
  char* path_tokens[MAX_TOK + 1] = {NULL};

  path_list_copy = (char*)malloc(strlen(path_list));
  strncpy(path_list_copy, path_list, strlen(path_list));

  // Add current directory first to the tokens
  char curr_dir[2] = {0};
  curr_dir[0] = '.';
  path_tokens[0] = curr_dir;

  // parse PATH list to tokens
  char* token = strtok(path_list_copy, ":\0\n");
  int n = 1;
  for (; token != NULL && n < MAX_TOK; ++n) {
    path_tokens[n] = token;
    token = strtok(NULL, ":\0\n");
  }

  for (int m = 0; path_tokens[m] != NULL && m < MAX_TOK; ++m) {
    if ((isCommand == 0) && (m > 0))
      break;  // if not command, search current dir only
    d = opendir(path_tokens[m]);

    if (d) {
      while ((dir = readdir(d)) != NULL) {
        // Find matching file name
        if (strncmp(dir->d_name, last_input, strlen(last_input)) == 0) {
          if (strncmp(saved_input, input, strlen(input)) == 0) {
            printf("\n%s", dir->d_name);
          }
          strncpy(file_found, dir->d_name, strlen(dir->d_name));
          num_occur++;
        }
      }
      closedir(d);
    }
  }

  free(path_list_copy);

  if (strncmp(saved_input, input, strlen(input)) == 0) {
    printf("\n");
    strncpy(saved_input, "\0", 1);
    return -1;
  } else {
    strncpy(saved_input, input, strlen(input));
  }

  // update input only if one occurrence
  if (num_occur == 1) {
    added_byte = strlen(file_found) - strlen(last_input);
    strncpy(last_input, file_found, strlen(file_found));
  }

  return added_byte;
}
