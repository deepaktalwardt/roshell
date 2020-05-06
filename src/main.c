#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>
#include "command.h"
#include "const.h"
#include "input.h"
#include "variable.h"

int main(int argc, char* argv[], char* envp[]) {
  // argc - argument count
  // argv - argument vector
  // envp - environment pointer
  // TODO: Should roshell inherit envp from parent shells?

  char hostname[_SC_HOST_NAME_MAX];
  char username[_SC_LOGIN_NAME_MAX];
  gethostname(hostname, _SC_HOST_NAME_MAX);  // system call to get the hostname
  getlogin_r(username, _SC_LOGIN_NAME_MAX);  // system call to get the username

  // print out all the environment variables
  for (int i = 0; envp[i]; ++i) {
    printf("environment variable %d: %s \n", i, envp[i]);
    if (strchr(envp[i], '=') != NULL) {
      // add to variable list
      addVariable(envp[i], 0);
    }
  }

  while (1) {
    char input[MAX_COMM_SIZE + 1] = {0x0};
    char path_str[MAX_COMM_SIZE + 1] = {0x0};

    char* path = searchVariable("PWD");
    sprintf(path_str,
            "\033[7m [roshell] "
            "\033[36;1;1m%s@%s:\033[33;1;1m%s\033[37;1;1m\033[27m$ ",
            username, hostname, path);
    printf("%s", path_str);

    readInput(input, MAX_COMM_SIZE, path_str);
    executeLine(input);
  }
}
