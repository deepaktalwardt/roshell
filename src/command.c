#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/wait.h>
#include <signal.h>
#include "command.h"
#include "const.h"
#include "source.h"
#include "variable.h"
//Global variable for mainting the process id used for Ctrl+c functionality
pid_t pid,main_process;
/*
Interrupt signal handler that tests for the child process in the if statement
Else it kill the whole program.
*/
void sigint_handler(int sig){
    if(pid!=main_process){
    kill(pid,SIGQUIT);
    pid=main_process;
    }
    else{
        printf("\n");
        exit(0);
    }
}

void executeCommand(char* input)
{
  main_process=pid=getpid(); //Make the current process id same as the main process id
  signal(SIGINT,sigint_handler); //Signal handler that uses the above interrupt handler function
  char* args[MAX_ARGS + 1] = { NULL };
  input[strlen(input) - 1] = '\0'; //terminate with null, rather than with \n
  
  char* token = strtok(input, " ");
  for(int i=0; token != NULL && i<MAX_ARGS; ++i)
  {
    args[i] = token;
    token = strtok(NULL, " ");
  }
  if(args[0] == NULL) return; //empty input

  // if command includes "=" set variable with the value after "=" as sting
  if (strchr(args[0], '=') != NULL)
  {
      //printf("variable set, %s\n", args[0]);
      addVariable(args[0]);
  }
  else if (strcmp(args[0], "exit") == 0) // exit shell
  {
      exit(0);
  }
  else if (strcmp(args[0], "env") == 0) // display all variables
  {
      displayVariable();
  }
  else if (strcmp(args[0], "source") == 0) // source command
  {
      printf("source command\n");
      sourceCommand(args);
  }
  /* 
  Creates a child process that executes a function such as quick ssh or some roshell graphics
  Tried this way, if you have anything better please suggest.
  */
  else if (strcmp(args[0], "test") == 0) // test command
  {
      printf("test command\n");
      pid=fork();
      if(pid==0)
      {
          pid=getpid(); // Update the current process id with the id of this child
          sleep(10000); // A dummy sleep() call. You can call a function here.
      }
      wait(NULL);
  }
  else
  {

     if (fork() == 0) // if inside the child process
     {
        int comm_res = execvp(args[0], args);

        if(comm_res == -1)  //execvp encountered error
        {
            printf("Command '%s' exited with the following error: %s \n", args[0], strerror(errno));  
            exit(-1);
        }
        else exit(0);
     }
  }
  wait(NULL);
}
