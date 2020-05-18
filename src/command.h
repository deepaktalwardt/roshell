#ifndef COMMAND_H
#define COMMAND_H

#include <stddef.h> // for size_t definition
#include <signal.h> // pid_t definition

int parseInput(char input[], char* tokens[], size_t max_tok);
void executeLine(char* input);

void executeShellCommand(char* input);
void executeProgram(char* tokens[]);

// Signal handler for Interrupt signal
void sigint_handler(int sig);

//Global variable for mainting the process id used for Ctrl+c functionality
pid_t process_id;
pid_t main_process;

#endif //COMMAND_H
