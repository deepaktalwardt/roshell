#ifndef COMMAND_H
#define COMMAND_H

#include <stddef.h> // for size_t definition

int parseInput(char input[], char* tokens[], size_t max_tok);
void executeLine(char* input);

void executeShellCommand(char* input);
void executeProgram(char* tokens[]);


#endif //COMMAND_H
