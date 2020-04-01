#ifndef COMMAND_H
#define COMMAND_H

void executeCommand(char* input);
int parseInput(char input[], char* tokens[], size_t max_tok);

#endif //COMMAND_H
