#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// variable list structure
typedef struct varItem {
  char* name;
  char* value;
  struct varItem* next;
} varList;

static varList* head = NULL;

//------------------------------------------------------------------------------
//  addVariable()
//
//  Function: Add new variable
//  To-Do: Replace existing variable if duplicate is found
//
//------------------------------------------------------------------------------
void addVariable(char* input) {
  varList* ptr = head;
  char* tokens[2] = {NULL};

  // create copy not to corrupt input by strtok
  char* input_copy = (char*)malloc(strlen(input) + 1);
  strcpy(input_copy, input);

  tokens[0] = strtok(input_copy, "=");
  tokens[1] = strtok(NULL, " \n");

  // exit if empty variable is passed for now
  // improvement required to handle saving empty variable
  if (tokens[1] == NULL) return;

  // empty list, add to head
  if (head == NULL) {
    head = (varList*)malloc(sizeof(varList));
    ptr = head;
    ptr->name = (char*)malloc(strlen(tokens[0]) + 1);
    strcpy(ptr->name, tokens[0]);
    ptr->next = NULL;
  } else {
    // search through list
    for (; ptr->next != NULL; ptr = ptr->next) {
      // variable matches, free value and overwrite it
      if (!strcmp(ptr->name, tokens[0])) {
        free(ptr->value);
        break;
      }
    }
    // last item in the list
    if (ptr->next == NULL) {
      // if last var matches, free value and overwrite it
      if (!strcmp(ptr->name, tokens[0])) {
        free(ptr->value);
      } else {
        // add to the end of list
        ptr->next = (varList*)malloc(sizeof(varList));
        ptr = ptr->next;
        ptr->name = (char*)malloc(strlen(tokens[0]) + 1);
        strcpy(ptr->name, tokens[0]);
        ptr->next = NULL;
      }
    }
  }
  // copy variable value
  ptr->value = (char*)malloc(strlen(tokens[1]) + 1);
  strcpy(ptr->value, tokens[1]);
}

//------------------------------------------------------------------------------
//  displayVariable()
//
//  Function: Displau all variables including environment variables
//  To-Do: Separate environment variables and other variables
//
//------------------------------------------------------------------------------
void displayVariable() {
  varList* ptr = head;
  if (ptr == NULL) {
    printf("No list\n");
    return;
  } else {
    printf("%s=%s\n", ptr->name, ptr->value);
    while (ptr->next != NULL) {
      ptr = ptr->next;
      printf("%s=%s\n", ptr->name, ptr->value);
    }
  }
}

//------------------------------------------------------------------------------
//  searchVariable()
//
//  Function: Search a variable by scanning the list
//  Output: variable value string or NULL if not found
//
//------------------------------------------------------------------------------
char* searchVariable(char* var) {
  varList* ptr = head;
  if (ptr == NULL) {
    printf("Empty List\n");
    return NULL;
  } else {
    if (!strcmp(ptr->name, var)) {
      return ptr->value;
    }
    do {
      ptr = ptr->next;
      if (!strcmp(ptr->name, var)) {
        return ptr->value;
      }
    } while (ptr->next != NULL);

    // variable not found
    return NULL;
  }
}

//------------------------------------------------------------------------------
//  searchVariable()
//
//  Function: Search a variable by scanning the list
//  Output: variable value string or NULL if not found
//
//------------------------------------------------------------------------------
void removeVariable(char* var) {
  varList* ptr = head;
  if (ptr == NULL) {
    printf("Empty List\n");
  } else {
    if (!strcmp(ptr->name, var)) {
      printf("Remove variable %s, %s\n", ptr->name, ptr->value);
    }
    do {
      if (!strcmp(ptr->name, var)) {
        printf("Remove variable %s, %s\n", ptr->name, ptr->value);
      }
      ptr = ptr->next;
    } while (ptr->next != NULL);
  }
}
