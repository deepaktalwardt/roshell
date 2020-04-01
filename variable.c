#include <stdlib.h>
#include <stdio.h>
#include <string.h>

// variable list structure
typedef struct varItem {
    char * name;
    char * value;
    struct varItem * next;
} varList;

static varList * head = NULL;

//------------------------------------------------------------------------------
//  addVariable()
//
//  Function: Add new variable
//  To-Do: Replace existing variable if duplicate is found
//
//------------------------------------------------------------------------------
void addVariable(char* input)
{
    varList * ptr = head;
    char* tokens[2] = { NULL };
    tokens[0] = strtok(input, "=");
    tokens[1] = strtok(NULL, " \n");

    // exit if empty variable is passed for now
    // improvement required to handle saving empty variable
    if (tokens[1]==NULL) return;

    if (head==NULL)
    {
        head = (varList *) malloc(sizeof(varList));
        ptr = head;
    }
    else
    {
        for (; ptr->next != NULL; ptr = ptr->next);
        ptr->next = (varList *) malloc(sizeof(varList));
        ptr = ptr->next;
    }
    ptr->name = (char *) malloc(strlen(tokens[0])+1);
    ptr->value = (char *) malloc(strlen(tokens[1])+1);
    ptr->next = NULL;

    strcpy(ptr->name, tokens[0]);
    strcpy(ptr->value, tokens[1]);
}

//------------------------------------------------------------------------------
//  displayVariable()
//
//  Function: Displau all variables including environment variables
//  To-Do: Separate environment variables and other variables
//
//------------------------------------------------------------------------------
void displayVariable()
{
    varList * ptr = head;
    if (ptr==NULL)
    {
        printf("No list\n");
        return;
    }
    else
    {
        printf("%s=%s\n", ptr->name, ptr->value);
        while (ptr->next != NULL)
        {
            ptr = ptr->next;
            printf("%s=%s\n", ptr->name, ptr->value);
        }
    }
}
