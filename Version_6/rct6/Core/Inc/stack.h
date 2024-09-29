#include <stdio.h>
#include <stdbool.h>
#include "Main.h"

#define MAX_SIZE 1000

typedef struct {
    coord arr[MAX_SIZE];  
    int top;        
} Stack;

void initializeStack(Stack *stack) ;
bool isEmptyStack(Stack *stack);
bool isFullStack(Stack *stack);
void pushStack(Stack *stack, coord value);
coord popStack(Stack *stack);

coord peekStack(Stack *stack);

int sizeStack(Stack *stack);
