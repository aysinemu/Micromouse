#include "stack.h"

void initializeStack(Stack *stack) {
    stack->top = -1;  
}

bool isEmptyStack(Stack *stack) {
    return stack->top == -1;  
}

bool isFullStack(Stack *stack) {
    return stack->top == MAX_SIZE - 1;  
}

void pushStack(Stack *stack, coord value) {
    if (isFullStack(stack)) {
        return;
    }
    stack->arr[++stack->top] = value;
}

coord popStack(Stack *stack) {
    if (isEmptyStack(stack)) {
        coord error = {-1, -1, -1}; // or any other error indication
        return error;
    }
    return stack->arr[stack->top--];
}

coord peekStack(Stack *stack) {
    if (isEmptyStack(stack)) {
        coord error = {-1, -1, -1}; // or any other error indication
        return error;
    }
    return stack->arr[stack->top];
}
int sizeStack(Stack *stack){
    return stack->top + 1;
}

