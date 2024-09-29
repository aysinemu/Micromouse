
#include <stdbool.h>
#include <stdio.h>
#include "Main.h"
#define MAX_SIZE 1000

typedef struct {
    coord items[MAX_SIZE];
    int front;
    int rear;
} Queue;

void initializeQueue(Queue* q);
bool isEmptyQueue(Queue* q);
bool isFullQueue(Queue* q);

void pushQueue(Queue* q, coord value);
void popQueue(Queue* q);
coord  peekQueue(Queue* q);
int sizeQueue(Queue* q);
