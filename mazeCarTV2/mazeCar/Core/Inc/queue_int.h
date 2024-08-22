
#include <stdbool.h>
#include <stdio.h>

#define MAX_SIZE 1000

typedef struct {
    int items[MAX_SIZE];
    int front;
    int rear;
} QueueInt;

void initializeQueueInt(QueueInt* q);
bool isEmptyQueueInt(QueueInt* q);
bool isFullQueueInt(QueueInt* q);

void pushQueueInt(QueueInt* q, int value);
void popQueueInt(QueueInt* q);
int  peekQueueInt(QueueInt* q);
int sizeQueueInt(QueueInt* q);
