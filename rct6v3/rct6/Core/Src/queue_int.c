#include "queue_int.h"


void initializeQueueInt(QueueInt* q)
{
    q->front = -1;
    q->rear = 0;
}

bool isEmptyQueueInt(QueueInt* q) { 
    return (q->front == q->rear - 1);
}

bool isFullQueueInt(QueueInt* q) { 
    return (q->rear == MAX_SIZE); 
}

void pushQueueInt(QueueInt* q, int value)
{
    if (isFullQueueInt(q)) {
        return;
    }
    q->items[q->rear] = value;
    q->rear++;
}

void popQueueInt(QueueInt* q)
{
    if (isEmptyQueueInt(q)) {
        return;
    }
    q->front++;
}

int peekQueueInt(QueueInt* q)
{
    if (isEmptyQueueInt(q)) {
        return -1;
    }
    return q->items[q->front + 1];
}

int sizeQueueInt(QueueInt* q){
    return q->rear - q->front - 1;
}

