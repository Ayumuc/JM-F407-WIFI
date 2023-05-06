#ifndef DATA_FILTER_H_
#define DATA_FILTER_H_

#define MAXSIZE 15

typedef struct{
	int point;
	int elems[MAXSIZE];
}SIMPLE_QUEUE;

void QuickSort(int *a, int low, int high);
void simpleQueueInit(SIMPLE_QUEUE *q);
void simpleQueueEnQueue(SIMPLE_QUEUE *q, int data);
int enQueueAndCalcAverage(SIMPLE_QUEUE *queue, int data);

#endif
