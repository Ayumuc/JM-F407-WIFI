#include "../../Utils/dataFilter.h"

void simpleQueueInit(SIMPLE_QUEUE *q)
{
 	unsigned char i;
	q->point = 0;
	for(i=0; i<MAXSIZE; i++)
	{
		q->elems[i]=0;
	}
}

void simpleQueueEnQueue(SIMPLE_QUEUE *q, int data)
{
	q->elems[q->point]=data;
	if(++(q->point) >= MAXSIZE)
	{
		q->point = 0;
	}
}

void QuickSort(int *a, int low, int high)
{
    int i = low;
    int j = high;
    int key = a[low];
    if (low >= high)
    {
        return ;
    }
    while (low < high)
    {
        while (low < high && key <= a[high])
        {
            --high; 
        }
        if (key > a[high])
        {
            a[low] = a[high]; 
            ++low;
        }
        while (low < high && key >= a[low])
        {
            ++low; 
        }
        if (key < a[low])
        {
            a[high] = a[low]; 
            --high;
        }
    }
    a[low] = key;  
    QuickSort(a, i, low-1); 
    QuickSort(a, low+1, j); 
}

void copyQueue(int dest[], int src[], char size)
{
	unsigned char i;
	for(i=0; i<size; i++)
	{
		dest[i] = src[i];
	}
}

int sortedQueue[MAXSIZE];

int enQueueAndCalcAverage(SIMPLE_QUEUE *queue, int data)
{
	unsigned char i;
	int sum=0;
	simpleQueueEnQueue(queue, data);
	copyQueue(sortedQueue, queue->elems, MAXSIZE);
	QuickSort(sortedQueue, 0, MAXSIZE-1);

	for(i=5; i<(MAXSIZE-5); i++)
	{
		sum += sortedQueue[i];
	}	
	return sum / (MAXSIZE-10);
}
