#include "circqueue.h"

#include <string.h>

void circqueue_clear(circqueue_t *q)
{
	q->head = q->count = 0;
}

int circqueue_copy(char *dest, const circqueue_t *q)
{
	memcpy(dest, &q->buffer[q->head], CIRCQUEUE_LENGTH - q->head);
	if (q->head != 0)
		memcpy(dest + CIRCQUEUE_LENGTH - q->head, q->buffer, q->head);
	return q->count;
}

bool circqueue_isempty(const circqueue_t *q)
{
	return (q->count == 0);
}

bool circqueue_ishalffull(const circqueue_t *q)
{
	return (q->count >= CIRCQUEUE_LENGTH / 2);
}

bool circqueue_isfull(const circqueue_t *q)
{
	return (q->count == CIRCQUEUE_LENGTH);
}

void circqueue_enqueue(circqueue_t *q, char data)
{
	if (circqueue_isfull(q))
		return;

	q->buffer[(q->head + q->count++) % CIRCQUEUE_LENGTH] = data;
}

char circqueue_dequeue(circqueue_t *q)
{
	if (circqueue_isempty(q))
		return 0;

	int old_head = q->head++;
	if (q->head == CIRCQUEUE_LENGTH)
		q->head = 0;

	q->count--;
	return q->buffer[old_head];
}
