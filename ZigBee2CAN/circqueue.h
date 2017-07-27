#ifndef CIRCQUEUE_H
#define CIRCQUEUE_H

#include <stdbool.h>

#define CIRCQUEUE_LENGTH 90 // Lunghezza massima

typedef struct
{
	char buffer[CIRCQUEUE_LENGTH];
	int head, count;
} circqueue_t;

void circqueue_clear(circqueue_t *q);
int circqueue_copy(char *dest, const circqueue_t *q);
bool circqueue_isempty(const circqueue_t *q);
bool circqueue_ishalffull(const circqueue_t *q);
bool circqueue_isfull(const circqueue_t *q);
void circqueue_enqueue(circqueue_t *q, char data);
char circqueue_dequeue(circqueue_t *q);

#endif /* CIRCQUEUE_H */
