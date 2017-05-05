#define CIRCBUF_DEF(x,y) uint8_t x##_space[y]; circBuf_t x = { x##_space,0,0,y}

int circBufPush(circBuf_t *c, uint8_t data)
{
    int next = c->head + 1;
    if (next >= c->maxLen)
        next = 0;
 
    // Cicular buffer is full
    if (next == c->tail)
        return -1;  // quit with an error
 
    c->buffer[c->head] = data;
    c->head = next;
    return 0;
}
 
int circBufPop(circBuf_t *c, uint8_t *data)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (c->head == c->tail)
        return -1;  // quit with an error
 
    *data = c->buffer[c->tail];
    c->buffer[c->tail] = 0;  // clear the data (optional)
 
    int next = c->tail + 1;
    if(next >= c->maxLen)
        next = 0;
 
    c->tail = next;
 
    return 0;
}

CIRCBUF_DEF(cb, 32);
 
uint8_t thisIsYourAppCode(uint8_t data)
{
//    ...
//    some code..
//    ...
    if (circBufPush(&cb, data)) {
//            DBG("Out of space in CB");
        }
//    ...
//    some code..
//    ...
    if (circBufPop(&cb, &data)) {
//            DBG("CB is empty");
        }
//        ... some code.. ... 
        return data; 
}
 

