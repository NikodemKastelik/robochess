/*
 * my_queue.c
 *
 * Created: 2017-12-20 20:22:50
 *  Author: Nikodem Kastelik
 *
 * @brief
 *
 * Implements basic concurrent queue. Concurrency is maintained by mutex locking
 * mechanism. This operation is atomic with additional workaround with storing
 * previous PRIMASK value (global interrupt flag) and then disabling interrupts.
 *
 * @TODO
 * - raise some error on fifo-circural buffer overflow (when w(rite)index 
 *   catches r(ead)index)
 * 
 */ 

//#define DEBUG 1

#include "my_queue.h"
#include "stm32f4xx.h"


/** @brief Perform nonblocking lock on queue mutex.
      * @param my_queue* Queue - pointer to this lock queue
      * @return TRUE of FALSE - with regards to the fact if mutex was locked 
      * or not.
      */
uint8_t trylock_(volatile myQueue * Queue)
{       
    //disable interrupts
    uint32_t old_primask = __get_PRIMASK();
    __disable_irq();
    if(Queue->mutex != LOCKED)
    {
        Queue->mutex = LOCKED;
        Queue->old_primask = old_primask;
        return TRUE;          
    }
    else
    {
    	__set_PRIMASK( old_primask );
        return FALSE;
    }

}

/** @brief Releases queue lock.
      * @param my_queue* Queue - pointer to this lock queue
      * @return nothing
      */
void release_(volatile myQueue * Queue)
{
    Queue->mutex = UNLOCKED;
    __set_PRIMASK( Queue->old_primask );
}


/** @brief Puts string on queue
      * @param my_queue* Queue - pointer to queue to put string to
      * @param char* arr - array filled with string to put
      * @return nothing
      */
void put_(volatile myQueue * Queue, const char * arr)
{
     char c;
     uint16_t i=0;
     uint16_t fifo_windex = Queue->windex;
     while( (c = arr[i++]) != '\0' )
     {      
            Queue->fifo[fifo_windex] = c;
            fifo_windex = (fifo_windex + 1) % FIFO_SIZE;
            //Queue->windex = (Queue->windex +1) % FIFO_SIZE;
     }
     Queue->fifo[fifo_windex] = '\0';
     fifo_windex = (fifo_windex + 1) % FIFO_SIZE;
     Queue->windex = fifo_windex;
     
     (Queue->semaphore)++;
}

/** @brief Get string from queue
      * @param my_queue* Queue - pointer to queue to get string from
      * @param char* arr - array to get filled by queue 
      * @return nothing (indirecty array containing popped string)
      */
void get_(volatile myQueue * Queue, char * arr)
{
     char c;
     uint16_t i=0;
     uint16_t fifo_rindex = Queue->rindex;
     while( (c=(Queue->fifo[fifo_rindex])) != '\0')
     {
            arr[i++] = c;
            fifo_rindex = (fifo_rindex + 1) % FIFO_SIZE;
     }
     arr[i] = '\0';
     fifo_rindex = (fifo_rindex + 1) % FIFO_SIZE;
     Queue->rindex = fifo_rindex;
     
     (Queue->semaphore)--;
}


/** @brief Initiates queue assigning default values
      * @param my_queue* qq - pointer to my_queue structure to initialize
      * @return nothing
      */
void myQueueInit(volatile myQueue * qq)
{
     qq->put = put_;
     qq->get = get_;
     qq->trylock = trylock_;
     qq->release = release_;
     qq->semaphore = 0;
     qq->mutex = UNLOCKED;
     qq->windex = 0;
     qq->rindex = 0;
     qq->old_primask = 0;
}
