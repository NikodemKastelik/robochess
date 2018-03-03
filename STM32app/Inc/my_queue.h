/*
 * my_queue.h
 *
 * Created: 2017-12-20 20:22:50
 *  Author: Nikodem Kastelik
 *
 * @brief
 *
 * Implements basic concurrent queue. Concurrency is maintained by mutex locking
 * mechanism. This operation is atomic with additional workaround with storing
 * previous PREMASK value (global interrupt flag) and then disabling interrupts.
 *
 * @changelog
 * ver 1.1:
 * - properly restore PRIMASK after lock
 * - saving old PRIMASK in queue structure
 * ver 1.0:
 * - created library
 *
 * @TODO
 * - raise some error on fifo-circural buffer overflow (when w(rite)index 
 *   catches r(ead)index)
 * 
 */ 

#ifndef _GUARD_MY_QUEUE
#define _GUARD_MY_QUEUE

#include <inttypes.h>

#define FIFO_SIZE		1024
#define ELEMENT_SIZE	64

#define LOCKED     1
#define UNLOCKED   0

#define TRUE       1
#define FALSE      0


/** @brief Struct containing queue fields. It tries to mimic OOP model. 
      * @field *put - contains pointer to function performing pushing string to queue
      * @field *get - contains pointer to function performing poping string from queue
      * @field *lock - contains pointer to function performing blocking-lock on mutex
      *              NOT IMPLEMENTED - would make sense to use this with preemptive
      *              scheduler, due to fact that now it would lead to deadlock if
      *              used in interrupt routine
      * @field *trylock - contains pointer to function performing nonblocking
      *                 lock on mutex. If mutex is already locked it returns
      *                 LOCK_FAILURE and caller can react accordingly
      * @field *release - releases mutex
      * @field fifo - array containing queue's strings
      * @field semaphore - indicates how many strings are there pushed to queue
      * @field mutex - queue mutex. Has to be acquired before 'put' or 'get' 
      *              operations.
      * @field windex - (write index) - index of writing cursor
      * @field rindex - (read index) - index of reading cursor
      * #field old_primask - PRIMASK before lock. Used to restore PRIMASK register
      */
      
typedef struct my_queue {
   //public
   void (*put)(volatile struct my_queue*, const char*);
   void (*get)(volatile struct my_queue*, char*);
   //void (*lock)(my_queue*) = &lock;
   uint8_t (*trylock)(volatile struct my_queue*);
   void (*release)(volatile struct my_queue*);
   uint8_t semaphore;
   
   //private
   char fifo[FIFO_SIZE];
   uint8_t mutex;
   uint16_t windex;
   uint16_t rindex;
   uint32_t old_primask;
} myQueue;

void myQueueInit(volatile myQueue * msgQueue);

#endif
