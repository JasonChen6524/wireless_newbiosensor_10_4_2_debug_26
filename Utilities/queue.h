/*******************************************************************************
* Author: Ismail Kose, Ismail.Kose@maximintegrated.com
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#ifndef _QUEUE_H_
#define _QUEUE_H_

#define E_INVALID                 (-1)
#define E_UNKNOWN                 (-2)
//#define __disable_irq()
//#define __enable_irq()

struct queue_t {
	void *wr; // write pointer
	void *rd; // read pointer
	void *base; // buffer base pointer
	int num_item; // number of data item
	int ovf_item; // Number of overflowed data
	int buffer_size; // buffer size in bytes
	int item_size; // data size
};

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Queue initialization.
 * @param[in]       *q Points to the queue handle
 * @param[in]       *buf Points to external queue buffer
 * @param[in]       item_size Data size
 * @param[in]       buffer_size Total buffer size in bytes
 * @param[out]      *pDst points to output matrix structure
 * @return     The function returns 0: success
 *								-EINVAL (-22): Invalid Pointer, data or parameters
 *								-2: Queue buffer is full, no more space
 **/
int queue_init(struct queue_t *q, void *buf, int item_size, int buffer_size);




/**
 * @brief Data reset.
 * @param[in]       *q Points to the queue handle
 * @param[in]       *data Points to any type of data to put FIFO
 * @param[out]      *pDst Points to output matrix structure
 * @return     The function returns 0: success
 *								-EINVAL (-22): Invalid Pointer
 *								-2: Queue buffer is full, no more space
 **/
int queue_reset(struct queue_t *q);


/**
 * @brief Data enqueue.
 * @param[in]       *q points to the queue handle
 * @param[in]       *data points to any type of data to put FIFO
 * @return     The function returns 0: success
 *								-EINVAL (-22): Invalid Pointer
 *								-2: Queue buffer is full, no more space
 **/
int enqueue(struct queue_t *q, void *data);


/**
 * @brief Data dequeue.
 * @param[in]       *q points to the queue handle
 * @param[in]       *data points to any type of data to put FIFO
 * @param[out]      *data pop data from Queue
 * @return     The function returns 0: success
 *								-EINVAL (-22): Invalid Pointer or data
 *								-2: Queue buffer is empty
 **/
int dequeue(struct queue_t *q, void *data);



/**
 * @brief Queue Destroy
 * @param[in]       *q points to the queue handle
 **/
void queue_destroy(struct queue_t *q);

/**
 * @brief Number of elements in Queue
 * @param[in]       *q points to the queue handle
 * @return		number of elements
 **/
int queue_len(struct queue_t *q);


/*
	Reads the item from the front of queue but does not remove
*/
/**
 * @brief Copies an item from the front of queue to data, but does not remove it from queue
 * @param[in]  *q points to the queue handle
 * @param[out]  Copy of item from front of the queue
 * @return		if value is greater than 0, return value is number of elements.
				If value is less than 0, returns -EINVAL (-22)
 **/

int queue_front(struct queue_t *q, void *data);

/**
 * @brief Removes an item from front of queue
 * @param[in]	*q points to the queue handle
 * @return		status, success or fail
 **/
int queue_pop(struct queue_t *q);

/**
 * @brief		returns fifo usage info
 * @param[in]	*q points to the queue handle
 * @param[out]	*total returns total FIFO size in number of elements
 * @param[out]	*nm_item returns number of elements in FIFO
 * @return		status, success or fail
 *				-EINVAL (-22): Invalid Pointer, data or parameters
 **/
int queue_usage(struct queue_t *q, int *total, int *nm_item);

/**
 * @brief		Pops out delimiter terminated string
 * @param[in]	*q points to the queue handle
 * @param[out]	*buf output char array to write
 * @param[in]	delimiter Delimiter character, NULL for string
 * @param[in]	buffer_size Maximum buffer size to write the output char array
 * @return		status, string length if positive or fail if negative
 *				-EINVAL (-22): Invalid Pointer, data or parameters
 **/
int dequeue_string(struct queue_t *q, char *buf, char delimiter, int buffer_size);

/**
 * @brief		Pushes null terminated string (char array)
 * @param[in]	*q points to the queue handle
 * @param[in]	*data string(char array) to add it to the circullar buffer
 * @param[in]	sz 'data' length
 * @return		status, success or fail
 *				-EINVAL (-22): Invalid Pointer, data or parameters
 **/
int enqueue_string(struct queue_t *q, char *data, int sz);

#ifdef __cplusplus
}
#endif
#endif //_QUEUE_H_
