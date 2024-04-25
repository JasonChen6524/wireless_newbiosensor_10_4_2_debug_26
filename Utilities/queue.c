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

/*
 * TODO:
 * Add a function to enqueue data block instead of one by one.
 * Write function definitions in the header file as doxygen format
 * Init function will also allocate memory for queue buffer, providing the buffer will not necessary
 *
 * */
#include "app.h"
#include "cmsis_compiler.h"

int queue_reset(struct queue_t *q)
{
	if (!q)
		return E_INVALID;

	__disable_irq();
	q->wr = q->base;
	q->rd = q->base;
	q->num_item = 0;
	q->ovf_item = 0;
	__enable_irq();
	return 0;
}

int queue_len(struct queue_t *q)
{
	int num_elements;

	if (!q)
		return E_INVALID;

	__disable_irq();
	num_elements = q->num_item;
	__enable_irq();

	return num_elements;
}


int queue_init(struct queue_t *q, void *buf, int item_size, int buffer_size)
{
	if (!q || !buf)
		return E_INVALID;

	if (buffer_size % item_size != 0)
		return E_INVALID; // Padding problem

	__disable_irq();
	q->num_item = 0;
	q->ovf_item = 0;
	q->base = buf;
	q->wr = buf;
	q->rd = buf;
	q->item_size = item_size;
	q->buffer_size = buffer_size;
	__enable_irq();
	return 0;
}

void queue_destroy(struct queue_t *q)
{
  (void)q;
/* TODO: This is placeholder function, double check the implementation */
	//free((void *)q->base);
	//free((void *)q);
}

int enqueue(struct queue_t *q, void *data)
{
	int ret = 0;

	if (!q || !data)
		return E_INVALID; // Invalid pointer

	__disable_irq();
	if (q->wr == q->rd)
		ret = (q->num_item != 0) ? -2 : 0; // Is FIFO Full or Empty?

	if (((uint32_t)q->wr) >= ((uint32_t)q->base + q->buffer_size))
		q->wr = q->base;

	memcpy((void *)q->wr, data, q->item_size);
	q->wr = (void *)((uint32_t)q->wr + q->item_size);
	q->num_item++;
	__enable_irq();
	return ret;
}

int dequeue(struct queue_t *q, void *data)
{
	int fifo_size = q->buffer_size / q->item_size;

	if (!q || !data)
		return E_INVALID;

	__disable_irq();
	if (q->num_item <= 0) {
		__enable_irq();
		return -2;
	}

	if (q->num_item > fifo_size) {
		uint32_t curr_rd_off = (((uint32_t)q->rd - (uint32_t)q->base) + q->num_item * q->item_size);
		q->ovf_item = q->num_item - fifo_size;
		q->rd = (void *)((uint32_t)q->base + (curr_rd_off % q->buffer_size));
		q->num_item = fifo_size; // OVF number samples are already gone.
		pr_info("%s:%d - %d samples lost, avail:%d \n",
				__func__, __LINE__, q->ovf_item, q->num_item);
	} else
		q->ovf_item = 0;

	if (((uint32_t)q->rd) >= ((uint32_t)q->base + q->buffer_size))
		q->rd = q->base;

	memcpy(data, (void *)q->rd, q->item_size);
	q->rd = (void *)((uint32_t)q->rd + q->item_size);
	q->num_item--;
	__enable_irq();

#if defined(QUEUE_DEBUG)
	do {
		static int cnt;

		if (cnt++ % 100 == 0)
			pr_debug("$ Fifo size: %d, usage: %d\n", fifo_size, q->num_item);
	} while(0);
#endif

	return 0;
}

int queue_usage(struct queue_t *q, int *total, int *nm_item)
{
	if (!q)
		return E_INVALID;

	*total = q->buffer_size / q->item_size;
	*nm_item = q->num_item;

	return 0;
}

int queue_pop(struct queue_t *q)
{
	int fifo_size = q->buffer_size / q->item_size;

	if (!q)
		return E_INVALID;

	__disable_irq();
	if (q->num_item <= 0) {
		__enable_irq();
		return -2;
	}

	if (q->num_item > fifo_size) {
		uint32_t curr_rd_off = (((uint32_t)q->rd - (uint32_t)q->base) + q->num_item * q->item_size);
		q->ovf_item = q->num_item - fifo_size;
		q->rd = (void *)((uint32_t)q->base + (curr_rd_off % q->buffer_size));
		q->num_item = fifo_size; // OVF number samples are already gone.
		app_log_debug("%s:%d - %d samples lost, avail:%d \n",
				__func__, __LINE__, q->ovf_item, q->num_item);
	} else
		q->ovf_item = 0;

	if (((uint32_t)q->rd) >= ((uint32_t)q->base + q->buffer_size))
		q->rd = q->base;

	q->rd = (void *)((uint32_t)q->rd + q->item_size);
	q->num_item--;
	__enable_irq();

#if defined(QUEUE_DEBUG)
	do {
		static int cnt;

		if (cnt++ % 100 == 0)
			pr_debug("$ Fifo size: %d, usage: %d\n", fifo_size, q->num_item);
	} while(0);
#endif

	return 0;
}

int queue_front(struct queue_t *q, void *data)
{
	int fifo_size = q->buffer_size / q->item_size;
	void *rd = 0;

	if (!q || !data)
		return E_INVALID;

	__disable_irq();
	if (q->num_item <= 0) {
		__enable_irq();
		return -2;
	}

	if (q->num_item > fifo_size) {
		uint32_t curr_rd_off = (((uint32_t)q->rd - (uint32_t)q->base) + q->num_item * q->item_size);
		rd = (void *)((uint32_t)q->base + (curr_rd_off % q->buffer_size));
		app_log_debug("%s:%d - %d samples lost, avail:%d \n",
				__func__, __LINE__, q->ovf_item, q->num_item);
	} else {
		q->ovf_item = 0;
		rd = q->rd;
	}

	if (((uint32_t)q->rd) >= ((uint32_t)q->base + q->buffer_size))
		rd = q->base;

	memcpy(data, (void *)rd, q->item_size);
	__enable_irq();

	return 0;
}

int enqueue_string(struct queue_t *q, char *data, int sz)
{
	int ret = 0;
	int buf_index;
	char *wr_ptr;

	if (!q || !data || sz <= 0)
		return E_UNKNOWN; // Invalid parameters

	__disable_irq();
	if (q->wr == q->rd)
		ret = (q->num_item != 0) ? -2 : 0; // Is FIFO Full or Empty?

	if (((uint32_t)q->wr) >= ((uint32_t)q->base + q->buffer_size))
		q->wr = q->base;

	if ((q->num_item + sz) > q->buffer_size) {
		__enable_irq();
#if defined(QUEUE_DEBUG)
		{
			char buf[128];
			int len;
			len = sprintf(buf, "\r\n**** %s - Fifo is full. num_item: %d, sz: %d, buffer size: %d\r\n",
					__func__, q->num_item, sz, q->buffer_size);
			UART_Write(UART_PORT, (uint8_t*)buf, len);
		}
#endif
		return E_UNKNOWN;
	}

	buf_index = (uint32_t)q->wr - (uint32_t)q->base;
	wr_ptr = (char *)q->base;
	q->num_item += sz;
	while(sz--)
		wr_ptr[buf_index++ % q->buffer_size] = *data++;

	q->wr = (void *)((uint32_t)q->base + buf_index % q->buffer_size);
	__enable_irq();
	return ret;
}

int dequeue_string(struct queue_t *q, char *buf, char delimiter, int buffer_size)
{
	char *rd_ptr;
	int buf_index;
	int len;

	if (!q || !buf || buffer_size <= 0)
		return E_UNKNOWN;

	__disable_irq();
	if (q->num_item <= 0) {
		__enable_irq();
		return E_UNKNOWN;
	}

	rd_ptr = (char *)q->base;
	buf_index = (uint32_t)q->rd - (uint32_t)q->base;
	len = q->num_item;

	while (buffer_size-- && q->num_item--) {
		char tmp = rd_ptr[buf_index % q->buffer_size];
		rd_ptr[buf_index % q->buffer_size] = 0; // Remove this later on
		buf_index++;
		*buf++ = tmp;
		if (tmp == delimiter)
			break;
	}

	if (q->num_item < 0) {
		/* Data corruption in FIFO */
		q->num_item = 0;
	} else
		len -= q->num_item;

	q->rd = (void *)((uint32_t)q->base + buf_index % q->buffer_size);
	__enable_irq();

	return len;
}



#if 0
void queue_test(void)
{
	int ret;
	ppg_data_t ppg_test = { 0, };
	ppg_data_t ppg_test_out = { 0, };
	int i, j, ii, jj;
	static ppg_data_t ppg_data[10];
	static queue_t queue;

	srand((unsigned)time(NULL));
	ret = queue_init(&queue, &ppg_data, sizeof(ppg_data_t), sizeof(ppg_data));
	while (1) {
		ii = rand() % 20;
		for (i = 0; i < ii; i++) {
			/* Test data */
			ppg_test.timestamp++;
			ppg_test.ir++;
			ppg_test.red++;
			ppg_test.green++;
			/* Test functions */
			ret = enqueue(&queue, &ppg_test);
		}
		jj = rand() % 20;
		for (j = 0; j < jj; j++) {
			ret = dequeue(&queue, &ppg_test_out);
		}
	}
}
#endif
