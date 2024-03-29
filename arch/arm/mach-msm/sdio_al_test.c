/* Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/*
 * SDIO-Abstraction-Layer Test Module.
 *
 */

#include "sdio_al_private.h"

#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/platform_device.h>
#include <mach/sdio_smem.h>
#include <linux/wakelock.h>

#include <linux/debugfs.h>

#include <linux/kthread.h>
enum lpm_test_msg_type {
	LPM_NO_MSG,	/* 0 */
	LPM_MSG_SEND,	/* 1 */
	LPM_MSG_REC,	/* 2 */
	LPM_SLEEP,	/* 3 */
	LPM_WAKEUP,	/* 4 */
	LPM_NOTIFY	/* 5 */
};

#define LPM_NO_MSG_NAME "LPM No Event"
#define LPM_MSG_SEND_NAME "LPM Send Msg Event"
#define LPM_MSG_REC_NAME "LPM Receive Msg Event"
#define LPM_SLEEP_NAME "LPM Sleep Event"
#define LPM_WAKEUP_NAME "LPM Wakeup Event"

/** Module name string */
#define TEST_MODULE_NAME "sdio_al_test"

#define TEST_SIGNATURE 0x12345678
#define TEST_CONFIG_SIGNATURE 0xBEEFCAFE

#define MAX_XFER_SIZE (16*1024)
#define SMEM_MAX_XFER_SIZE 0xBC000
#define A2_MIN_PACKET_SIZE 5
#define DUN_PACKET_SIZE (2*1024)

#define TEST_DBG(x...) if (test_ctx->runtime_debug) pr_info(x)

#define LPM_TEST_NUM_OF_PACKETS 100
#define LPM_MAX_OPEN_CHAN_PER_DEV 4
#define LPM_ARRAY_SIZE	(7*LPM_TEST_NUM_OF_PACKETS*LPM_MAX_OPEN_CHAN_PER_DEV)
#define SDIO_LPM_TEST "sdio_lpm_test_reading_task"
#define LPM_TEST_CONFIG_SIGNATURE 0xDEADBABE
#define LPM_MSG_NAME_SIZE 20

#define A2_HEADER_OVERHEAD 8

enum sdio_test_case_type {
	SDIO_TEST_LOOPBACK_HOST,
	SDIO_TEST_LOOPBACK_CLIENT,
	SDIO_TEST_LPM_HOST_WAKER,
	SDIO_TEST_LPM_CLIENT_WAKER,
	SDIO_TEST_LPM_RANDOM,
	SDIO_TEST_HOST_SENDER_NO_LP,
	SDIO_TEST_CLOSE_CHANNEL,
	/* The following tests are not part of the 9k tests and should be
	 * kept last in case new tests are added
	 */
	SDIO_TEST_PERF,
	SDIO_TEST_RTT,
	SDIO_TEST_MODEM_RESET,
};

struct lpm_task {
	struct task_struct *lpm_task;
	const char *task_name;
};

struct lpm_entry_type {
	enum lpm_test_msg_type msg_type;
	char msg_name[LPM_MSG_NAME_SIZE];
	u32 counter;
	u32 current_ms;
	u32 read_avail_mask;
	char chan_name[CHANNEL_NAME_SIZE];
};

struct lpm_msg {
	u32 signature;
	u32 counter;
	u32 reserve1;
	u32 reserve2;
};

struct test_config_msg {
	u32 signature;
	u32 test_case;
	u32 test_param;
	u32 num_packets;
	u32 num_iterations;
};

struct test_result_msg {
	u32 signature;
	u32 is_successful;
};

struct test_work {
	struct work_struct work;
	struct test_channel *test_ch;
};

enum sdio_channels_ids {
	SDIO_RPC,
	SDIO_QMI,
	SDIO_RMNT,
	SDIO_DIAG,
	SDIO_DUN,
	SDIO_SMEM,
	SDIO_CIQ,
	SDIO_MAX_CHANNELS
};

enum sdio_test_results {
	TEST_NO_RESULT,
	TEST_FAILED,
	TEST_PASSED
};

enum sdio_lpm_vote_state {
	SDIO_NO_VOTE,
	SDIO_VOTE_FOR_SLEEP,
	SDIO_VOTE_AGAINST_SLEEP
};

struct sdio_test_device {
	int open_channels_counter_to_recv;
	int open_channels_counter_to_send;
	struct lpm_entry_type *lpm_arr;
	int array_size;
	void *sdio_al_device;
	spinlock_t lpm_array_lock;
	unsigned long lpm_array_lock_flags;
	u32 next_avail_entry_in_array;
	struct lpm_task lpm_test_task;
	u32 next_mask_id;
	u32 read_avail_mask;
	int modem_result_per_dev;
	int final_result_per_dev;
};

struct test_channel {
	struct sdio_channel *ch;

	char name[CHANNEL_NAME_SIZE];
	int ch_id;

	struct sdio_test_device *test_device;

	u32 *buf;
	u32 buf_size;

	struct workqueue_struct *workqueue;
	struct test_work test_work;

	u32 rx_bytes;
	u32 tx_bytes;

	wait_queue_head_t   wait_q;
	atomic_t rx_notify_count;
	atomic_t tx_notify_count;
	atomic_t any_notify_count;
	atomic_t wakeup_client;
	atomic_t card_detected_event;

	int wait_counter;

	int is_used;
	int test_type;
	int ch_ready;

	struct test_config_msg config_msg;

	int test_completed;
	int test_result;
	struct timer_list timer;
	int timer_interval_ms;

	struct timer_list timeout_timer;
	int timeout_ms;
	void *sdio_al_device;
	int is_ok_to_sleep;
	unsigned int packet_length;
	int random_packet_size;
	int next_index_in_sent_msg_per_chan;
	int channel_mask_id;
	int modem_result_per_chan;
	int notify_counter_per_chan;
	int max_burst_size;        /* number of writes before close/open */
	int card_removed;
};

struct sdio_al_test_debug {
	u32 dun_throughput;
	u32 rmnt_throughput;
	struct dentry *debug_root;
	struct dentry *debug_test_result;
	struct dentry *debug_dun_throughput;
	struct dentry *debug_rmnt_throughput;
};

struct test_context {
	dev_t dev_num;
	struct device *dev;
	struct cdev *cdev;
	int number_of_active_devices;
	int max_number_of_devices;

	struct sdio_test_device test_dev_arr[MAX_NUM_OF_SDIO_DEVICES];

	struct test_channel *test_ch;

	struct test_channel *test_ch_arr[SDIO_MAX_CHANNELS];

	long testcase;

	const char *name;

	int exit_flag;

	u32 signature;

	int runtime_debug;

	struct platform_device smem_pdev;
	struct sdio_smem_client *sdio_smem;
	int smem_was_init;
	u8 *smem_buf;

	wait_queue_head_t   wait_q;
	int test_completed;
	int test_result;
	struct sdio_al_test_debug debug;

	struct wake_lock wake_lock;

	unsigned int lpm_pseudo_random_seed;
};

/*
 * Seed for pseudo random time sleeping in Random LPM test.
 * If not set, current time in jiffies is used.
 */
static unsigned int seed;
module_param(seed, int, 0);
static struct test_context *test_ctx;

#ifdef CONFIG_DEBUG_FS
/*
*
* Trigger on/off for debug messages
* for trigger off the data messages debug level use:
* echo 0 > /sys/kernel/debugfs/sdio_al/debug_data_on
* for trigger on the data messages debug level use:
* echo 1 > /sys/kernel/debugfs/sdio_al/debug_data_on
* for trigger off the lpm messages debug level use:
* echo 0 > /sys/kernel/debugfs/sdio_al/debug_lpm_on
* for trigger on the lpm messages debug level use:
* echo 1 > /sys/kernel/debugfs/sdio_al/debug_lpm_on
*/
static int sdio_al_test_debugfs_init(void)
{
	test_ctx->debug.debug_root = debugfs_create_dir("sdio_al_test",
							       NULL);
	if (!test_ctx->debug.debug_root)
		return -ENOENT;

	test_ctx->debug.debug_test_result = debugfs_create_u32(
					"test_result",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->test_result);

	test_ctx->debug.debug_dun_throughput = debugfs_create_u32(
					"dun_throughput",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->debug.dun_throughput);

	test_ctx->debug.debug_rmnt_throughput = debugfs_create_u32(
					"rmnt_throughput",
					S_IRUGO | S_IWUGO,
					test_ctx->debug.debug_root,
					&test_ctx->debug.rmnt_throughput);

	if ((!test_ctx->debug.debug_dun_throughput) &&
	    (!test_ctx->debug.debug_rmnt_throughput)) {
		debugfs_remove_recursive(test_ctx->debug.debug_root);
		test_ctx->debug.debug_root = NULL;
		return -ENOENT;
	}
	return 0;
}

static void sdio_al_test_debugfs_cleanup(void)
{
       debugfs_remove(test_ctx->debug.debug_dun_throughput);
       debugfs_remove(test_ctx->debug.debug_rmnt_throughput);
       debugfs_remove(test_ctx->debug.debug_root);
}
#endif

static int channel_name_to_id(char *name)
{
	pr_info(TEST_MODULE_NAME "%s: channel name %s\n",
		__func__, name);

	if (!strncmp(name, "SDIO_RPC_TEST",
		     strnlen("SDIO_RPC_TEST", CHANNEL_NAME_SIZE)))
		return SDIO_RPC;
	else if (!strncmp(name, "SDIO_QMI_TEST",
			  strnlen("SDIO_QMI_TEST", TEST_CH_NAME_SIZE)))
		return SDIO_QMI;
	else if (!strncmp(name, "SDIO_RMNT_TEST",
			  strnlen("SDIO_RMNT_TEST", TEST_CH_NAME_SIZE)))
		return SDIO_RMNT;
	else if (!strncmp(name, "SDIO_DIAG_TEST",
			  strnlen("SDIO_DIAG", TEST_CH_NAME_SIZE)))
		return SDIO_DIAG;
	else if (!strncmp(name, "SDIO_DUN_TEST",
			  strnlen("SDIO_DUN_TEST", TEST_CH_NAME_SIZE)))
		return SDIO_DUN;
	else if (!strncmp(name, "SDIO_SMEM_TEST",
			  strnlen("SDIO_SMEM_TEST", TEST_CH_NAME_SIZE)))
		return SDIO_SMEM;
	else if (!strncmp(name, "SDIO_CIQ_TEST",
			  strnlen("SDIO_CIQ_TEST", TEST_CH_NAME_SIZE)))
		return SDIO_CIQ;
	else
		return SDIO_MAX_CHANNELS;

	return SDIO_MAX_CHANNELS;
}

/**
 * Config message
 */

static void send_config_msg(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 write_avail = 0;
	int size = sizeof(test_ch->config_msg);

	pr_debug(TEST_MODULE_NAME "%s\n", __func__);

	memcpy(test_ch->buf, (void *)&test_ch->config_msg, size);

	if (test_ctx->exit_flag) {
		pr_info(TEST_MODULE_NAME ":Exit Test.\n");
		return;
	}

	pr_info(TEST_MODULE_NAME ":Sending the config message.\n");

	/* wait for data ready event */
	write_avail = sdio_write_avail(test_ch->ch);
	pr_debug(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
	if (write_avail < size) {
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->tx_notify_count));
		atomic_dec(&test_ch->tx_notify_count);
	}

	write_avail = sdio_write_avail(test_ch->ch);
	pr_debug(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
	if (write_avail < size) {
		pr_info(TEST_MODULE_NAME ":not enough write avail.\n");
		return;
	}

	ret = sdio_write(test_ch->ch, test_ch->buf, size);
	if (ret)
		pr_err(TEST_MODULE_NAME ":%s sdio_write err=%d.\n",
			__func__, -ret);
	else
		pr_info(TEST_MODULE_NAME ":%s sent config_msg successfully.\n",
		       __func__);
}

/**
 * Loopback Test
 */
static void loopback_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;

	while (1) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		TEST_DBG(TEST_MODULE_NAME "--LOOPBACK WAIT FOR EVENT--.\n");
		/* wait for data ready event */
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->rx_notify_count));
		atomic_dec(&test_ch->rx_notify_count);

		read_avail = sdio_read_avail(test_ch->ch);
		if (read_avail == 0)
			continue;


		write_avail = sdio_write_avail(test_ch->ch);
		if (write_avail < read_avail) {
			pr_info(TEST_MODULE_NAME
				":not enough write avail.\n");
			continue;
		}

		ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME
			       ":worker, sdio_read err=%d.\n", -ret);
			continue;
		}
		test_ch->rx_bytes += read_avail;

		TEST_DBG(TEST_MODULE_NAME ":worker total rx bytes = 0x%x.\n",
			 test_ch->rx_bytes);


		ret = sdio_write(test_ch->ch,
				 test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME
				":loopback sdio_write err=%d.\n",
				-ret);
			continue;
		}
		test_ch->tx_bytes += read_avail;

		TEST_DBG(TEST_MODULE_NAME
			 ":loopback total tx bytes = 0x%x.\n",
			 test_ch->tx_bytes);
	} /* end of while */
}

/**
 * Check if all tests completed
 */
static void check_test_completion(void)
{
	int i;

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready))
			continue;
		if (!tch->test_completed) {
			pr_info(TEST_MODULE_NAME ": %s - Channel %s test is "
				"not completed", __func__, tch->name);
			return;
		}
	}
	pr_info(TEST_MODULE_NAME ": %s - Test is completed", __func__);
	test_ctx->test_completed = 1;
	wake_up(&test_ctx->wait_q);
}

static int pseudo_random_seed(unsigned int *seed_number)
{
	if (!seed_number)
		return 0;

	*seed_number = (unsigned int)(((unsigned long)*seed_number *
				(unsigned long)1103515367) + 35757);
	return (int)((*seed_number / (64*1024)) % 500);
}

/* this function must be locked before accessing it */
static void lpm_test_update_entry(struct test_channel *tch,
				  enum lpm_test_msg_type msg_type,
				   char *msg_name,
				  int counter)
{
	u32 index = 0;
	static int print_full = 1;
	struct sdio_test_device *test_device;

	if (!tch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test channel\n", __func__);
		return;
	}

	test_device = tch->test_device;

	if (!test_device) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test device\n", __func__);
		return;
	}

	if (!test_device->lpm_arr) {
		pr_err(TEST_MODULE_NAME ": %s - NULL lpm_arr\n", __func__);
		return;
	}

	if (test_device->next_avail_entry_in_array >=
					test_device->array_size) {
		pr_err(TEST_MODULE_NAME ": %s - lpm array is full",
			__func__);

		if (print_full) {
			print_hex_dump(KERN_INFO, TEST_MODULE_NAME ": lpm_arr:",
				0, 32, 2,
				(void *)test_device->lpm_arr,
				sizeof(test_device->lpm_arr), false);
			print_full = 0;
		}
		return;
	}

	index = test_device->next_avail_entry_in_array;
	if ((msg_type == LPM_MSG_SEND) || (msg_type == LPM_MSG_REC))
		test_device->lpm_arr[index].counter = counter;
	else
		test_device->lpm_arr[index].counter = 0;

	test_device->lpm_arr[index].msg_type = msg_type;
	memcpy(test_device->lpm_arr[index].msg_name, msg_name,
	       LPM_MSG_NAME_SIZE);
	test_device->lpm_arr[index].current_ms =
		jiffies_to_msecs(get_jiffies_64());

	test_device->lpm_arr[index].read_avail_mask =
		test_device->read_avail_mask;

	if ((msg_type == LPM_SLEEP) || (msg_type == LPM_WAKEUP))
		memcpy(test_device->lpm_arr[index].chan_name, "DEVICE  ",
		       CHANNEL_NAME_SIZE);
	else
		memcpy(test_device->lpm_arr[index].chan_name, tch->name,
		       CHANNEL_NAME_SIZE);

	test_device->next_avail_entry_in_array++;
}

static int wait_for_result_msg(struct test_channel *test_ch)
{
	u32 read_avail = 0;
	int ret = 0;

	pr_info(TEST_MODULE_NAME ": %s - START, channel %s\n",
		__func__, test_ch->name);

	while (1) {
		read_avail = sdio_read_avail(test_ch->ch);

		if (read_avail == 0) {
			pr_info(TEST_MODULE_NAME
				": read_avail is 0 for chan %s\n",
				test_ch->name);
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->rx_notify_count));
			atomic_dec(&test_ch->rx_notify_count);
			continue;
		}

		memset(test_ch->buf, 0x00, test_ch->buf_size);

		ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":  sdio_read for chan"
				"%s failed, err=%d.\n",
				test_ch->name, -ret);
			goto exit_err;
		}

		if (test_ch->buf[0] != TEST_CONFIG_SIGNATURE) {
			pr_info(TEST_MODULE_NAME ": Not a test_result "
				"signature. expected 0x%x. received 0x%x "
				"for chan %s\n",
				TEST_CONFIG_SIGNATURE,
				test_ch->buf[0],
				test_ch->name);
			continue;
		} else {
			pr_info(TEST_MODULE_NAME ": Signature is "
				"TEST_CONFIG_SIGNATURE as expected\n");
			break;
		}
	}

	return test_ch->buf[1];

exit_err:
	return 0;
}

static int check_random_lpm_test_array(struct sdio_test_device *test_dev)
{
	int i = 0, j = 0;
	unsigned int delta_ms = 0;
	int arr_ind = 0;
	int ret = 1;
	int notify_counter = 0;
	int sleep_counter = 0;
	int wakeup_counter = 0;
	int lpm_activity_counter = 0;

	if (!test_dev) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test device\n", __func__);
		return -ENODEV;
	}

	for (i = 0 ; i < test_dev->next_avail_entry_in_array ; ++i) {
		if (i == 0)
			pr_err(TEST_MODULE_NAME ": index %4d, chan=%2s, "
			       "code=%1d=%4s, msg#%1d, ms from before=-1, "
			       "read_mask=0x%d, ms=%2u",
			       i,
			       test_dev->lpm_arr[i].chan_name,
			       test_dev->lpm_arr[i].msg_type,
			       test_dev->lpm_arr[i].msg_name,
			       test_dev->lpm_arr[i].counter,
			       test_dev->lpm_arr[i].read_avail_mask,
			       test_dev->lpm_arr[i].current_ms);
		else
			pr_err(TEST_MODULE_NAME ": index "
			       "%4d, %2s, code=%1d=%4s, msg#%1d, ms from "
			       "before=%2u, read_mask=0x%d, ms=%2u",
			       i,
			       test_dev->lpm_arr[i].chan_name,
			       test_dev->lpm_arr[i].msg_type,
			       test_dev->lpm_arr[i].msg_name,
			       test_dev->lpm_arr[i].counter,
			       test_dev->lpm_arr[i].current_ms -
			       test_dev->lpm_arr[i-1].current_ms,
			       test_dev->lpm_arr[i].read_avail_mask,
			       test_dev->lpm_arr[i].current_ms);
	}

	for (i = 0; i < test_dev->next_avail_entry_in_array; i++) {
		notify_counter = 0;
		sleep_counter = 0;
		wakeup_counter = 0;

		if ((test_dev->lpm_arr[i].msg_type == LPM_MSG_SEND) ||
		     (test_dev->lpm_arr[i].msg_type == LPM_MSG_REC)) {
			/* find the next message in the array */
			arr_ind = test_dev->next_avail_entry_in_array;
			for (j = i+1; j < arr_ind; j++) {
				if ((test_dev->lpm_arr[j].msg_type ==
				     LPM_MSG_SEND) ||
				    (test_dev->lpm_arr[j].msg_type ==
				     LPM_MSG_REC) ||
				    (test_dev->lpm_arr[j].msg_type ==
				     LPM_NOTIFY))
					break;
				if (test_dev->lpm_arr[j].msg_type ==
				    LPM_SLEEP)
					sleep_counter++;
				if (test_dev->lpm_arr[j].msg_type ==
				    LPM_WAKEUP)
					wakeup_counter++;
			}
			if (j == arr_ind) {
				ret = 1;
				break;
			}

			delta_ms = test_dev->lpm_arr[j].current_ms -
				test_dev->lpm_arr[i].current_ms;
			if (delta_ms < 30) {
				if ((sleep_counter == 0)
				    && (wakeup_counter == 0)) {
					continue;
				} else {
					pr_err(TEST_MODULE_NAME "%s: lpm "
						"activity while delta is less "
						"than 30, i=%d, j=%d, "
						"sleep_counter=%d, "
						"wakeup_counter=%d",
					       __func__, i, j,
					       sleep_counter, wakeup_counter);
					ret = 0;
					break;
				}
			} else {
				if ((delta_ms > 90) &&
				    (test_dev->lpm_arr[i].
						read_avail_mask == 0)) {
					if (j != i+3) {
						pr_err(TEST_MODULE_NAME
						       "%s: unexpected "
						       "lpm activity "
						       "while delta is "
						       "bigger than "
						       "90, i=%d, "
						       "j=%d, "
						       "notify_counter"
						       "=%d",
						       __func__, i, j,
						       notify_counter);
						ret = 0;
						break;
					}
					lpm_activity_counter++;
				}
			}
		}
	}

	pr_info(TEST_MODULE_NAME ": %s - lpm_activity_counter=%d",
		__func__, lpm_activity_counter);

	return ret;
}

static int lpm_test_main_task(void *ptr)
{
	u32 read_avail = 0;
	int last_msg_index = 0;
	struct test_channel *test_ch = (struct test_channel *)ptr;
	struct sdio_test_device *test_dev;
	struct lpm_msg lpm_msg;
	int ret = 0;
	int host_result = 0;

	if (!test_ch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL channel\n", __func__);
		return -ENODEV;
	}

	pr_err(TEST_MODULE_NAME ": %s - STARTED. channel %s\n",
	       __func__, test_ch->name);

	test_dev = test_ch->test_device;

	if (!test_dev) {
		pr_err(TEST_MODULE_NAME ": %s - NULL Test Device\n", __func__);
		return -ENODEV;
	}

	while (last_msg_index < test_ch->config_msg.num_packets - 1) {

		TEST_DBG(TEST_MODULE_NAME ": %s - "
			"IN LOOP last_msg_index=%d\n",
		       __func__, last_msg_index);

		read_avail = sdio_read_avail(test_ch->ch);
		if (read_avail == 0) {
			TEST_DBG(TEST_MODULE_NAME
					":read_avail 0 for chan %s, "
					"wait for event\n",
					test_ch->name);
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->rx_notify_count));
			atomic_dec(&test_ch->rx_notify_count);

			read_avail = sdio_read_avail(test_ch->ch);
			if (read_avail == 0) {
				pr_err(TEST_MODULE_NAME
					":read_avail size %d for chan %s not as"
					" expected\n",
					read_avail, test_ch->name);
				continue;
			}
		}

		memset(test_ch->buf, 0x00, sizeof(test_ch->buf));

		ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sdio_read for chan %s"
				" err=%d.\n", test_ch->name, -ret);
			goto exit_err;
		}

		memcpy((void *)&lpm_msg, test_ch->buf, sizeof(lpm_msg));

		/*
		 * when reading from channel, we want to turn off the bit
		 * mask that implies that there is pending data on that channel
		 */
		if (test_ch->test_device != NULL) {
			spin_lock_irqsave(&test_dev->lpm_array_lock,
					  test_dev->lpm_array_lock_flags);

			test_ch->notify_counter_per_chan--;

			/*
			 * if the channel has no pending data, turn off the
			 * pending data bit mask of the channel
			 */
			if (test_ch->notify_counter_per_chan == 0) {
				test_ch->test_device->read_avail_mask =
					test_ch->test_device->read_avail_mask &
					~test_ch->channel_mask_id;
			}

			last_msg_index = lpm_msg.counter;
			lpm_test_update_entry(test_ch,
					      LPM_MSG_REC,
					      "RECEIVE",
					      last_msg_index);

			spin_unlock_irqrestore(&test_dev->lpm_array_lock,
					       test_dev->lpm_array_lock_flags);
		}
	}

	pr_info(TEST_MODULE_NAME ":%s: Finished to recieve all (%d) "
		"packets from the modem %s. Waiting for result_msg",
		__func__, test_ch->config_msg.num_packets, test_ch->name);

	/* Wait for the resault message from the modem */
	test_ch->modem_result_per_chan = wait_for_result_msg(test_ch);

	/*
	 * the DEVICE modem result is a failure if one of the channels on
	 * that device, got modem_result = 0. this is why we bitwise "AND" each
	 * time another channel completes its task
	 */
	test_dev->modem_result_per_dev &= test_ch->modem_result_per_chan;

	/*
	 * when reading from channel, we want to turn off the bit
	 * mask that implies that there is pending data on that channel
	 */
	spin_lock_irqsave(&test_dev->lpm_array_lock,
					  test_dev->lpm_array_lock_flags);

	test_dev->open_channels_counter_to_recv--;

	/* turning off the read_avail bit of the channel */
	test_ch->test_device->read_avail_mask =
		test_ch->test_device->read_avail_mask &
		~test_ch->channel_mask_id;

	spin_unlock_irqrestore(&test_dev->lpm_array_lock,
					       test_dev->lpm_array_lock_flags);

	/* Wait for all the packets to be sent to the modem */
	while (1) {
		spin_lock_irqsave(&test_dev->lpm_array_lock,
				  test_dev->lpm_array_lock_flags);

		if (test_ch->next_index_in_sent_msg_per_chan >=
		    test_ch->config_msg.num_packets - 1) {

			spin_unlock_irqrestore(&test_dev->lpm_array_lock,
					       test_dev->lpm_array_lock_flags);
			break;
		} else {
			pr_info(TEST_MODULE_NAME ":%s: Didn't finished to send "
				"all packets, "
				"next_index_in_sent_msg_per_chan = %d ",
				__func__,
				test_ch->next_index_in_sent_msg_per_chan);
		}
		spin_unlock_irqrestore(&test_dev->lpm_array_lock,
				       test_dev->lpm_array_lock_flags);
		msleep(60);
	}

	if (test_dev->open_channels_counter_to_recv != 0 ||
	    test_dev->open_channels_counter_to_send != 0) {
		test_ch->test_completed = 1;
		test_ch->next_index_in_sent_msg_per_chan = 0;
		return 0;
	} else {
		test_ctx->number_of_active_devices--;
		sdio_al_unregister_lpm_cb(test_ch->sdio_al_device);

		if (test_ch->test_type == SDIO_TEST_LPM_RANDOM)
			host_result = check_random_lpm_test_array(test_dev);

		pr_info(TEST_MODULE_NAME ": %s - host_result=%d. "
			"device_modem_result=%d",
			__func__, host_result, test_dev->modem_result_per_dev);

		test_ch->test_completed = 1;
		if (test_dev->modem_result_per_dev && host_result) {
			pr_info(TEST_MODULE_NAME ": %s - Random LPM "
				"TEST_PASSED for device %d of %d\n",
				__func__,
				(test_ctx->max_number_of_devices-
				test_ctx->number_of_active_devices),
				test_ctx->max_number_of_devices);
			test_dev->final_result_per_dev = 1; /* PASSED */
		} else {
			pr_info(TEST_MODULE_NAME ": %s - Random LPM "
				"TEST_FAILED for device %d of %d\n",
				__func__,
				(test_ctx->max_number_of_devices-
				test_ctx->number_of_active_devices),
				test_ctx->max_number_of_devices);
			test_dev->final_result_per_dev = 0; /* FAILED */
		}

		test_dev->next_avail_entry_in_array = 0;
		test_ch->next_index_in_sent_msg_per_chan = 0;

		check_test_completion();

		kfree(test_ch->test_device->lpm_arr);

		return 0;
	}

exit_err:
	pr_info(TEST_MODULE_NAME ": TEST FAIL for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_dev->open_channels_counter_to_recv--;
	test_dev->next_avail_entry_in_array = 0;
	test_ch->next_index_in_sent_msg_per_chan = 0;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return -ENODEV;
}

static int lpm_test_create_read_thread(struct test_channel *test_ch)
{
	struct sdio_test_device *test_dev;

	pr_info(TEST_MODULE_NAME ": %s - STARTED channel %s\n",
		__func__, test_ch->name);

	if (!test_ch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test channel\n", __func__);
		return -ENODEV;
	}

	test_dev = test_ch->test_device;

	if (!test_dev) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test device\n", __func__);
		return -ENODEV;
	}

	test_dev->lpm_test_task.task_name = SDIO_LPM_TEST;

	test_dev->lpm_test_task.lpm_task =
		kthread_create(lpm_test_main_task,
			       (void *)(test_ch),
			       test_dev->lpm_test_task.task_name);

	if (IS_ERR(test_dev->lpm_test_task.lpm_task)) {
		pr_err(TEST_MODULE_NAME ": %s - kthread_create() failed\n",
			__func__);
		return -ENOMEM;
	}

	wake_up_process(test_dev->lpm_test_task.lpm_task);

	return 0;
}

static void lpm_continuous_rand_test(struct test_channel *test_ch)
{
	unsigned int local_ms = 0;
	int ret = 0;
	unsigned int write_avail = 0;
	struct sdio_test_device *test_dev;

	pr_info(MODULE_NAME ": %s - STARTED\n", __func__);

	if (!test_ch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL channel\n", __func__);
		return;
	}

	test_dev = test_ch->test_device;

	if (!test_dev) {
		pr_err(TEST_MODULE_NAME ": %s - NULL Test Device\n", __func__);
		return;
	}

	ret = lpm_test_create_read_thread(test_ch);
	if (ret != 0) {
		pr_err(TEST_MODULE_NAME ": %s - failed to create lpm reading "
		       "thread", __func__);
	}

	while (test_ch->next_index_in_sent_msg_per_chan <=
	       test_ch->config_msg.num_packets - 1) {

		struct lpm_msg msg;
		u32 ret = 0;

		/* sleeping period is dependent on number of open channels */
		test_ch->config_msg.test_param =
				test_ctx->lpm_pseudo_random_seed;

		local_ms = test_dev->open_channels_counter_to_send *
			test_ctx->lpm_pseudo_random_seed;
		TEST_DBG(TEST_MODULE_NAME ":%s: SLEEPING for %d ms",
		       __func__, local_ms);
		msleep(local_ms);

		msg.counter = test_ch->next_index_in_sent_msg_per_chan;
		msg.signature = LPM_TEST_CONFIG_SIGNATURE;
		msg.reserve1 = 0;
		msg.reserve2 = 0;

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		pr_debug(TEST_MODULE_NAME ": %s: write_avail=%d\n",
		       __func__, write_avail);
		if (write_avail < sizeof(msg)) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		if (write_avail < sizeof(msg)) {
			pr_info(TEST_MODULE_NAME ": %s: not enough write "
				"avail.\n", __func__);
			break;
		}

		ret = sdio_write(test_ch->ch, (u32 *)&msg, sizeof(msg));
		if (ret)
			pr_err(TEST_MODULE_NAME ":%s: sdio_write err=%d.\n",
				__func__, -ret);

		TEST_DBG(TEST_MODULE_NAME ": %s: for chan %s, write, "
			 "msg # %d\n",
			 __func__,
			 test_ch->name,
			 test_ch->next_index_in_sent_msg_per_chan);

		if (test_ch->test_type == SDIO_TEST_LPM_RANDOM) {
			spin_lock_irqsave(&test_dev->lpm_array_lock,
					  test_dev->lpm_array_lock_flags);
			lpm_test_update_entry(test_ch, LPM_MSG_SEND,
					      "SEND  ",
					      test_ch->
					      next_index_in_sent_msg_per_chan);

			test_ch->next_index_in_sent_msg_per_chan++;

			spin_unlock_irqrestore(&test_dev->lpm_array_lock,
					       test_dev->lpm_array_lock_flags);
		}
	}

	spin_lock_irqsave(&test_dev->lpm_array_lock,
				  test_dev->lpm_array_lock_flags);
	test_dev->open_channels_counter_to_send--;
	spin_unlock_irqrestore(&test_dev->lpm_array_lock,
				       test_dev->lpm_array_lock_flags);

	pr_info(TEST_MODULE_NAME ": %s: - Finished to send all (%d) "
		"packets to the modem on channel %s",
		__func__, test_ch->config_msg.num_packets, test_ch->name);

	return;
}

static void lpm_test(struct test_channel *test_ch)
{
	pr_info(TEST_MODULE_NAME ": %s - START channel %s\n", __func__,
		test_ch->name);

	if (!test_ch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL test channel\n", __func__);
		return;
	}

	test_ch->modem_result_per_chan = wait_for_result_msg(test_ch);
	pr_debug(TEST_MODULE_NAME ": %s - delete the timeout timer\n",
	       __func__);
	del_timer_sync(&test_ch->timeout_timer);

	if (test_ch->modem_result_per_chan == 0) {
		pr_err(TEST_MODULE_NAME ": LPM TEST - Client didn't sleep. "
		       "Result Msg - is_successful=%d\n", test_ch->buf[1]);
		goto exit_err;
	} else {
		pr_info(TEST_MODULE_NAME ": %s -"
			"LPM 9K WAS SLEEPING - PASS\n", __func__);
		if (test_ch->test_result == TEST_PASSED) {
			pr_info(TEST_MODULE_NAME ": LPM TEST_PASSED\n");
			test_ch->test_completed = 1;
			check_test_completion();
		} else {
			pr_err(TEST_MODULE_NAME ": LPM TEST - Host didn't "
			       "sleep. Client slept\n");
			goto exit_err;
		}
	}

	return;

exit_err:
	pr_info(TEST_MODULE_NAME ": TEST FAIL for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}


/**
 * LPM Test while the host wakes up the modem
 */
static void lpm_test_host_waker(struct test_channel *test_ch)
{
	pr_info(TEST_MODULE_NAME ": %s - START\n", __func__);
	wait_event(test_ch->wait_q, atomic_read(&test_ch->wakeup_client));
	atomic_set(&test_ch->wakeup_client, 0);

	pr_info(TEST_MODULE_NAME ": %s - Sending the config_msg to wakeup "
		" the client\n", __func__);
	send_config_msg(test_ch);

	lpm_test(test_ch);
}

static void notify(void *priv, unsigned channel_event);

/**
  * Writes number of packets into test channel
  * @test_ch: test channel control struct
  * @burst_size: number of packets to send
  */
static int write_packet_burst(struct test_channel *test_ch,
		int burst_size)
{
	int ret = 0;
	int packet_count = 0;
	unsigned int random_num = 0;
	int size = test_ch->packet_length; /* first packet size */
	u32 write_avail = 0;

	while (packet_count < burst_size) {
		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":%s write_avail=%d,size=%d on chan"
				" %s\n", __func__,
				write_avail, size, test_ch->name);
		if (write_avail < size) {
			TEST_DBG(TEST_MODULE_NAME ":%s wait for event on"
					" chan %s\n", __func__, test_ch->name);
			wait_event(test_ch->wait_q,
					atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}
		write_avail = sdio_write_avail(test_ch->ch);
		if (write_avail < size) {
			pr_info(TEST_MODULE_NAME ":%s not enough write"
					" avail %d, need %d on chan %s\n",
					__func__, write_avail, size,
					test_ch->name);
			continue;
		}
		ret = sdio_write(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_err(TEST_MODULE_NAME ":%s sdio_write "
					"failed (%d) on chan %s\n", __func__,
					ret, test_ch->name);
			break;
		}
		udelay(1000); /*low bus usage while running number of channels*/
		TEST_DBG(TEST_MODULE_NAME ":%s() successfully write %d bytes"
				", packet_count=%d on chan %s\n", __func__,
				size, packet_count, test_ch->name);
		test_ch->tx_bytes += size;
		packet_count++;
		/* get next packet size */
		random_num = get_random_int();
		size = (random_num % test_ch->packet_length) + 1;
	}
	return ret;
}
/**
  * Reads packet from test channel and checks that packet number
  * encoded into the packet is equal to packet_counter
  *
  * @test_ch: test channel
  * @size: expected packet size
  * @packet_counter: number to validate readed packet
  */
static int read_data_from_channel(struct test_channel *test_ch,
				unsigned int size,
				int packet_counter)
{
	u32 read_avail = 0;
	int ret = 0;

	if (!test_ch->ch->is_packet_mode) {
		pr_err(TEST_MODULE_NAME
				":%s:not packet mode ch %s\n",
				__func__, test_ch->name);
		return -EINVAL;
	}
	read_avail = sdio_read_avail(test_ch->ch);
	/* wait for read data ready event */
	if (read_avail < size) {
		TEST_DBG(TEST_MODULE_NAME ":%s() wait for rx data on "
				"chan %s\n", __func__, test_ch->name);
		wait_event(test_ch->wait_q,
				atomic_read(&test_ch->rx_notify_count));
		atomic_dec(&test_ch->rx_notify_count);
	}
	read_avail = sdio_read_avail(test_ch->ch);
	TEST_DBG(TEST_MODULE_NAME ":%s read_avail=%d bytes on chan %s\n",
			__func__, read_avail, test_ch->name);

	if (read_avail != size) {
		pr_err(TEST_MODULE_NAME
				":read_avail size %d for chan %s not as "
				"expected size %d\n",
				read_avail, test_ch->name, size);
		return -EINVAL;
	}

	ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
	if (ret) {
		pr_err(TEST_MODULE_NAME ":%s() sdio_read for chan %s (%d)\n",
				__func__, test_ch->name, -ret);
		return ret;
	}
	if ((test_ch->buf[0] != packet_counter) && (size != 1)) {
		pr_err(TEST_MODULE_NAME ":Read WRONG DATA"
				" for chan %s, size=%d\n",
				test_ch->name, size);
		return -EINVAL;
	}
	return 0;
}

/**
 *   Test close channel feature:
 *   1. write random packet number into channel
 *   2. read some data from channel (do this only for second half of
 *   requested packets to send).
 *   3. close && re-open then repeat 1.
 *
 *   Total packets to send: test_ch->config_msg.num_packets.
 *   Burst size is random in [1..test_ch->max_burst_size] range
 *   Packet size is random in [1..test_ch->packet_length]
 */
static void open_close_test(struct test_channel *test_ch)
{
	int ret = 0;
	u32 read_avail = 0;
	int total_packet_count = 0;
	int size = test_ch->packet_length;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packet_count = 0;
	unsigned int random_num = 0;
	int curr_burst_size = test_ch->max_burst_size;

	/* the test sends configured number of packets in
	   2 portions: first without reading between write bursts,
	   second with it */
	max_packet_count = test_ch->config_msg.num_packets / 2;

	pr_info(TEST_MODULE_NAME ":%s channel %s, total packets:%d,"
			" max packet size %d, max burst size:%d\n",
			__func__, test_ch->name,
			test_ch->config_msg.num_packets, test_ch->packet_length,
			test_ch->max_burst_size);
	for (i = 0 ; i < size / 2 ; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	for (i = 0; i < 2 ; i++) {
		total_packet_count = 0;
		while (total_packet_count < max_packet_count) {
			if (test_ctx->exit_flag) {
				pr_info(TEST_MODULE_NAME ":%s exit test\n",
						__func__);
				return;
			}
			test_ch->buf[0] = total_packet_count;
			random_num = get_random_int();
			curr_burst_size = (random_num %
					test_ch->max_burst_size) + 1;

			/* limit burst size to send
			 * no more than configured packets */
			if (curr_burst_size + total_packet_count >
					max_packet_count) {
				curr_burst_size = max_packet_count -
					total_packet_count;
			}
			TEST_DBG(TEST_MODULE_NAME ":%s Current burst size:%d"
					" on chan %s\n", __func__,
					curr_burst_size, test_ch->name);
			ret = write_packet_burst(test_ch, curr_burst_size);
			if (ret) {
				pr_err(TEST_MODULE_NAME ":%s write burst "
						"failed (%d), ch %s\n",
						__func__, ret, test_ch->name);
				goto exit_err;
			}
			if (i > 0) {
				/* read from channel */
				ret = read_data_from_channel(test_ch, size,
						total_packet_count);
				if (ret) {
					pr_err(TEST_MODULE_NAME ":%s read"
							" failed:%d, chan %s\n",
							__func__, ret,
							test_ch->name);
					goto exit_err;
				}
			}
			TEST_DBG(TEST_MODULE_NAME ":%s before close, ch %s\n",
					__func__, test_ch->name);
			ret = sdio_close(test_ch->ch);
			if (ret) {
				pr_err(TEST_MODULE_NAME":%s close channel %s"
						" failed (%d)\n",
						__func__, test_ch->name, ret);
				goto exit_err;
			} else {
				TEST_DBG(TEST_MODULE_NAME":%s close channel %s"
						" success\n", __func__,
						test_ch->name);
				total_packet_count += curr_burst_size;
				atomic_set(&test_ch->rx_notify_count, 0);
				atomic_set(&test_ch->tx_notify_count, 0);
				atomic_set(&test_ch->any_notify_count, 0);
			}
			TEST_DBG(TEST_MODULE_NAME ":%s before open, ch %s\n",
					__func__, test_ch->name);
			ret = sdio_open(test_ch->name , &test_ch->ch,
					test_ch, notify);
			if (ret) {
				pr_err(TEST_MODULE_NAME":%s open channel %s"
						" failed (%d)\n",
						__func__, test_ch->name, ret);
				goto exit_err;
			} else {
				read_avail = sdio_read_avail(test_ch->ch);
				if (read_avail > 0) {
					pr_err(TEST_MODULE_NAME": after open"
						" ch %s read_availis not zero"
						" (%d bytes)\n",
						test_ch->name, read_avail);
					goto exit_err;
				}
			}
			TEST_DBG(TEST_MODULE_NAME ":%s total tx = %d,"
					" packet# = %d, size = %d for ch %s\n",
					__func__, test_ch->tx_bytes,
					total_packet_count, size,
					test_ch->name);
		} /* end of while */
	}
	pr_info(TEST_MODULE_NAME ":%s Test end: total rx bytes = 0x%x,"
			" total tx bytes = 0x%x for chan %s\n", __func__,
			test_ch->rx_bytes, test_ch->tx_bytes, test_ch->name);
	pr_info(TEST_MODULE_NAME ":%s TEST PASS for chan %s.\n", __func__,
			test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;
exit_err:
	pr_info(TEST_MODULE_NAME ":%s TEST FAIL for chan %s.\n", __func__,
			test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

/**
 * sender Test
 */
static void sender_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int packet_count = 0;
	int size = 512;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packet_count = 10000;
	int random_num = 0;

	max_packet_count = test_ch->config_msg.num_packets;

	for (i = 0 ; i < size / 2 ; i++)
		buf16[i] = (u16) (i & 0xFFFF);


	pr_info(TEST_MODULE_NAME
		 ":SENDER TEST START for chan %s\n", test_ch->name);

	while (packet_count < max_packet_count) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		random_num = get_random_int();
		size = (random_num % test_ch->packet_length) + 1;

		TEST_DBG(TEST_MODULE_NAME "SENDER WAIT FOR EVENT for chan %s\n",
			test_ch->name);

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			pr_info(TEST_MODULE_NAME ":not enough write avail.\n");
			continue;
		}

		test_ch->buf[0] = packet_count;

		ret = sdio_write(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sender sdio_write err=%d.\n",
				-ret);
			goto exit_err;
		}

		/* wait for read data ready event */
		TEST_DBG(TEST_MODULE_NAME ":sender wait for rx data for "
					  "chan %s\n",
			 test_ch->name);
		read_avail = sdio_read_avail(test_ch->ch);
		wait_event(test_ch->wait_q,
			   atomic_read(&test_ch->rx_notify_count));
		atomic_dec(&test_ch->rx_notify_count);

		read_avail = sdio_read_avail(test_ch->ch);

		if (read_avail != size) {
			pr_info(TEST_MODULE_NAME
				":read_avail size %d for chan %s not as "
				"expected size %d.\n",
				read_avail, test_ch->name, size);
			goto exit_err;
		}

		memset(test_ch->buf, 0x00, size);

		ret = sdio_read(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sender sdio_read for chan %s"
						 " err=%d.\n",
				test_ch->name, -ret);
			goto exit_err;
		}


		if ((test_ch->buf[0] != packet_count) && (size != 1)) {
			pr_info(TEST_MODULE_NAME ":sender sdio_read WRONG DATA"
						 " for chan %s, size=%d\n",
				test_ch->name, size);
			goto exit_err;
		}

		test_ch->tx_bytes += size;
		test_ch->rx_bytes += size;
		packet_count++;

		TEST_DBG(TEST_MODULE_NAME
			 ":sender total rx bytes = 0x%x , packet#=%d, size=%d"
			 " for chan %s\n",
			 test_ch->rx_bytes, packet_count, size, test_ch->name);
		TEST_DBG(TEST_MODULE_NAME
			 ":sender total tx bytes = 0x%x , packet#=%d, size=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, packet_count, size, test_ch->name);

	} /* end of while */

	pr_info(TEST_MODULE_NAME
		 ":SENDER TEST END: total rx bytes = 0x%x, "
		 " total tx bytes = 0x%x for chan %s\n",
		 test_ch->rx_bytes, test_ch->tx_bytes, test_ch->name);

	pr_info(TEST_MODULE_NAME ": TEST PASS for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;

exit_err:
	pr_info(TEST_MODULE_NAME ": TEST FAIL for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

/**
 * A2 Perf Test
 */
static void a2_performance_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int tx_packet_count = 0;
	int rx_packet_count = 0;
	int size = 0;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int total_bytes = 0;
	int max_packets = 10000;
	u32 packet_size = test_ch->buf_size;
	int rand_size = 0;

	u64 start_jiffy, end_jiffy, delta_jiffies;
	unsigned int time_msec = 0;
	u32 throughput = 0;

	max_packets = test_ch->config_msg.num_packets;
	packet_size = test_ch->packet_length;

	for (i = 0; i < packet_size / 2; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(TEST_MODULE_NAME ": A2 PERFORMANCE TEST START for chan %s\n",
		test_ch->name);

	start_jiffy = get_jiffies_64(); /* read the current time */

	while (tx_packet_count < max_packets) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		if (test_ch->random_packet_size) {
			rand_size = get_random_int();
			packet_size = (rand_size % test_ch->packet_length) + 1;
			if (packet_size < A2_MIN_PACKET_SIZE)
				packet_size = A2_MIN_PACKET_SIZE;
		}

		/* wait for data ready event */
		/* use a func to avoid compiler optimizations */
		write_avail = sdio_write_avail(test_ch->ch);
		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d, "
					 "read_avail=%d for chan %s\n",
			test_ch->name, write_avail, read_avail,
			test_ch->name);
		if ((write_avail == 0) && (read_avail == 0)) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->any_notify_count));
			atomic_set(&test_ch->any_notify_count, 0);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d\n",
			 test_ch->name, write_avail);
		if (write_avail > 0) {
			size = min(packet_size, write_avail) ;
			TEST_DBG(TEST_MODULE_NAME ":tx size = %d for chan %s\n",
				 size, test_ch->name);
			test_ch->buf[0] = tx_packet_count;
			test_ch->buf[(size/4)-1] = tx_packet_count;

			ret = sdio_write(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ":sdio_write err=%d"
							 " for chan %s\n",
					-ret, test_ch->name);
				goto exit_err;
			}
			tx_packet_count++;
			test_ch->tx_bytes += size;
		}

		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, read_avail=%d\n",
			 test_ch->name, read_avail);
		if (read_avail > 0) {
			size = min(packet_size, read_avail);
			pr_debug(TEST_MODULE_NAME ":rx size = %d.\n", size);
			ret = sdio_read(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ": sdio_read size %d "
							 " err=%d"
							 " for chan %s\n",
					size, -ret, test_ch->name);
				goto exit_err;
			}
			rx_packet_count++;
			test_ch->rx_bytes += size;
		}

		TEST_DBG(TEST_MODULE_NAME
			 ":total rx bytes = %d , rx_packet#=%d"
			 " for chan %s\n",
			 test_ch->rx_bytes, rx_packet_count, test_ch->name);
		TEST_DBG(TEST_MODULE_NAME
			 ":total tx bytes = %d , tx_packet#=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, tx_packet_count, test_ch->name);

	} /* while (tx_packet_count < max_packets ) */

	end_jiffy = get_jiffies_64(); /* read the current time */

	delta_jiffies = end_jiffy - start_jiffy;
	time_msec = jiffies_to_msecs(delta_jiffies);

	pr_info(TEST_MODULE_NAME ":total rx bytes = 0x%x , rx_packet#=%d for"
				 " chan %s.\n",
		test_ch->rx_bytes, rx_packet_count, test_ch->name);
	pr_info(TEST_MODULE_NAME ":total tx bytes = 0x%x , tx_packet#=%d"
				 " for chan %s.\n",
		test_ch->tx_bytes, tx_packet_count, test_ch->name);

	total_bytes = (test_ch->tx_bytes + test_ch->rx_bytes);
	pr_err(TEST_MODULE_NAME ":total bytes = %d, time msec = %d"
				" for chan %s\n",
		   total_bytes , (int) time_msec, test_ch->name);

	if (!test_ch->random_packet_size) {
		throughput = (total_bytes / time_msec) * 8 / 1000;
		pr_err(TEST_MODULE_NAME ":Performance = %d Mbit/sec for "
					"chan %s\n",
		       throughput, test_ch->name);
	}

#ifdef CONFIG_DEBUG_FS
	switch (test_ch->ch_id) {
	case SDIO_DUN:
		test_ctx->debug.dun_throughput = throughput;
		break;
	case SDIO_RMNT:
		test_ctx->debug.rmnt_throughput = throughput;
		break;
	default:
		pr_err(TEST_MODULE_NAME "No debugfs for this channel "
					"throughput");
	}
#endif

	pr_err(TEST_MODULE_NAME ": A2 PERFORMANCE TEST END for chan %s.\n",
	       test_ch->name);

	pr_err(TEST_MODULE_NAME ": TEST PASS for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;

exit_err:
	pr_err(TEST_MODULE_NAME ": TEST FAIL for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

/**
 * rx_cleanup
 * This function reads all the messages sent by the modem until
 * the read_avail is 0 after 1 second of sleep.
 * The function returns the number of packets that was received.
 */
static void rx_cleanup(struct test_channel *test_ch, int *rx_packet_count)
{
	int read_avail = 0;
	int ret = 0;

	read_avail = sdio_read_avail(test_ch->ch);
	TEST_DBG(TEST_MODULE_NAME ":channel %s, read_avail=%d\n",
		 test_ch->name, read_avail);

	/* If no pending messages, wait to see if the modem sends data */
	if (read_avail == 0) {
		msleep(1000);
		read_avail = sdio_read_avail(test_ch->ch);
	}

	while (read_avail > 0) {
		TEST_DBG(TEST_MODULE_NAME ": read_avail=%d for ch %s\n",
			 read_avail, test_ch->name);

		ret = sdio_read(test_ch->ch, test_ch->buf, read_avail);
		if (ret) {
			pr_info(TEST_MODULE_NAME ": sdio_read size %d "
						 " err=%d for chan %s\n",
				read_avail, -ret, test_ch->name);
			break;
		}
		(*rx_packet_count)++;
		test_ch->rx_bytes += read_avail;
		read_avail = sdio_read_avail(test_ch->ch);
		if (read_avail == 0) {
			msleep(1000);
			read_avail = sdio_read_avail(test_ch->ch);
		}
	}
	pr_info(TEST_MODULE_NAME ": finished cleanup for ch %s, "
				 "rx_packet_count=%d, total rx bytes=%d\n",
			 test_ch->name, *rx_packet_count, test_ch->rx_bytes);
}


/**
 * A2 RTT Test
 * This function sends a packet and calculate the RTT time of
 * this packet.
 * The test also calculte Min, Max and Average RTT
 */
static void a2_rtt_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int tx_packet_count = 0;
	int rx_packet_count = 0;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packets = test_ch->config_msg.num_packets;
	u32 packet_size = test_ch->packet_length;
	s64 start_time, end_time;
	int delta_usec = 0;
	int time_average = 0;
	int min_delta_usec = 0xFFFF;
	int max_delta_usec = 0;
	int total_time = 0;
	int expected_read_size = 0;

	for (i = 0; i < packet_size / 2; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(TEST_MODULE_NAME ": A2 RTT TEST START for chan %s\n",
		test_ch->name);

	rx_cleanup(test_ch, &rx_packet_count);
	rx_packet_count = 0;

	while (tx_packet_count < max_packets) {
		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}
		start_time = 0;
		end_time = 0;

		/* Allow sdio_al to go to sleep to change the read_threshold
		 *  to 1
		 */
		msleep(100);

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":ch %s: write_avail=%d\n",
			test_ch->name, write_avail);
		if (write_avail == 0) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d\n",
			 test_ch->name, write_avail);
		if (write_avail > 0) {
			TEST_DBG(TEST_MODULE_NAME ":tx size = %d for chan %s\n",
				 packet_size, test_ch->name);
			test_ch->buf[0] = tx_packet_count;

			start_time = ktime_to_us(ktime_get());
			ret = sdio_write(test_ch->ch, test_ch->buf,
					 packet_size);
			if (ret) {
				pr_err(TEST_MODULE_NAME ":sdio_write err=%d"
							 " for chan %s\n",
					-ret, test_ch->name);
				goto exit_err;
			}
			tx_packet_count++;
			test_ch->tx_bytes += packet_size;
		} else {
				pr_err(TEST_MODULE_NAME ": Invalid write_avail"
							 " %d for chan %s\n",
					write_avail, test_ch->name);
				goto exit_err;
		}

		expected_read_size = packet_size + A2_HEADER_OVERHEAD;

		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, read_avail=%d\n",
			 test_ch->name, read_avail);
		while (read_avail < expected_read_size) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->rx_notify_count));
			atomic_dec(&test_ch->rx_notify_count);
			read_avail = sdio_read_avail(test_ch->ch);
		}

		if (read_avail >= expected_read_size) {
			pr_debug(TEST_MODULE_NAME ":read_avail=%d for ch %s.\n",
				 read_avail, test_ch->name);
			ret = sdio_read(test_ch->ch, test_ch->buf,
					expected_read_size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ": sdio_read size %d "
					" err=%d for chan %s\n",
					expected_read_size, -ret,
					test_ch->name);
				goto exit_err;
			}
			end_time = ktime_to_us(ktime_get());
			rx_packet_count++;
			test_ch->rx_bytes += expected_read_size;
		} else {
				pr_info(TEST_MODULE_NAME ": Invalid read_avail "
							 "%d for chan %s\n",
					read_avail, test_ch->name);
				goto exit_err;
		}

		delta_usec = (int)(end_time - start_time);
		total_time += delta_usec;
		if (delta_usec < min_delta_usec)
				min_delta_usec = delta_usec;
		if (delta_usec > max_delta_usec)
				max_delta_usec = delta_usec;

		TEST_DBG(TEST_MODULE_NAME
			 ":RTT time=%d for packet #%d for chan %s\n",
			 delta_usec, tx_packet_count, test_ch->name);

	} /* while (tx_packet_count < max_packets ) */


	pr_info(TEST_MODULE_NAME ":total rx bytes = 0x%x , rx_packet#=%d for"
				 " chan %s.\n",
		test_ch->rx_bytes, rx_packet_count, test_ch->name);
	pr_info(TEST_MODULE_NAME ":total tx bytes = 0x%x , tx_packet#=%d"
				 " for chan %s.\n",
		test_ch->tx_bytes, tx_packet_count, test_ch->name);

	time_average = total_time / tx_packet_count;

	pr_info(TEST_MODULE_NAME ":Average RTT time = %d for chan %s\n",
		   time_average, test_ch->name);
	pr_info(TEST_MODULE_NAME ":MIN RTT time = %d for chan %s\n",
		   min_delta_usec, test_ch->name);
	pr_info(TEST_MODULE_NAME ":MAX RTT time = %d for chan %s\n",
		   max_delta_usec, test_ch->name);

	pr_info(TEST_MODULE_NAME ": A2 RTT TEST END for chan %s.\n",
	       test_ch->name);

	pr_info(TEST_MODULE_NAME ": TEST PASS for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;

exit_err:
	pr_err(TEST_MODULE_NAME ": TEST FAIL for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}


/**
 * sender No loopback Test
 */
static void sender_no_loopback_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 write_avail = 0;
	int packet_count = 0;
	int size = 512;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packet_count = 10000;
	unsigned int random_num = 0;

	max_packet_count = test_ch->config_msg.num_packets;

	for (i = 0 ; i < size / 2 ; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(TEST_MODULE_NAME
		 ":SENDER NO LP TEST START for chan %s\n", test_ch->name);

	while (packet_count < max_packet_count) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		random_num = get_random_int();
		size = (random_num % test_ch->packet_length) + 1;

		TEST_DBG(TEST_MODULE_NAME ":SENDER WAIT FOR EVENT "
					  "for chan %s\n",
			test_ch->name);

		/* wait for data ready event */
		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->tx_notify_count));
			atomic_dec(&test_ch->tx_notify_count);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":write_avail=%d\n", write_avail);
		if (write_avail < size) {
			pr_info(TEST_MODULE_NAME ":not enough write avail.\n");
			continue;
		}

		test_ch->buf[0] = packet_count;

		ret = sdio_write(test_ch->ch, test_ch->buf, size);
		if (ret) {
			pr_info(TEST_MODULE_NAME ":sender sdio_write err=%d.\n",
				-ret);
			goto exit_err;
		}

		test_ch->tx_bytes += size;
		packet_count++;

		TEST_DBG(TEST_MODULE_NAME
			 ":sender total tx bytes = 0x%x , packet#=%d, size=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, packet_count, size, test_ch->name);

	} /* end of while */

	pr_info(TEST_MODULE_NAME
		 ":SENDER TEST END: total tx bytes = 0x%x, "
		 " for chan %s\n",
		 test_ch->tx_bytes, test_ch->name);

	test_ch->modem_result_per_chan = wait_for_result_msg(test_ch);

	if (test_ch->modem_result_per_chan) {
		pr_info(TEST_MODULE_NAME ": TEST PASS for chan %s.\n",
			test_ch->name);
		test_ch->test_result = TEST_PASSED;
	} else {
		pr_info(TEST_MODULE_NAME ": TEST FAILURE for chan %s.\n",
			test_ch->name);
		test_ch->test_result = TEST_FAILED;
	}
	test_ch->test_completed = 1;
	check_test_completion();
	return;

exit_err:
	pr_info(TEST_MODULE_NAME ": TEST FAIL for chan %s.\n",
		test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}


/**
 * Modem reset Test
 * The test verifies that it finished sending all the packets
 * while there might be modem reset in the middle
 */
static void modem_reset_test(struct test_channel *test_ch)
{
	int ret = 0 ;
	u32 read_avail = 0;
	u32 write_avail = 0;
	int tx_packet_count = 0;
	int rx_packet_count = 0;
	int size = 0;
	u16 *buf16 = (u16 *) test_ch->buf;
	int i;
	int max_packets = 10000;
	u32 packet_size = test_ch->buf_size;
	int is_err = 0;

	max_packets = test_ch->config_msg.num_packets;
	packet_size = test_ch->packet_length;

	for (i = 0; i < packet_size / 2; i++)
		buf16[i] = (u16) (i & 0xFFFF);

	pr_info(TEST_MODULE_NAME ": Modem Reset TEST START for chan %s\n",
		test_ch->name);

	while (tx_packet_count < max_packets) {

		if (test_ctx->exit_flag) {
			pr_info(TEST_MODULE_NAME ":Exit Test.\n");
			return;
		}

		if (test_ch->card_removed) {
			pr_info(TEST_MODULE_NAME ": card removal was detected "
				"for chan %s, tx_total=0x%x\n",
				test_ch->name, test_ch->tx_bytes);
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->card_detected_event));
			atomic_set(&test_ch->card_detected_event, 0);
			pr_info(TEST_MODULE_NAME ": card_detected_event "
					"for chan %s\n", test_ch->name);
			if (test_ch->card_removed)
				continue;
			is_err = 0;
			/* Need to wait for the modem to be ready */
			msleep(5000);
			pr_info(TEST_MODULE_NAME ": sending the config message "
					"for chan %s\n", test_ch->name);
			send_config_msg(test_ch);
		}

		/* wait for data ready event */
		/* use a func to avoid compiler optimizations */
		write_avail = sdio_write_avail(test_ch->ch);
		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d, "
					 "read_avail=%d for chan %s\n",
			test_ch->name, write_avail, read_avail,
			test_ch->name);
		if ((write_avail == 0) && (read_avail == 0)) {
			wait_event(test_ch->wait_q,
				   atomic_read(&test_ch->any_notify_count));
			atomic_set(&test_ch->any_notify_count, 0);
		}
		if (atomic_read(&test_ch->card_detected_event)) {
			atomic_set(&test_ch->card_detected_event, 0);
			pr_info(TEST_MODULE_NAME ": card_detected_event "
				"for chan %s, tx_total=0x%x\n",
				test_ch->name,  test_ch->tx_bytes);
			if (test_ch->card_removed)
				continue;
			/* Need to wait for the modem to be ready */
			msleep(5000);
			is_err = 0;
			pr_info(TEST_MODULE_NAME ": sending the config message "
					"for chan %s\n", test_ch->name);
			send_config_msg(test_ch);
		}

		write_avail = sdio_write_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, write_avail=%d\n",
			 test_ch->name, write_avail);
		if (write_avail > 0) {
			size = min(packet_size, write_avail) ;
			pr_debug(TEST_MODULE_NAME ":tx size = %d for chan %s\n",
				 size, test_ch->name);
			test_ch->buf[0] = tx_packet_count;
			test_ch->buf[(size/4)-1] = tx_packet_count;

			TEST_DBG(TEST_MODULE_NAME ":channel %s, sdio_write, "
				"size=%d\n", test_ch->name, size);
			if (is_err) {
				msleep(100);
				continue;
			}
			ret = sdio_write(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ":sdio_write err=%d"
							 " for chan %s\n",
					-ret, test_ch->name);
				is_err = 1;
				msleep(20);
				continue;
			}
			tx_packet_count++;
			test_ch->tx_bytes += size;
			test_ch->config_msg.num_packets--;
		}

		read_avail = sdio_read_avail(test_ch->ch);
		TEST_DBG(TEST_MODULE_NAME ":channel %s, read_avail=%d\n",
			 test_ch->name, read_avail);
		if (read_avail > 0) {
			size = min(packet_size, read_avail);
			pr_debug(TEST_MODULE_NAME ":rx size = %d.\n", size);
			TEST_DBG(TEST_MODULE_NAME ":channel %s, sdio_read, "
				"size=%d\n", test_ch->name, size);
			if (is_err) {
				msleep(100);
				continue;
			}
			ret = sdio_read(test_ch->ch, test_ch->buf, size);
			if (ret) {
				pr_info(TEST_MODULE_NAME ": sdio_read size %d "
							 " err=%d"
							 " for chan %s\n",
					size, -ret, test_ch->name);
				is_err = 1;
				msleep(20);
				continue;
			}
			rx_packet_count++;
			test_ch->rx_bytes += size;
		}

		TEST_DBG(TEST_MODULE_NAME
			 ":total rx bytes = %d , rx_packet#=%d"
			 " for chan %s\n",
			 test_ch->rx_bytes, rx_packet_count, test_ch->name);
		TEST_DBG(TEST_MODULE_NAME
			 ":total tx bytes = %d , tx_packet#=%d"
			 " for chan %s\n",
			 test_ch->tx_bytes, tx_packet_count, test_ch->name);

		udelay(500);

	} /* while (tx_packet_count < max_packets ) */

	rx_cleanup(test_ch, &rx_packet_count);

	pr_info(TEST_MODULE_NAME ":total rx bytes = 0x%x , rx_packet#=%d for"
				 " chan %s.\n",
		test_ch->rx_bytes, rx_packet_count, test_ch->name);
	pr_info(TEST_MODULE_NAME ":total tx bytes = 0x%x , tx_packet#=%d"
				 " for chan %s.\n",
		test_ch->tx_bytes, tx_packet_count, test_ch->name);

	pr_err(TEST_MODULE_NAME ": Modem Reset TEST END for chan %s.\n",
	       test_ch->name);

	pr_err(TEST_MODULE_NAME ": TEST PASS for chan %s\n", test_ch->name);
	test_ch->test_completed = 1;
	test_ch->test_result = TEST_PASSED;
	check_test_completion();
	return;
}

/**
 * Worker thread to handle the tests types
 */
static void worker(struct work_struct *work)
{
	struct test_channel *test_ch = NULL;
	struct test_work *test_work = container_of(work,
						 struct	test_work,
						 work);
	int test_type = 0;

	test_ch = test_work->test_ch;

	if (test_ch == NULL) {
		pr_err(TEST_MODULE_NAME ":NULL test_ch\n");
		return;
	}

	test_type = test_ch->test_type;

	switch (test_type) {
	case SDIO_TEST_LOOPBACK_HOST:
		loopback_test(test_ch);
		break;
	case SDIO_TEST_LOOPBACK_CLIENT:
		sender_test(test_ch);
		break;
	case SDIO_TEST_PERF:
		a2_performance_test(test_ch);
		break;
	case SDIO_TEST_LPM_CLIENT_WAKER:
		lpm_test(test_ch);
		break;
	case SDIO_TEST_LPM_HOST_WAKER:
		lpm_test_host_waker(test_ch);
		break;
	case SDIO_TEST_HOST_SENDER_NO_LP:
		sender_no_loopback_test(test_ch);
		break;
	case SDIO_TEST_LPM_RANDOM:
		lpm_continuous_rand_test(test_ch);
		break;
	case SDIO_TEST_RTT:
		a2_rtt_test(test_ch);
		break;
	case SDIO_TEST_CLOSE_CHANNEL:
		open_close_test(test_ch);
		break;
	case SDIO_TEST_MODEM_RESET:
		modem_reset_test(test_ch);
		break;
	default:
		pr_err(TEST_MODULE_NAME ":Bad Test type = %d.\n",
			(int) test_type);
	}
}


/**
 * Notification Callback
 *
 * Notify the worker
 *
 */
static void notify(void *priv, unsigned channel_event)
{
	struct test_channel *test_ch = (struct test_channel *) priv;

	pr_debug(TEST_MODULE_NAME ": %s - notify event=%d.\n",
		 __func__, channel_event);

	if (test_ch->ch == NULL) {
		pr_info(TEST_MODULE_NAME ": %s - notify before ch ready.\n",
			__func__);
		return;
	}

	switch (channel_event) {
	case SDIO_EVENT_DATA_READ_AVAIL:
		atomic_inc(&test_ch->rx_notify_count);
		atomic_set(&test_ch->any_notify_count, 1);
		TEST_DBG(TEST_MODULE_NAME ": %s - SDIO_EVENT_DATA_READ_AVAIL, "
			 "any_notify_count=%d, rx_notify_count=%d\n",
			 __func__,
			 atomic_read(&test_ch->any_notify_count),
			 atomic_read(&test_ch->rx_notify_count));
		/*
		 * when there is pending data on a channel we would like to
		 * turn on the bit mask that implies that there is pending
		 * data for that channel on that deivce
		 */
		if (test_ch->test_device != NULL &&
		    test_ch->test_type == SDIO_TEST_LPM_RANDOM) {
			spin_lock_irqsave(&test_ch->test_device->lpm_array_lock,
					  test_ch->test_device->
						  lpm_array_lock_flags);
			test_ch->test_device->read_avail_mask |=
				test_ch->channel_mask_id;
			test_ch->notify_counter_per_chan++;

			lpm_test_update_entry(test_ch, LPM_NOTIFY, "NOTIFY", 0);
			spin_unlock_irqrestore(&test_ch->test_device->
					       lpm_array_lock,
					       test_ch->test_device->
						  lpm_array_lock_flags);
		}
		break;

	case SDIO_EVENT_DATA_WRITE_AVAIL:
		atomic_inc(&test_ch->tx_notify_count);
		atomic_set(&test_ch->any_notify_count, 1);
		TEST_DBG(TEST_MODULE_NAME ": %s - SDIO_EVENT_DATA_WRITE_AVAIL, "
			 "any_notify_count=%d, tx_notify_count=%d\n",
			 __func__,
			 atomic_read(&test_ch->any_notify_count),
			 atomic_read(&test_ch->tx_notify_count));
		break;

	default:
		BUG();
	}
	wake_up(&test_ch->wait_q);

}

#ifdef CONFIG_MSM_SDIO_SMEM
static int sdio_smem_test_cb(int event)
{
	struct test_channel *tch = test_ctx->test_ch_arr[SDIO_SMEM];
	pr_debug(TEST_MODULE_NAME ":%s: Received event %d\n", __func__, event);

	switch (event) {
	case SDIO_SMEM_EVENT_READ_DONE:
		tch->rx_bytes += SMEM_MAX_XFER_SIZE;
		if (tch->rx_bytes >= 40000000) {
			if (!tch->test_completed) {
				pr_info(TEST_MODULE_NAME ":SMEM test PASSED\n");
				tch->test_completed = 1;
				tch->test_result = TEST_PASSED;
				check_test_completion();
			}

		}
		break;
	case SDIO_SMEM_EVENT_READ_ERR:
		pr_err(TEST_MODULE_NAME ":Read overflow, SMEM test FAILED\n");
		tch->test_completed = 1;
		tch->test_result = TEST_FAILED;
		check_test_completion();
		return -EIO;
	default:
		pr_err(TEST_MODULE_NAME ":Unhandled event\n");
		return -EINVAL;
	}
	return 0;
}

static int sdio_smem_test_probe(struct platform_device *pdev)
{
	int ret = 0;

	test_ctx->sdio_smem = container_of(pdev, struct sdio_smem_client,
					   plat_dev);

	test_ctx->sdio_smem->buf = test_ctx->smem_buf;
	test_ctx->sdio_smem->size = SMEM_MAX_XFER_SIZE;
	test_ctx->sdio_smem->cb_func = sdio_smem_test_cb;
	ret = sdio_smem_register_client();
	if (ret)
		pr_info(TEST_MODULE_NAME "%s: Error (%d) registering sdio_smem "
					 "test client\n",
			__func__, ret);
	return ret;
}

static struct platform_driver sdio_smem_client_drv = {
	.probe		= sdio_smem_test_probe,
	.driver		= {
		.name	= "SDIO_SMEM_CLIENT",
		.owner	= THIS_MODULE,
	},
};
#endif


static void default_sdio_al_test_release(struct device *dev)
{
	pr_info(MODULE_NAME ":platform device released.\n");
}

static void sdio_test_lpm_timeout_handler(unsigned long data)
{
	struct test_channel *tch = (struct test_channel *)data;

	pr_info(TEST_MODULE_NAME ": %s - LPM TEST TIMEOUT Expired after "
			    "%d ms\n", __func__, tch->timeout_ms);
	tch->test_completed = 1;
	pr_info(TEST_MODULE_NAME ": %s - tch->test_result = TEST_FAILED\n",
		__func__);
	tch->test_completed = 1;
	tch->test_result = TEST_FAILED;
	check_test_completion();
	return;
}

static void sdio_test_lpm_timer_handler(unsigned long data)
{
	struct test_channel *tch = (struct test_channel *)data;

	pr_info(TEST_MODULE_NAME ": %s - LPM TEST Timer Expired after "
			    "%d ms\n", __func__, tch->timer_interval_ms);

	if (!tch) {
		pr_err(TEST_MODULE_NAME ": %s - LPM TEST FAILED. "
		       "tch is NULL\n", __func__);
		return;
	}

	if (!tch->ch) {
		pr_err(TEST_MODULE_NAME ": %s - LPM TEST FAILED. tch->ch "
		       "is NULL\n", __func__);
		tch->test_result = TEST_FAILED;
		return;
	}

	/* Verfiy that we voted for sleep */
	if (tch->is_ok_to_sleep) {
		tch->test_result = TEST_PASSED;
		pr_info(TEST_MODULE_NAME ": %s - 8K voted for sleep\n",
			__func__);
	} else {
		tch->test_result = TEST_FAILED;
		pr_info(TEST_MODULE_NAME ": %s - 8K voted against sleep\n",
			__func__);

	}

	sdio_al_unregister_lpm_cb(tch->sdio_al_device);

	if (tch->test_type == SDIO_TEST_LPM_HOST_WAKER) {
		atomic_set(&tch->wakeup_client, 1);
		wake_up(&tch->wait_q);
	}
}

int sdio_test_wakeup_callback(void *device_handle, int is_vote_for_sleep)
{
	int i = 0;

	TEST_DBG(TEST_MODULE_NAME ": %s is_vote_for_sleep=%d!!!",
		__func__, is_vote_for_sleep);

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready))
			continue;
		if (tch->sdio_al_device == device_handle) {
			tch->is_ok_to_sleep = is_vote_for_sleep;

			if (tch->test_type == SDIO_TEST_LPM_RANDOM) {
				spin_lock_irqsave(&tch->test_device->
						  lpm_array_lock,
						  tch->test_device->
						  lpm_array_lock_flags);
				if (is_vote_for_sleep == 1)
					lpm_test_update_entry(tch,
							      LPM_SLEEP,
							      "SLEEP ", 0);
				else
					lpm_test_update_entry(tch,
							      LPM_WAKEUP,
							      "WAKEUP", 0);

				spin_unlock_irqrestore(&tch->test_device->
						       lpm_array_lock,
						       tch->test_device->
						       lpm_array_lock_flags);
				break;
			}
		}
	}

	return 0;
}

static int sdio_test_find_dev(struct test_channel *tch)
{
	int j;
	int null_index = -1;

	for (j = 0 ; j < MAX_NUM_OF_SDIO_DEVICES; ++j) {

		struct sdio_test_device *test_dev =
		&test_ctx->test_dev_arr[j];

		if (test_dev->sdio_al_device == NULL) {
			if (null_index == -1)
				null_index = j;
			continue;
		}

		if (test_dev->sdio_al_device ==
		    tch->ch->sdio_al_dev) {
			test_dev->open_channels_counter_to_recv++;
			test_dev->open_channels_counter_to_send++;
			tch->test_device = test_dev;
			/* setting mask id for pending data for
			   this channel */
			tch->channel_mask_id = test_dev->next_mask_id;
			test_dev->next_mask_id *= 2;
			pr_info(TEST_MODULE_NAME ": %s - channel %s "
				"got read_mask_id = 0x%x. device "
				"next_mask_id=0x%x",
				__func__, tch->name, tch->channel_mask_id,
				test_dev->next_mask_id);
			break;
		}
	}

	/*
	 * happens ones a new device is "discovered" while testing. i.e
	 * if testing a few channels, a new deivce will be "discovered" once
	 * the first channel of a device is being tested
	 */
	if (j == MAX_NUM_OF_SDIO_DEVICES) {

		struct sdio_test_device *test_dev =
			&test_ctx->
			test_dev_arr[null_index];
		test_dev->sdio_al_device =
			tch->ch->sdio_al_dev;

		test_ctx->number_of_active_devices++;
		test_ctx->max_number_of_devices++;
		test_dev->open_channels_counter_to_recv++;
		test_dev->open_channels_counter_to_send++;
		test_dev->next_avail_entry_in_array = 0;
		tch->test_device = test_dev;
		tch->test_device->array_size =
			LPM_ARRAY_SIZE;
		test_dev->modem_result_per_dev = 1;
		tch->modem_result_per_chan = 0;

		spin_lock_init(&test_dev->
			       lpm_array_lock);

		if (tch->test_type == SDIO_TEST_LPM_RANDOM) {
			pr_err(MODULE_NAME ": %s - "
			       "Allocating Msg Array for "
			       "Maximum open channels for device (%d) "
			       "Channels. Array has %d entries",
			       __func__,
			       LPM_MAX_OPEN_CHAN_PER_DEV,
			       test_dev->array_size);

			test_dev->lpm_arr =
				kzalloc(sizeof(
				struct lpm_entry_type) *
					tch->
					test_device->array_size,
				GFP_KERNEL);

			if (!test_dev->lpm_arr) {
				pr_err(MODULE_NAME ": %s - "
					"lpm_arr is NULL",
					__func__);
				return -ENOMEM;
			}
		}

		/*
		 * in new device, initialize next_mask_id, and setting
		 * mask_id to the channel
		 */
		test_dev->next_mask_id = 0x1;
		tch->channel_mask_id = test_dev->next_mask_id;
		test_dev->next_mask_id *= 2;
		pr_info(TEST_MODULE_NAME ": %s - channel %s got "
			"read_mask_id = 0x%x. device next_mask_id=0x%x",
			__func__,
			tch->name,
			tch->channel_mask_id,
			test_dev->next_mask_id);
	}

	return 0;
}

static void check_test_result(void)
{
	int result = 1;
	int i = 0;

	test_ctx->max_number_of_devices = 0;

	pr_info(TEST_MODULE_NAME ": %s - Woke Up\n", __func__);

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready))
			continue;

		if (tch->test_type == SDIO_TEST_LPM_RANDOM)
			result &= tch->test_device->final_result_per_dev;
		else
			if (tch->test_result == TEST_FAILED) {
				pr_info(TEST_MODULE_NAME ": %s - "
					"Test FAILED\n", __func__);
				test_ctx->test_result = TEST_FAILED;
				pr_err(TEST_MODULE_NAME ": %s - "
				       "test_result %d",
				       __func__, test_ctx->test_result);
				return;
			}
	}

	if (result == 0) {
		pr_info(TEST_MODULE_NAME ": %s - Test FAILED\n", __func__);
		test_ctx->test_result = TEST_FAILED;
		pr_err(TEST_MODULE_NAME ": %s - "
		       "test_result %d",
		       __func__, test_ctx->test_result);
		return;
	}

	pr_info(TEST_MODULE_NAME ": %s - Test PASSED", __func__);
	test_ctx->test_result = TEST_PASSED;
	pr_err(TEST_MODULE_NAME ": %s - "
	       "test_result %d",
	       __func__, test_ctx->test_result);
	return;
}

/**
 * Test Main
 */
static int test_start(void)
{
	int ret = -ENOMEM;
	int i;

	pr_debug(TEST_MODULE_NAME ":Starting Test ....\n");

	test_ctx->test_completed = 0;
	test_ctx->test_result = TEST_NO_RESULT;
	test_ctx->debug.dun_throughput = 0;
	test_ctx->debug.rmnt_throughput = 0;
	test_ctx->number_of_active_devices = 0;

	pr_err(TEST_MODULE_NAME ": %s - test_result %d",
	       __func__, test_ctx->test_result);

	memset(test_ctx->test_dev_arr, 0,
		sizeof(struct sdio_test_device)*MAX_NUM_OF_SDIO_DEVICES);

	/* Open The Channels */
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used))
			continue;

		tch->rx_bytes = 0;
		tch->tx_bytes = 0;

		atomic_set(&tch->tx_notify_count, 0);
		atomic_set(&tch->rx_notify_count, 0);
		atomic_set(&tch->any_notify_count, 0);
		atomic_set(&tch->wakeup_client, 0);

		/* in case there are values left from previous tests */
		tch->notify_counter_per_chan = 0;

		memset(tch->buf, 0x00, tch->buf_size);
		tch->test_result = TEST_NO_RESULT;

		tch->test_completed = 0;

		if (!tch->ch_ready) {
			pr_info(TEST_MODULE_NAME ":openning channel %s\n",
				tch->name);
			tch->ch_ready = true;
			if (tch->ch_id == SDIO_SMEM) {
				test_ctx->smem_pdev.name = "SDIO_SMEM";
				test_ctx->smem_pdev.dev.release =
					default_sdio_al_test_release;
				platform_device_register(&test_ctx->smem_pdev);
			} else {
				ret = sdio_open(tch->name , &tch->ch, tch,
						notify);
				if (ret) {
					pr_info(TEST_MODULE_NAME
						":openning channel %s failed\n",
					tch->name);
					tch->ch_ready = false;
					continue;
				}
			}
		}

		if (tch->ch_id != SDIO_SMEM) {
			ret = sdio_test_find_dev(tch);

			if (ret) {
				pr_err(TEST_MODULE_NAME ": %s - "
				       "sdio_test_find_dev() returned with "
				       "error", __func__);
				return -ENODEV;
			}

			tch->sdio_al_device = tch->ch->sdio_al_dev;
		}

		if ((tch->test_type == SDIO_TEST_LPM_HOST_WAKER) ||
		    (tch->test_type == SDIO_TEST_LPM_CLIENT_WAKER) ||
		    (tch->test_type == SDIO_TEST_LPM_RANDOM))
			sdio_al_register_lpm_cb(tch->sdio_al_device,
					 sdio_test_wakeup_callback);
	}

	/*
	 * make some space between opening the channels and sending the
	 * config messages
	 */
	msleep(100);

	/*
	 * try to delay send_config_msg of all channels to after the point
	 * when we open them all
	 */
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used))
			continue;

		if ((tch->ch_ready) && (tch->ch_id != SDIO_SMEM))
			send_config_msg(tch);

		if ((tch->test_type == SDIO_TEST_LPM_HOST_WAKER) ||
		    (tch->test_type == SDIO_TEST_LPM_CLIENT_WAKER) ||
		    (tch->test_type == SDIO_TEST_LPM_RANDOM)) {
			if (tch->timer_interval_ms > 0) {
				pr_info(TEST_MODULE_NAME ": %s - init timer, "
					"ms=%d\n",
					__func__, tch->timer_interval_ms);
				init_timer(&tch->timer);
				tch->timer.data = (unsigned long)tch;
				tch->timer.function =
					sdio_test_lpm_timer_handler;
				tch->timer.expires = jiffies +
				    msecs_to_jiffies(tch->timer_interval_ms);
				add_timer(&tch->timer);
			}
		}
	}

	pr_debug(TEST_MODULE_NAME ":queue_work..\n");
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used) || (!tch->ch_ready) ||
		    (tch->ch_id == SDIO_SMEM))
			continue;

		queue_work(tch->workqueue, &tch->test_work.work);
	}

	pr_info(TEST_MODULE_NAME ": %s - Waiting for the test completion\n",
		__func__);

	wait_event(test_ctx->wait_q, test_ctx->test_completed);
	check_test_result();

	/*
	 * Close the channels and zero the is_used flag so that if the modem
	 * will be reset after the test completion we won't re-open
	 * the channels
	 */
	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];

		if ((!tch) || (!tch->is_used))
			continue;
		if ((!tch->ch_ready) ||
		    (tch->ch_id == SDIO_SMEM) || (!tch->ch->is_packet_mode)) {
			tch->is_used = 0;
			continue;
		}
		ret = sdio_close(tch->ch);
		if (ret) {
			pr_err(TEST_MODULE_NAME":%s close channel %s"
					" failed\n", __func__, tch->name);
		} else {
			pr_info(TEST_MODULE_NAME":%s close channel %s"
					" success\n", __func__, tch->name);
			tch->ch_ready = false;
		}
		tch->is_used = 0;
	}
	return 0;
}

static int set_params_loopback_9k(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_LOOPBACK_CLIENT;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->config_msg.num_packets = 10000;
	tch->config_msg.num_iterations = 1;

	tch->packet_length = 512;
	if (tch->ch_id == SDIO_RPC)
		tch->packet_length = 128;
	tch->timer_interval_ms = 0;

	return 0;
}
static int set_params_loopback_9k_close(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_CLOSE_CHANNEL;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->config_msg.num_packets = 5000;
	tch->config_msg.num_iterations = 1;
	tch->max_burst_size = 10;
	switch (tch->ch_id) {
	case SDIO_DUN:
	case SDIO_RPC:
		tch->packet_length = 128; /* max is 2K*/
		break;
	case SDIO_CIQ:
	case SDIO_DIAG:
	case SDIO_RMNT:
	default:
		tch->packet_length = 512; /* max is 4k */
	}
	tch->timer_interval_ms = 0;
	return 0;
}
static int set_params_a2_perf(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_PERF;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->packet_length = MAX_XFER_SIZE;
	if (tch->ch_id == SDIO_DIAG)
		tch->packet_length = 512;
	else if (tch->ch_id == SDIO_DUN)
			tch->packet_length = DUN_PACKET_SIZE;
	pr_info(TEST_MODULE_NAME ": %s: packet_length=%d", __func__,
			tch->packet_length);

	tch->config_msg.num_packets = 10000;
	tch->config_msg.num_iterations = 1;
	tch->random_packet_size = 0;

	tch->timer_interval_ms = 0;

	return 0;
}

static int set_params_rtt(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_RTT;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->packet_length = 32;

	tch->config_msg.num_packets = 200;
	tch->config_msg.num_iterations = 1;
	tch->random_packet_size = 0;

	tch->timer_interval_ms = 0;

	return 0;
}

static int set_params_a2_small_pkts(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_PERF;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->packet_length = 128;

	tch->config_msg.num_packets = 1000000;
	tch->config_msg.num_iterations = 1;
	tch->random_packet_size = 1;

	tch->timer_interval_ms = 0;

	return 0;
}

static int set_params_modem_reset(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_MODEM_RESET;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_LOOPBACK_CLIENT;
	tch->packet_length = 512;
	if (tch->ch_id == SDIO_RPC)
		tch->packet_length = 128;
	else if ((tch->ch_id == SDIO_RMNT) || (tch->ch_id == SDIO_DUN))
		tch->packet_length = MAX_XFER_SIZE;

	tch->config_msg.num_packets = 50000;
	tch->config_msg.num_iterations = 1;

	tch->timer_interval_ms = 0;

	return 0;
}

static int set_params_smem_test(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->timer_interval_ms = 0;

	return 0;
}

static int set_params_lpm_test(struct test_channel *tch,
				enum sdio_test_case_type test,
				int timer_interval_ms)
{
	static int first_time = 1;
	if (!tch) {
		pr_err(TEST_MODULE_NAME ": %s - NULL channel\n", __func__);
		return -EINVAL;
	}

	tch->is_used = 1;
	tch->test_type = test;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = test;
	tch->config_msg.num_packets = LPM_TEST_NUM_OF_PACKETS;
	tch->config_msg.num_iterations = 1;
	tch->timer_interval_ms = timer_interval_ms;
	tch->timeout_ms = 10000;

	tch->packet_length = 0;
	if (test != SDIO_TEST_LPM_RANDOM) {
		init_timer(&tch->timeout_timer);
		tch->timeout_timer.data = (unsigned long)tch;
		tch->timeout_timer.function = sdio_test_lpm_timeout_handler;
		tch->timeout_timer.expires = jiffies +
			msecs_to_jiffies(tch->timeout_ms);
		add_timer(&tch->timeout_timer);
		pr_info(TEST_MODULE_NAME ": %s - Initiated LPM TIMEOUT TIMER."
			"set to %d ms\n",
			__func__, tch->timeout_ms);
	}

	if (first_time) {
		pr_info(TEST_MODULE_NAME ": %s - wake_lock_init() called\n",
		__func__);
		wake_lock_init(&test_ctx->wake_lock,
			       WAKE_LOCK_SUSPEND, TEST_MODULE_NAME);
		first_time = 0;
	}

	pr_info(TEST_MODULE_NAME ": %s - wake_lock() for the TEST is "
		"called channel %s. to prevent real sleeping\n",
		__func__, tch->name);
	wake_lock(&test_ctx->wake_lock);

	return 0;
}

static int set_params_8k_sender_no_lp(struct test_channel *tch)
{
	if (!tch) {
		pr_err(TEST_MODULE_NAME ":NULL channel\n");
		return -EINVAL;
	}
	tch->is_used = 1;
	tch->test_type = SDIO_TEST_HOST_SENDER_NO_LP;
	tch->config_msg.signature = TEST_CONFIG_SIGNATURE;
	tch->config_msg.test_case = SDIO_TEST_HOST_SENDER_NO_LP;
	tch->config_msg.num_packets = 1000;
	tch->config_msg.num_iterations = 1;

	tch->packet_length = 512;
	if (tch->ch_id == SDIO_RPC)
		tch->packet_length = 128;
	tch->timer_interval_ms = 0;

	return 0;
}

static void set_pseudo_random_seed(void)
{
	/* Set the seed accoring to the kernel command parameters if any or
	   get a random value */
	if (seed != 0) {
		test_ctx->lpm_pseudo_random_seed = seed;
	} else {
		test_ctx->lpm_pseudo_random_seed =
			(unsigned int)(get_jiffies_64() & 0xFFFF);
		test_ctx->lpm_pseudo_random_seed =
			pseudo_random_seed(&test_ctx->lpm_pseudo_random_seed);
	}

	pr_info(TEST_MODULE_NAME ":%s: seed is %u",
		   __func__, test_ctx->lpm_pseudo_random_seed);
}

/*
   for each channel
   1. open channel
   2. close channel
   3. Run lpm Host waker test on packet channel
*/
#define MAX_LPM_CH_ARR 4
static int close_channel_lpm_test(void)
{
	int ret = 0;
	struct test_channel *tch = NULL;
	int i;
	int lpm_ch_arr[] = { SDIO_RPC,
			SDIO_QMI,
			SDIO_DIAG,
			SDIO_CIQ};

	for (i = 0; i < MAX_LPM_CH_ARR; i++) {
		tch = test_ctx->test_ch_arr[lpm_ch_arr[i]];

		/* sdio_close is not implemented for stream ch */
		if ((!tch) ||
			(tch->ch_id == SDIO_DUN) ||
			(tch->ch_id == SDIO_RMNT))
			continue;

		if (!tch->ch_ready) {
			ret = sdio_open(tch->name , &tch->ch, tch, notify);
			if (ret) {
				pr_err(TEST_MODULE_NAME":%s open channel %s"
					" failed\n", __func__, tch->name);
				return ret;
			} else {
				pr_info(TEST_MODULE_NAME":%s open channel %s"
					" success\n", __func__, tch->name);
			}
		}
		ret = sdio_close(tch->ch);
		if (ret) {
			pr_err(TEST_MODULE_NAME":%s close channel %s"
					" failed\n", __func__, tch->name);
			return ret;
		} else {
			pr_info(TEST_MODULE_NAME":%s close channel %s"
					" success\n", __func__, tch->name);
			tch->ch_ready = false;
		}
		if (tch->ch->is_packet_mode) {
			set_params_lpm_test(tch, SDIO_TEST_LPM_HOST_WAKER, 120);

			ret = test_start();
			if (ret) {
				pr_err(TEST_MODULE_NAME ":test_start failed, "
					"ret = %d.\n", ret);
				return ret;
			}
		}
		tch->is_used = 0;

	}
	return ret;
}

/**
 * Write File.
 *
 * @note Trigger the test from user space by:
 * echo 1 > /dev/sdio_al_test
 *
 */
ssize_t test_write(struct file *filp, const char __user *buf, size_t size,
		   loff_t *f_pos)
{
	int ret = 0;
	int i;
	struct test_channel **ch_arr = test_ctx->test_ch_arr;

	for (i = 0 ; i < MAX_NUM_OF_SDIO_DEVICES ; ++i)
			test_ctx->test_dev_arr[i].sdio_al_device = NULL;

	ret = strict_strtol(buf, 10, &test_ctx->testcase);

	switch (test_ctx->testcase) {
	case 1:
		/* RPC */
		pr_debug(TEST_MODULE_NAME " --RPC sender--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]))
			return size;
		break;
	case 2:
		/* RPC, QMI and DIAG */
		pr_debug(TEST_MODULE_NAME " --RPC, QMI and DIAG sender--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_DIAG]))
			return size;
		break;
	case 4:
		pr_debug(TEST_MODULE_NAME " --SMEM--.\n");
		if (set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]))
			return size;
		break;

	case 5:
		pr_debug(TEST_MODULE_NAME " --SMEM and RPC--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]))
			return size;
		break;
	case 6:
		pr_debug(TEST_MODULE_NAME " --RmNet A2 Performance--.\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;

	case 7:
		pr_debug(TEST_MODULE_NAME " --DUN A2 Performance--.\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]))
			return size;
		break;
	case 8:
		pr_debug(TEST_MODULE_NAME " --RmNet and DUN A2 Performance--."
					  "\n");
		if (set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]))
			return size;
		break;
	case 9:
		pr_debug(TEST_MODULE_NAME " --RPC sender and RmNet A2 "
					  "Performance--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;
	case 10:
		pr_debug(TEST_MODULE_NAME " --All the channels--.\n");
		if (set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_DIAG]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_a2_perf(test_ctx->test_ch_arr[SDIO_DUN]) ||
		    set_params_smem_test(test_ctx->test_ch_arr[SDIO_SMEM]) ||
		    set_params_loopback_9k(test_ctx->test_ch_arr[SDIO_CIQ]))
			return size;
		break;
	case 16:
		pr_info(TEST_MODULE_NAME " -- host sender no LP for Diag --");
		set_params_8k_sender_no_lp(test_ctx->test_ch_arr[SDIO_DIAG]);
		break;
	case 17:
		pr_info(TEST_MODULE_NAME " -- host sender no LP for Diag, RPC, "
					 "CIQ  --");
		set_params_8k_sender_no_lp(test_ctx->test_ch_arr[SDIO_DIAG]);
		set_params_8k_sender_no_lp(test_ctx->test_ch_arr[SDIO_CIQ]);
		set_params_8k_sender_no_lp(test_ctx->test_ch_arr[SDIO_RPC]);
		break;
	case 18:
		pr_info(TEST_MODULE_NAME " -- rmnet small packets (5-128)  --");
		if (set_params_a2_small_pkts(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;
	case 19:
		pr_info(TEST_MODULE_NAME " -- rmnet RTT --");
		if (set_params_rtt(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;
	case 22:
		pr_info(TEST_MODULE_NAME " -- modem reset - RPC --");
		if (set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RPC]))
			return size;
		break;
	case 23:
		pr_info(TEST_MODULE_NAME " -- modem reset - RMNT --");
		if (set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RMNT]))
			return size;
		break;
	case 24:
		pr_info(TEST_MODULE_NAME " -- modem reset - all chs "
					"4bit device --");
		if (set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_DIAG]))
			return size;
		break;
	case 25:
		pr_info(TEST_MODULE_NAME " -- modem reset - all chs "
					"8bit device--");
		if (set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_DUN]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_CIQ]))
			return size;
		break;
	case 26:
		pr_info(TEST_MODULE_NAME " -- modem reset - all chs --");
		if (set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RPC]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_QMI]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_DIAG]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_RMNT]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_DUN]) ||
		    set_params_modem_reset(test_ctx->test_ch_arr[SDIO_CIQ]))
			return size;
		break;
	case 27:
		pr_info(TEST_MODULE_NAME " -- host sender with open/close for "
				"Diag, CIQ and RPC --");
		if (set_params_loopback_9k_close(ch_arr[SDIO_DIAG]) ||
		    set_params_loopback_9k_close(ch_arr[SDIO_CIQ]) ||
		    set_params_loopback_9k_close(ch_arr[SDIO_RPC]))
			return size;
		break;
	case 28:
		pr_info(TEST_MODULE_NAME " -- Close channel & LPM Test "
			"Host wakes the Client --\n");
		ret = close_channel_lpm_test();
		if (ret) {
			pr_err(TEST_MODULE_NAME " -- Close channel & LPM Test "
					"FAILED: %d --\n", ret);
		} else {
			pr_err(TEST_MODULE_NAME " -- Close channel & LPM Test "
					"PASSED\n");
		}
		return size;
	case 111:
		pr_info(TEST_MODULE_NAME " --LPM Test For Device 1. Client "
			"wakes the Host --.\n");
		if (set_params_lpm_test(test_ctx->test_ch_arr[SDIO_RPC],
				    SDIO_TEST_LPM_CLIENT_WAKER, 90))
			return size;
		break;
	case 113:
		pr_info(TEST_MODULE_NAME " --LPM Test For Device 1. Host "
			"wakes the Client --.\n");
		if (set_params_lpm_test(test_ctx->test_ch_arr[SDIO_RPC],
				    SDIO_TEST_LPM_HOST_WAKER, 120))
			return size;
		break;
	case 114:
		pr_info(TEST_MODULE_NAME " --LPM Test RANDOM SINGLE "
					 "CHANNEL--.\n");
		set_pseudo_random_seed();
		if (set_params_lpm_test(test_ctx->test_ch_arr[SDIO_RPC],
					    SDIO_TEST_LPM_RANDOM, 0))
			return size;
		break;
	case 115:
		pr_info(TEST_MODULE_NAME " --LPM Test RANDOM MULTI "
					 "CHANNEL--.\n");
		set_pseudo_random_seed();
		if (set_params_lpm_test(test_ctx->test_ch_arr[SDIO_RPC],
				    SDIO_TEST_LPM_RANDOM, 0) ||
		    set_params_lpm_test(test_ctx->test_ch_arr[SDIO_CIQ],
					SDIO_TEST_LPM_RANDOM, 0) ||
		    set_params_lpm_test(test_ctx->test_ch_arr[SDIO_DIAG],
					SDIO_TEST_LPM_RANDOM, 0) ||
		    set_params_lpm_test(test_ctx->test_ch_arr[SDIO_QMI],
				    SDIO_TEST_LPM_RANDOM, 0))
			return size;
		break;
	case 98:
		pr_info(TEST_MODULE_NAME " set runtime debug on");
		test_ctx->runtime_debug = 1;
		return size;
	case 99:
		pr_info(TEST_MODULE_NAME " set runtime debug off");
		test_ctx->runtime_debug = 0;
		return size;
	default:
		pr_info(TEST_MODULE_NAME ":Bad Test number = %d.\n",
			(int)test_ctx->testcase);
		return size;
	}
	ret = test_start();
	if (ret) {
		pr_err(TEST_MODULE_NAME ":test_start failed, ret = %d.\n",
			ret);
		return size;
	}
	/* combined test cases */
	switch (test_ctx->testcase) {
	case 27:
		pr_info(TEST_MODULE_NAME " -- correctness test for"
				"DIAG, CIQ and DUN ");
		if (set_params_loopback_9k(ch_arr[SDIO_DIAG]) ||
		    set_params_loopback_9k(ch_arr[SDIO_CIQ]) ||
		    set_params_loopback_9k(ch_arr[SDIO_RPC]))
			return size;
		break;
	default:
		pr_debug(TEST_MODULE_NAME ":2nd test not defined for %ld\n",
				test_ctx->testcase);
		return size;
	}
	ret = test_start();
	if (ret) {
		pr_err(TEST_MODULE_NAME ":2nd round test failed (%d)\n",
				ret);
	}
	return size;
}

/**
 * Test Channel Init.
 */
int test_channel_init(char *name)
{
	struct test_channel *test_ch;
	int ch_id = 0;
	int ret;

	pr_debug(TEST_MODULE_NAME ":%s.\n", __func__);
	pr_info(TEST_MODULE_NAME ": init test cahnnel %s.\n", name);

	ch_id = channel_name_to_id(name);
	pr_debug(TEST_MODULE_NAME ":id = %d.\n", ch_id);
	if (test_ctx->test_ch_arr[ch_id] == NULL) {
		test_ch = kzalloc(sizeof(*test_ch), GFP_KERNEL);
		if (test_ch == NULL) {
			pr_err(TEST_MODULE_NAME ":kzalloc err for allocating "
						"test_ch %s.\n",
			       name);
			return -ENOMEM;
		}
		test_ctx->test_ch_arr[ch_id] = test_ch;

		test_ch->ch_id = ch_id;

		strncpy(test_ch->name, name,
		       strnlen(name, TEST_CH_NAME_SIZE)-SDIO_TEST_POSTFIX_SIZE);

		test_ch->buf_size = MAX_XFER_SIZE;

		test_ch->buf = kzalloc(test_ch->buf_size, GFP_KERNEL);
		if (test_ch->buf == NULL) {
			kfree(test_ch);
			test_ctx->test_ch = NULL;
			return -ENOMEM;
		}

		if (test_ch->ch_id == SDIO_SMEM) {
			test_ctx->smem_buf = kzalloc(SMEM_MAX_XFER_SIZE,
						     GFP_KERNEL);
			if (test_ctx->smem_buf == NULL) {
				pr_err(TEST_MODULE_NAME ":%s: Unable to "
							"allocate smem buf\n",
				       __func__);
				kfree(test_ch);
				test_ctx->test_ch = NULL;
				return -ENOMEM;
			}

#ifdef CONFIG_MSM_SDIO_SMEM
			ret = platform_driver_register(&sdio_smem_client_drv);
			if (ret) {
				pr_err(TEST_MODULE_NAME ":%s: Unable to "
							"register sdio smem "
							"test client\n",
				       __func__);
				return ret;
			}
#endif
		} else {
			test_ch->workqueue =
				create_singlethread_workqueue(test_ch->name);
			test_ch->test_work.test_ch = test_ch;
			INIT_WORK(&test_ch->test_work.work, worker);

			init_waitqueue_head(&test_ch->wait_q);
		}
	} else {
		test_ch = test_ctx->test_ch_arr[ch_id];
		pr_info(TEST_MODULE_NAME ":%s: ch %s was detected again\n",
			__func__, test_ch->name);
		test_ch->card_removed = 0;
		if ((test_ch->is_used) &&
		    (test_ch->test_type == SDIO_TEST_MODEM_RESET)) {
			if (test_ch->ch_id == SDIO_SMEM) {
				test_ctx->smem_pdev.name = "SDIO_SMEM";
				test_ctx->smem_pdev.dev.release =
					default_sdio_al_test_release;
				platform_device_register(&test_ctx->smem_pdev);
			} else {
				test_ch->ch_ready = true;
				ret = sdio_open(test_ch->name , &test_ch->ch,
						test_ch, notify);
				if (ret) {
					pr_info(TEST_MODULE_NAME
						":openning channel %s failed\n",
					test_ch->name);
					test_ch->ch_ready = false;
					return 0;
				}
				ret = sdio_test_find_dev(test_ch);

				if (ret) {
					pr_err(TEST_MODULE_NAME ": %s - "
					       "sdio_test_find_dev() returned "
					       "with error", __func__);
					return -ENODEV;
				}

				test_ch->sdio_al_device =
					test_ch->ch->sdio_al_dev;
			}
			atomic_set(&test_ch->card_detected_event, 1);
			wake_up(&test_ch->wait_q);
		}
	}

	return 0;
}

static int sdio_test_channel_probe(struct platform_device *pdev)
{
	if (!pdev)
		return -EIO;
	return test_channel_init((char *)pdev->name);
}

static int sdio_test_channel_remove(struct platform_device *pdev)
{
	int ch_id;

	if (!pdev)
		return -EIO;

	ch_id = channel_name_to_id((char *)pdev->name);
	if (test_ctx->test_ch_arr[ch_id] == NULL)
		return 0;

	pr_info(TEST_MODULE_NAME "%s: remove ch %s\n",
		__func__, test_ctx->test_ch_arr[ch_id]->name);
	test_ctx->test_ch_arr[ch_id]->ch_ready = 0;
	test_ctx->test_ch_arr[ch_id]->card_removed = 1;

	return 0;

}

static struct platform_driver sdio_rpc_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_RPC_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_qmi_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_QMI_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_diag_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_DIAG_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_smem_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_SMEM_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_rmnt_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_RMNT_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_dun_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_DUN_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct platform_driver sdio_ciq_drv = {
	.probe		= sdio_test_channel_probe,
	.remove		= sdio_test_channel_remove,
	.driver		= {
		.name	= "SDIO_CIQ_TEST",
		.owner	= THIS_MODULE,
	},
};

static struct class *test_class;

const struct file_operations test_fops = {
	.owner = THIS_MODULE,
	.write = test_write,
};

/**
 * Module Init.
 */
static int __init test_init(void)
{
	int ret;

	pr_debug(TEST_MODULE_NAME ":test_init.\n");

	test_ctx = kzalloc(sizeof(struct test_context), GFP_KERNEL);

	if (test_ctx == NULL) {
		pr_err(TEST_MODULE_NAME ":kzalloc err.\n");
		return -ENOMEM;
	}
	test_ctx->test_ch = NULL;
	test_ctx->signature = TEST_SIGNATURE;

	test_ctx->name = "UNKNOWN";

	init_waitqueue_head(&test_ctx->wait_q);

#ifdef CONFIG_DEBUG_FS
	sdio_al_test_debugfs_init();
#endif

	test_class = class_create(THIS_MODULE, TEST_MODULE_NAME);

	ret = alloc_chrdev_region(&test_ctx->dev_num, 0, 1, TEST_MODULE_NAME);
	if (ret) {
		pr_err(TEST_MODULE_NAME "alloc_chrdev_region err.\n");
		return -ENODEV;
	}

	test_ctx->dev = device_create(test_class, NULL, test_ctx->dev_num,
				      test_ctx, TEST_MODULE_NAME);
	if (IS_ERR(test_ctx->dev)) {
		pr_err(TEST_MODULE_NAME ":device_create err.\n");
		return -ENODEV;
	}

	test_ctx->cdev = cdev_alloc();
	if (test_ctx->cdev == NULL) {
		pr_err(TEST_MODULE_NAME ":cdev_alloc err.\n");
		return -ENODEV;
	}
	cdev_init(test_ctx->cdev, &test_fops);
	test_ctx->cdev->owner = THIS_MODULE;

	ret = cdev_add(test_ctx->cdev, test_ctx->dev_num, 1);
	if (ret)
		pr_err(TEST_MODULE_NAME ":cdev_add err=%d\n", -ret);
	else
		pr_debug(TEST_MODULE_NAME ":SDIO-AL-Test init OK..\n");

	platform_driver_register(&sdio_rpc_drv);
	platform_driver_register(&sdio_qmi_drv);
	platform_driver_register(&sdio_diag_drv);
	platform_driver_register(&sdio_smem_drv);
	platform_driver_register(&sdio_rmnt_drv);
	platform_driver_register(&sdio_dun_drv);
	platform_driver_register(&sdio_ciq_drv);

	return ret;
}

/**
 * Module Exit.
 */
static void __exit test_exit(void)
{
	int i;

	pr_debug(TEST_MODULE_NAME ":test_exit.\n");

	test_ctx->exit_flag = true;

	msleep(100); /* allow gracefully exit of the worker thread */

	cdev_del(test_ctx->cdev);
	device_destroy(test_class, test_ctx->dev_num);
	unregister_chrdev_region(test_ctx->dev_num, 1);

	platform_driver_unregister(&sdio_rpc_drv);
	platform_driver_unregister(&sdio_qmi_drv);
	platform_driver_unregister(&sdio_diag_drv);
	platform_driver_unregister(&sdio_smem_drv);
	platform_driver_unregister(&sdio_rmnt_drv);
	platform_driver_unregister(&sdio_dun_drv);
	platform_driver_unregister(&sdio_ciq_drv);

	for (i = 0; i < SDIO_MAX_CHANNELS; i++) {
		struct test_channel *tch = test_ctx->test_ch_arr[i];
		if (!tch)
			continue;
		kfree(tch->buf);
		kfree(tch);
	}

#ifdef CONFIG_DEBUG_FS
	sdio_al_test_debugfs_cleanup();
#endif

	kfree(test_ctx);

	pr_debug(TEST_MODULE_NAME ":test_exit complete.\n");
}

module_init(test_init);
module_exit(test_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SDIO_AL Test");
MODULE_AUTHOR("Amir Samuelov <amirs@codeaurora.org>");


