#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/completion.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/ioctl.h>

/* 先頭の #include 群の下あたりに追記 */
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/atomic.h>

#include "config.h"
#include "driver_define.h"
#include "logging.h"

#define VENDOR_ID 0x1962
#define PRODUCT_ID 0x2080

#define MINOR_BASE 192			// マイナー番号

#define MAX_PROC_TEXT_SIZE 512

static atomic_t beuato_dbg = ATOMIC_INIT(0);

/* === 受信完了通知用 === */
static DECLARE_WAIT_QUEUE_HEAD(beuato_waitq);
static atomic_t beuato_data_ready = ATOMIC_INIT(0);


/** プロトタイプ宣言 **/ 
int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID);
void skel_disconnect(struct usb_interface* ip);
int skel_open(struct inode *inode, struct file *file);
int skel_release(struct inode *inode, struct file *file);
ssize_t skel_read (struct file* file, char __user *buff, size_t count, loff_t *f_pos) ;
ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos);
__poll_t skel_poll(struct file *file, poll_table *wait);
long skel_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
int skel_flush(struct file *file, fl_owner_t id);

static void urb_in_complete(struct urb* urb);


/** ベンダーIDとプロダクトIDの登録 **/ 
struct usb_device_id skel_table[] = {
	{USB_DEVICE(VENDOR_ID, PRODUCT_ID)},
	{}
};

/** ドライバ初期化時の処理の登録 **/
struct usb_driver skel_driver = {
//	.name = "BeuatoBalancer Driver",
	.name = "BeuatoBalancer_Driver",
	.id_table = skel_table,
	.probe = skel_probe,					// 最初に実行される関数
	.disconnect = skel_disconnect		// 切断時に実行される関数
};


/** ファイルオペレーションの登録 **/
struct file_operations skel_fops = {
	.owner = THIS_MODULE,
	.open = skel_open,
	.release = skel_release,
	.read = skel_read,
	.write = skel_write,
	.poll = skel_poll,
	.unlocked_ioctl = skel_ioctl,
	.flush = skel_flush,
};

/** ドライバクラスの登録 */
struct usb_class_driver skel_class = {
	.name = "usb/BeuatoCtrl%d",
	.fops = &skel_fops,
	.minor_base = MINOR_BASE
};


/** USB-HID-INの制御 
 **/

struct user_read_result read_results = {
	.command = 0,
	.addr = 0,
	.buffer = NULL,
	.buffer_length = 0,
	.datasize = 0,
	.ready_to_return = 0,
};

static const int MAX_IN_TEXT_SIZE = 64;
static char line_buf[MAX_PROC_TEXT_SIZE];
static size_t line_pos=0, line_len=0;


static int init_read_results(struct user_read_result *r)
{
	r->command = 0;
	r->addr = 0;
	if(!r->buffer) {
		r->buffer = kmalloc(MAX_IN_TEXT_SIZE, GFP_KERNEL);
		if(!r->buffer) {
			DMESG_DEBUG("no memory");
			return -ENOMEM;
		}
	}
	r->buffer_length = MAX_IN_TEXT_SIZE;
	DMESG_DEBUG("read_results initialzed.");
	return 0;
}

static int free_read_results(struct user_read_result *r)
{
	if(r->buffer) {
		kfree(r->buffer);
	}
	r->command = 0;
	r->addr = 0;
	r->buffer = NULL;
	r->buffer_length = 0;
	DMESG_DEBUG("read_results freeed.");
	return 0;
}

inline void clear_read_results(struct user_read_result *r)
{
	if(r->ready_to_return)
		DMESG_DEBUG("unsent data remains in read_results but cleard.");
	r->command = 0;
	r->addr = 0;
	r->datasize=0;
	r->ready_to_return=0;
	DMESG_DEBUG("read_results cleared (not freeed).");
}

static void clear_device_state(struct usb_skel *pDev)
{
	pDev->received_len = 0;
	pDev->expected_len = 0;
	clear_read_results(&read_results);
	atomic_set(&beuato_data_ready, 0);
}


/** データ読み込み開始 **/
int run_read(struct usb_skel* pDev) 
{
	// urb for data receive from device
	usb_fill_int_urb(pDev->int_in_urb, pDev->udev, 
		usb_rcvintpipe(pDev->udev, pDev->int_in_endpoint->bEndpointAddress),
		pDev->int_in_buffer,
		pDev->int_in_buffer_length,
		urb_in_complete,
		pDev,
		pDev->int_in_endpoint->bInterval
	);
	pDev->int_in_urb->transfer_flags |= URB_SHORT_NOT_OK;
        DMESG_DEBUG("read/bInterval:%d\n",pDev->int_in_endpoint->bInterval);
	int ret = usb_submit_urb(pDev->int_in_urb, GFP_ATOMIC);
	if(ret) {
		if(ret == -EBUSY)
			DMESG_DEBUG("run_read: urb submit BUSY.\n")
		else
			DMESG_DEBUG("run_read: urb submit Other err(%d).\n", ret)
		usb_unanchor_urb(pDev->int_in_urb);
		DMESG_DEBUG("no submit and return\n");
		return ret;
	}

	usb_anchor_urb(pDev->int_in_urb, &(pDev->submitted));
	DMESG_DEBUG("submit and anchor done.\n");

	return 0;
}


static void skel_dispose(struct kref* pKref) 
{
	struct usb_skel* pDev = container_of(pKref, struct usb_skel, kref);
	usb_put_dev(pDev->udev);
	kfree(pDev);
	DMESG_DEBUG("Disposed.");
}

/* デバイスオープン時に実行 */
int skel_open(struct inode *inode, struct file *file)
{
	DMESG_INFO("Device open\n");

	struct usb_interface* ip = usb_find_interface(&skel_driver, iminor(inode));
	struct usb_skel* pDev = usb_get_intfdata(ip);

	kref_get(&pDev->kref);
	file->private_data = (void*)pDev;

	//
	// <<already read_results allocated in skel_probe so no need to allocate here>>
	//
	//int errno = init_read_results(&read_results);
	//if(errno)
	//	return errno;

	clear_device_state(pDev);
	line_pos = line_len = 0; //reset
        pDev->mode = BEUATO_MODE_BINARY; // reset to binary mode at every open

	run_read(pDev);

	DMESG_DEBUG("successfully opened.");
	return 0;
}


/* デバイスリリース時に実行 */
int skel_release(struct inode *inode, struct file *file)
{
	DMESG_INFO("Device release\n");

	struct usb_skel* pDev = file->private_data;
	//free_read_results(&read_results); // no need to free here.

	kref_put(&pDev->kref, skel_dispose);

	DMESG_DEBUG("successfully released.\n");
	return 0;
}




/* データの解釈 */
void report_in_handler(unsigned char *buf, int length)
{
	if( buf[0] == 'r' || buf[0] == 'w') {
		const u8* data;
		int datasize;
		char command = buf[0];
		int addr = 0;
	        if( buf[0] == 'r' ) {
			DMESG_DEBUG("[I] <Read> Message:%d\n", length);
			datasize = buf[1];
			data = (const u8*)&buf[2];
		}
	        else if( buf[0] == 'w' )	{
			DMESG_DEBUG("[I] <Write> Message:%d\n", length);
			addr = buf[1];
			datasize = buf[2];
			data = (const u8*)&buf[3];
		}

		DMESG_DEBUG("Contained Data Size:%d\n", datasize);
		if(datasize >= read_results.buffer_length)
		{
			DMESG_ERR("Buffer Overflow\n");
			DMESG_ERR("¥tbuffer_len:%d datasize:%d\n",read_results.buffer_length, datasize);
			return;
		}

		read_results.command = command;
		read_results.addr = addr;
		read_results.datasize = datasize;

		DMESG_DEBUG_DUMP("Contained Data: ", data, datasize, true);
		//print_hex_dump(KERN_DEBUG, "Contained Data:", KERN_CONT, 16, 1, (const u8*)&(buf[2]), datasize, true);

		for(int i = 0 ; i < read_results.datasize; ++i)
		{
			//printk(KERN_CONT "%x ", buf[i+2]);
			read_results.buffer[i] = data[i];
		}
		read_results.ready_to_return = 1;
	}
	else {
		DMESG_DEBUG("NOT 'r/w' of reported buf in report_in_handler.\n");
		DMESG_DEBUG("¥t data: %02x\n", buf[0]);
	}

	DMESG_DEBUG("Report Handled.\n");
}

/** HID-INデータ完了通知 **/
static void urb_in_complete(struct urb* urb)
{
	struct usb_skel *pDev = urb->context;

	usb_unanchor_urb(urb); // unanchor first

	DMESG_DEBUG("urb_in_complete: actual_length: %d", urb->actual_length);
	if(urb->actual_length > 64) {
                DMESG_INFO("Longer packet received(len:%d):\n", urb->actual_length);
		print_hex_dump(KERN_INFO, "     received:", DUMP_PREFIX_NONE, 64, 1, (const u8*)(urb->transfer_buffer), urb->actual_length, true);
	}

	/* 1) エラー／切断ステータスは再投入せずに即終了 */
	if (urb->status == -ECONNRESET ||
	        urb->status == -ENOENT   ||
	        urb->status == -ESHUTDOWN) {
	        DMESG_DEBUG("urb shutting down: %d\n", urb->status);
	        return;
	}
	else if (urb->status == -EREMOTEIO ) {
		DMESG_DEBUG("urb SHORT packet.(len:%d)", urb->actual_length);
		// continue following process including urb resubmit.
	}
	else if (urb->status != 0) {
	    DMESG_DEBUG("urb error %d\n", urb->status);
	    return;
	}

	/* 2) 長さ０なら再投入だけ */
	if (urb->actual_length == 0) {
		DMESG_DEBUG("0 byte actual_length...");
		run_read(pDev);
		/*
		int ret = usb_submit_urb(urb, GFP_ATOMIC);
		switch(ret) {
		case 0:
			usb_anchor_urb(urb, &(pDev->submitted)); // anchor again
			break;
		case -EBUSY:
			DMESG_DEBUG("urb_in_complete(0byte): urb submit BUSY.");
			break;
		default:
			DMESG_DEBUG("urb_in_complete(0byte): urb submit Other err.");
			break;
		}
		*/
		return;
	}
	/* 3) 正常受信パス */
	DMESG_DEBUG("[IN] actual=%d max=%d",
	            urb->actual_length, urb->transfer_buffer_length);
	if (pDev->received_len + urb->actual_length > sizeof(pDev->bin_buf)) {
		DMESG_DEBUG("Rx overflow, drop\n");
		pDev->received_len = 0;
	}
	else {
		//print_hex_dump(KERN_DEBUG, "Received Data:", DUMP_PREFIX_NONE, 16, 1, (const u8*)(urb->transfer_buffer), urb->actual_length, true);
		DMESG_DEBUG_DUMP("Received Data: ", (const u8*)(urb->transfer_buffer), urb->actual_length, true);
                //DMESG_DEBUG("Bin_buf(before cpy): len=%d", pDev->received_len);
		if(pDev->received_len > 0) {
			DMESG_DEBUG_DUMP("Bin_buf(before %d cpy) contained:", (const u8*)(pDev->bin_buf), pDev->received_len, true, urb->actual_length);
			//print_hex_dump(KERN_DEBUG, " data:", DUMP_PREFIX_NONE, 8, 1, (const u8*)(pDev->bin_buf), pDev->received_len, true);
		}
		else {
			DMESG_DEBUG("length received so far: %d", pDev->received_len);
		}
		memcpy(pDev->bin_buf + pDev->received_len,
			urb->transfer_buffer, urb->actual_length);
		pDev->received_len += urb->actual_length;
		DMESG_DEBUG_DUMP( "Bin_buf(after %d cpy) contained:", (const u8*)(pDev->bin_buf), pDev->received_len, true, urb->actual_length);
		//print_hex_dump(KERN_DEBUG, "     Bin_buf(after):", DUMP_PREFIX_NONE, 8, 1, (const u8*)(pDev->bin_buf), pDev->received_len, true);
	}

	if (pDev->expected_len == 0) {
		if(pDev->bin_buf[0] == 'r') {
			// read : 'r'+<len>+<data>...
			if(pDev->received_len >= 2)
				pDev->expected_len = pDev->bin_buf[1] + 2;
		}
		else if(pDev->bin_buf[0] == 'w') {
			// write: 'w'+<addr>+<len>+<data>...
			if(pDev->received_len >= 3)
				pDev->expected_len = pDev->bin_buf[2] + 3;
		}
	}

	DMESG_DEBUG("MultiTX Check: actual_length:%d expected_length:%d", urb->actual_length, pDev->expected_len);

	if (pDev->received_len >= pDev->expected_len) {
		clear_read_results(&read_results);
		report_in_handler(pDev->bin_buf, pDev->expected_len);

		if (read_results.datasize == 0) {
			DMESG_DEBUG("read_results.datasize == 0.???");
		}
		atomic_set(&beuato_data_ready, 1);
		wake_up_interruptible(&beuato_waitq);
		DMESG_DEBUG("wake up read system call.\n");

		pDev->received_len = 0;
		pDev->expected_len = 0;
	}
	else {
		DMESG_DEBUG("Following packet expected: expected_len:%d received_len:%d\n", pDev->expected_len, pDev->received_len);
	}
	/* 4) 最後にまとめて再投入 */
	run_read(pDev);
	/*
	int ret = usb_submit_urb(urb, GFP_ATOMIC);
	switch(ret) {
	case 0:
		usb_anchor_urb(urb, &(pDev->submitted)); // anchor again
		break;
	case -EBUSY:
		DMESG_DEBUG("urb_in_complete(last): urb submit BUSY.");
		break;
	default:
		DMESG_DEBUG("urb_in_complete(last): urb submit Other err.");
		break;
	}
	*/
	DMESG_DEBUG("urb_in callback successfully done.\n");
}



/*
void urb_in_complete(struct urb* urb) 
{
	if( urb->actual_length == 0 ){
		DMESG_ERR("0 byte actual_length in urb.\n");
		// 0byte responce received so resubmit urb for poring
		usb_submit_urb(urb, GFP_ATOMIC);
		return;
        }

	switch (urb->status) {
	case 0:
                DMESG_DEBUG("[IN] actual=%d / max=%d",
			urb->actual_length, urb->transfer_buffer_length);

		struct usb_skel *pDev = urb->context;
		if(pDev->received_len + urb->actual_length > sizeof(pDev->bin_buf)) {
			DMESG_ERR("Rx overflow, packet dropped\n");
			pDev->received_len = 0;
			goto resubmit;
		}
		memcpy(pDev->bin_buf + pDev->received_len,
			urb->transfer_buffer, urb->actual_length);
		pDev->received_len += urb->actual_length;
		if(pDev->expected_len == 0 && urb->actual_length >= 2) {
			pDev->expected_len = ((u8 *)urb->transfer_buffer)[1] + 2;
		}

		if(pDev->received_len < pDev->expected_len)
			goto resubmit;

		size_t true_len = 2 + pDev->bin_buf[1];
		DMESG_DEBUG("true_len:%d\n", true_len);
		clear_read_results(&read_results);
		report_in_handler(pDev->bin_buf, true_len);
		DMESG_DEBUG("Urb load rom Success");

		if( read_results.datasize > 0 ) {
			// wakeup
			atomic_set(&beuato_data_ready, 1);
			wake_up_interruptible(&beuato_waitq);
		}
		pDev->received_len = 0;
		pDev->expected_len = 0;
		goto resubmit;

	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		DMESG_ERR("urb shutting down with %d\n", urb->status);
		goto no_resubmit;
	default:
		DMESG_ERR("urb status %d received\n", urb->status);	
		goto no_resubmit;
	}

resubmit:
	usb_submit_urb(urb, GFP_ATOMIC);
	DMESG_DEBUG("urb submitted");

no_resubmit:
	DMESG_DEBUG("Completed");

}
*/


/** データ読み込み準備 **/
int prepare_read(struct usb_interface* ip, const struct usb_device_id* pID, struct usb_skel* pDev) 
{
	// in urb
	pDev->int_in_urb = usb_alloc_urb(0, GFP_KERNEL); // urb for receive
	if(!pDev->int_in_urb) 
	{
		DMESG_ERR("Memory allocation failed");
		return -1;
	}

	pDev->int_in_buffer = kmalloc(pDev->int_in_buffer_length, GFP_KERNEL);
	memset(pDev->int_in_buffer, 0, pDev->int_in_buffer_length);
	DMESG_DEBUG("Allocation");

	// received data buffer memory
	int errno = init_read_results(&read_results);
	if(errno)
		return errno;
	clear_device_state(pDev);
	return 0;
}

int free_prepare_read(struct usb_skel* pDev)
{
	if(pDev->int_in_urb) {
		usb_free_urb(pDev->int_in_urb);
		pDev->int_in_urb = NULL;
	}
	if(pDev->int_in_buffer) {
		kfree(pDev->int_in_buffer);
		pDev->int_in_buffer = NULL;
	}
	return 0;
}



/** USB-HID-OUTの制御 
 * 
 * 
 * 
 * **/

/* データの解釈 */
void report_out_handler(u8 *buf, int length)
{
	switch (buf[0])
	{
		case 'r':
			DMESG_DEBUG("[O] Read Message: \n");
			//printk("Data:");
			DMESG_DEBUG("length: %d\n", length);
			for(int i = 0 ; i < length; i ++)
			{
				//printk(KERN_CONT "%x ", buf[i]);
				DMESG_DEBUG("%x ", buf[i]);
			}
			break;
		case 'w':
			DMESG_DEBUG("[O] Write Message: \n");
			//printk("Data:");
			DMESG_DEBUG("length: %d\n", length);
			for(int i = 0 ; i < length; i ++)
			{
				//printk(KERN_CONT "%x ", buf[i]);
				DMESG_DEBUG("%x ", buf[i]);
			}
			break;
		default:
			DMESG_DEBUG("Other Message:%c\n", buf[0]);
			DMESG_DEBUG("length: %d\n", length);
			DMESG_DEBUG("buf: %s\n", buf);
	}

	DMESG_DEBUG("Report Handled");
}

/** HID-OUTデータ完了通知 **/
void urb_out_complete(struct urb* urb) 
{
	switch (urb->status) {
	case 0:
                DMESG_DEBUG("[OUT] actual=%d / max=%d",
			urb->actual_length, urb->transfer_buffer_length);
		report_out_handler(urb->transfer_buffer, urb->transfer_buffer_length);
		DMESG_DEBUG("Urb Success");
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		DMESG_ERR("urb shutting down with %d\n", urb->status);
		break;
	default:
		DMESG_ERR("urb status %d received\n", urb->status);
		break;
	}

	kfree(urb->transfer_buffer);
	usb_free_urb(urb);
	DMESG_DEBUG("Completed");
}

/** Spaceの位置の検索 **/
loff_t seek_space(const char* buff_from_user, size_t count, loff_t pos) 
{
	for(loff_t i = pos ; i < count; i ++ ) 
	{
		if(buff_from_user[i] == ' ' || buff_from_user[i] == 0) 
		{
			return i;
		}
	}

	//return count-1;
	return count;
}

static const int MAX_RAW_TEXT_SIZE = 64;
static const int RETVAL_FUNC_OK = 0;
static const int RETVAL_PARSE_FINISHED = 1;
static const int RETVAL_PARSE_INVALID = -1;

int parse_user_command(char* raw_text,  size_t count, struct user_command* command) 
{
	char temporary_text[MAX_RAW_TEXT_SIZE];
	loff_t offset = 0;
	loff_t next_space = 0;
	const int MAX_ARGUMENT_LOOP = 16;

	memset(command, 0, sizeof(struct user_command));

        DMESG_DEBUG("Raw text: %s", raw_text);
	if(raw_text[0] == 'r') 
	{
		command->command_state = STATE_READ;
		DMESG_DEBUG("Read command receieved.\n");
	}
	else if(raw_text[0] == 'w')
	{
		command->command_state = STATE_WRITE;
		DMESG_DEBUG("Write command receieved.\n");
	}
	else
	{
		command->command_state = STATE_INVALID;
		DMESG_DEBUG("Invalid command receieved.\n");
		return RETVAL_PARSE_INVALID;
	}

	offset = seek_space(raw_text, count, 0);

	for(int i = 0 ; i < MAX_ARGUMENT_LOOP; ++i) 
	{
		next_space = seek_space(raw_text, count, offset + 1);

		if(offset == next_space) 
		{
			return RETVAL_PARSE_FINISHED;
		}

		if( command->command_state == STATE_READ ||
		    command->command_state == STATE_WRITE )
		{
			struct user_read_parameter* read_parameter = &command->read_parameters;

			memset(temporary_text, 0, MAX_RAW_TEXT_SIZE);
			memcpy(temporary_text, raw_text + offset + 1, next_space - offset - 1);	

			DMESG_DEBUG("command: '%c' argument(%d,%d):%s\n",
				raw_text[0],
				offset, next_space, temporary_text);

			switch(i)
			{
				case 0:
					kstrtol(temporary_text, 16, &read_parameter->address);	
					break;
				case 1:
					kstrtol(temporary_text, 16, &read_parameter->datasize);	
					break;
				default:
					if( command->command_state == STATE_WRITE) {
						long val; 
						kstrtol(temporary_text, 16, &val);
						read_parameter->senddata[i-2] = (char)val;	
					}
					break;
			}
		}

		offset = next_space;
	}

	return RETVAL_FUNC_OK;
}

static const int RETVAL_FORMAT_INVALID = -1;

int format_buffer_command(const struct user_command* command, char** buffer, int buffer_size)
{
	char* transmit_buff = *buffer;
	if(command->command_state == STATE_READ)
	{
		long address = command->read_parameters.address;
		long datasize = command->read_parameters.datasize;

		transmit_buff[0] = 'r';
		transmit_buff[1] = address & 0xFF;
		transmit_buff[2] = (address & 0xFF00) >> 8;
		transmit_buff[3] = datasize;
	}
	else if(command->command_state == STATE_WRITE)
	{
		char* transmit_buff = *buffer;
		long address = command->read_parameters.address;
		long datasize = command->read_parameters.datasize;
		char* senddata = command->read_parameters.senddata;

		int i=0;
		transmit_buff[i++] = 'w';
		transmit_buff[i++] = address & 0xFF;
		transmit_buff[i++] = (address & 0xFF00) >> 8;
		transmit_buff[i++] = datasize;
		if(datasize > sizeof(senddata) || datasize > sizeof(*transmit_buff)-4) {
			DMESG_INFO("transmit_buff overflow. datasize: %d / senddata: %d / transmit_buff: %d\n",
				datasize, sizeof(senddata), sizeof(transmit_buff));
			return RETVAL_FORMAT_INVALID;
		}
		for(int j=0; j<datasize; j++ ) {
			transmit_buff[i+j] = senddata[j];
		}
	}
	else
	{
		return RETVAL_FORMAT_INVALID;
	}
	DMESG_DEBUG_DUMP("Constructed command: ", transmit_buff, 16, true);

	return RETVAL_FUNC_OK;
}

/** データ書き込み **/
ssize_t skel_write(struct file *file, const char __user *buff, size_t count, loff_t *f_pos)
{
	struct usb_skel *pDev = file->private_data;
	if(pDev->disconnected)
		return -ENODEV;

	if(count >= MAX_RAW_TEXT_SIZE) 
	{
		DMESG_ERR("Data size is too large");
		return -1;
	}

	char user_raw_text[MAX_RAW_TEXT_SIZE];
	memset(user_raw_text, 0, MAX_RAW_TEXT_SIZE);
	copy_from_user(user_raw_text, buff, count);

	struct user_command command;
	if(parse_user_command(user_raw_text, count, &command) < 0 )
	{
		DMESG_ERR("Parsing failed");
		return -1;
	}

	long address = command.read_parameters.address;
	long datasize = command.read_parameters.datasize;
        char* senddata = command.read_parameters.senddata;

	struct urb* urb_out; // urb header for send
	urb_out = usb_alloc_urb(0, GFP_KERNEL);
	if(!urb_out) 
	{
		DMESG_ERR("Memory allocation failed.\n");
		return -ENOMEM;
	}

	char* transmit_buff = kmalloc(pDev->int_out_buffer_length, GFP_KERNEL); 
	if(!transmit_buff) {
		DMESG_ERR("transmit_buff alloc failed.\n");
		usb_free_urb(urb_out);
		return -ENOMEM;
	}
	memset(transmit_buff, 0, pDev->int_out_buffer_length);
	if (format_buffer_command(&command, &transmit_buff, pDev->int_out_buffer_length ) < 0)
	{
		DMESG_ERR("Format failed.\n");
		kfree(transmit_buff);
		usb_free_urb(urb_out);
		return -EINVAL;
	}

	pDev->expected_len = 0;
	pDev->received_len = 0;

	usb_fill_int_urb(
			urb_out, pDev->udev, 
			usb_sndintpipe(pDev->udev, pDev->int_out_endpoint->bEndpointAddress),
			transmit_buff,
			pDev->int_out_buffer_length,
			urb_out_complete,
			pDev,
			pDev->int_out_endpoint->bInterval);

	int ret = usb_submit_urb(urb_out, GFP_KERNEL);
	if(ret == 0) {
		// succeeded. no free required
		DMESG_DEBUG("urb(out) submit complete.")
		//run_read(pDev); // no need to call because skel_open called it before.
		return count;
	}
	else {
		if(ret == -EBUSY)
			DMESG_DEBUG("urb(out) submit BUSY.")
		else
			DMESG_DEBUG("urb(out) submit Other err(%d).", ret)
		kfree(transmit_buff);
		usb_free_urb(urb_out);
		return ret;
	}
}


/**
 * USB接続開始
 * 
 * **/

int skel_probe(struct usb_interface* ip, const struct usb_device_id* pID) 
{
	int errno = -ENOMEM;
	bool registered = false;


	struct usb_skel* pDev = kmalloc(sizeof(struct usb_skel), GFP_KERNEL);
	if(!pDev)
	{
		DMESG_ERR("Out of memory.\n");
		goto L_Error;
	}
	memset(pDev, 0, sizeof(*pDev));

	init_usb_anchor(&(pDev->submitted));
	kref_init(&pDev->kref);							// 参照カウンタの初期化
	pDev->udev = usb_get_dev(interface_to_usbdev(ip));		// USBデバイスの存在チェック
	pDev->ip = ip;
	pDev->mode = BEUATO_MODE_BINARY;
	DMESG_INFO("return mode: %d\n", pDev->mode)
	// エンドポイントの取得
	// https://wiki.bit-hive.com/north/pg/usb%E3%83%89%E3%83%A9%E3%82%A4%E3%83%90
	struct usb_host_interface* pHostIf = ip->cur_altsetting;
	for(int i = 0 ; i < pHostIf->desc.bNumEndpoints; ++i ) 
	{
		dev_info(&ip->dev, "Endpoint %d\n", i);

		struct usb_endpoint_descriptor* endpoint = &pHostIf->endpoint[i].desc;

		if(usb_endpoint_dir_in(endpoint)) 
		{
			pDev->int_in_endpoint = endpoint;
			pDev->int_in_buffer_length = endpoint->wMaxPacketSize;
			dev_info(&ip->dev, "[I] Endpoint count:%d\n", i);
			dev_info(&ip->dev, "[I] Endpoint wMaxPacketSize:%d\n", pDev->int_in_buffer_length);
		}

		if(usb_endpoint_dir_out(endpoint)) 
		{
			pDev->int_out_endpoint = endpoint;
			pDev->int_out_buffer_length = endpoint->wMaxPacketSize;
			dev_info(&ip->dev, "[O] Endpoint count:%d\n", i);
			dev_info(&ip->dev, "[O] Endpoint wMaxPacketSize:%d\n", pDev->int_out_buffer_length);
		}
	}
	usb_set_intfdata(ip, pDev);

	errno = usb_register_dev(ip, &skel_class);
	if(errno) 
	{
		DMESG_ERR("Not able to get minor for this device.\n");
		usb_set_intfdata(ip, NULL);
		goto L_Error;
	}
	registered = true;

	dev_info(&ip->dev, "[+] usb_skel: Attached=%d", ip->minor);

	// memory for urb and data buffer for receive
	errno = prepare_read(ip, pID, pDev);
	if(errno)
	{
		DMESG_ERR("Failed to prepare reading buffer\n");
		goto L_Error;
	}

	return 0;
L_Error:
	if(registered)
		usb_deregister_dev(ip, &skel_class);
	usb_set_intfdata(ip,NULL);

	if(pDev) free_prepare_read(pDev);
	free_read_results(&read_results);

	if(pDev)
	{
		kref_put(&pDev->kref, skel_dispose);
	}
	return errno;
}

/**
 * USB接続終了
 * 
 * **/

void skel_disconnect(struct usb_interface* ip) 
{
	dev_info(&ip->dev, "[+] usb_skel: (%d) is skel_disconnected", ip->minor);

	struct usb_skel* pDev = usb_get_intfdata(ip);
	usb_set_intfdata(ip, NULL);

	pDev->disconnected = true;
	line_pos = line_len = 0;
	atomic_set(&beuato_data_ready, 0);

        if(pDev->int_in_urb) {
		//usb_kill_urb(pDev->int_in_urb);
		usb_kill_anchored_urbs(&(pDev->submitted));
		//usb_unanchor_urb(pDev->int_in_urb);
		usb_free_urb(pDev->int_in_urb);
		pDev->int_in_urb=NULL;
		DMESG_INFO("Free Urb\n");
	}

	// dispose interrupt in buffer
	kfree(pDev->int_in_buffer);
	pDev->int_in_buffer = NULL;
	//kfree(read_results.buffer);
	free_read_results(&read_results);
	DMESG_INFO("Free Buffer\n");


	usb_deregister_dev(ip, &skel_class);
	kref_put(&pDev->kref, skel_dispose);
}



/* read_results → ASCII 1 行 ("r <len> <data…>\n") へ変換して
 * 文字列を dst に書き込み、生成したバイト数を返す。
 */
static int build_line_ascii(char *dst)
{
    if(!read_results.ready_to_return)
    	DMESG_INFO("once used read_results are reused! WRONG.\n");

    int len = sprintf(dst, "r %x ", read_results.datasize);

    for (int i = 0; i < read_results.datasize; ++i)
        len += sprintf(dst + len, "%02x ", read_results.buffer[i]);
    dst[len++] = '\n';
    read_results.ready_to_return = 0; // already copied to return buf for "read" call.
    return len;
}

static int build_line_binary(char *dst)
{
    int datasize;

    if (!read_results.ready_to_return) {
        DMESG_INFO("once used read_results are reused! WRONG.\n");
    }

    /* データ長を取得 */
    datasize = read_results.datasize;

    if (datasize < 0)
        datasize = 0;
    if (datasize > MAX_IN_TEXT_SIZE)
        datasize = MAX_IN_TEXT_SIZE;

    /* READヘッダ:  'r' + uint8_t length */
    /* WRITEヘッダ: 'w' + uint8_t addr + uint8_t length */
    int pt = 0;
    dst[pt++] = (u8)read_results.command;
    if(read_results.command == 'w')
            dst[pt++] = (u8)read_results.addr;
    dst[pt++] = (u8)datasize;

    /* データ本体をコピー */
    if (datasize > 0)
        memcpy(dst+pt, read_results.buffer, datasize);

    /* 返却済みフラグをクリア */
    read_results.ready_to_return = 0;

    /* トータル長を返す */
    return pt+datasize;
}



ssize_t skel_read_partial(struct file *file, char __user *buf,
                  size_t count, loff_t *ppos)
{
	struct usb_skel *pDev = file->private_data;
	if(pDev->disconnected)
		return -ENODEV;

	size_t n;

	if(line_pos < line_len) { // if exist data not yet returned to user
		size_t n = min(count, line_len-line_pos);
		if(copy_to_user(buf, line_buf+line_pos, n))
			return -EFAULT;
		line_pos += n;
		return n;
	}

	// no more buffered data
	if(wait_event_interruptible(beuato_waitq, atomic_read(&beuato_data_ready)))
		return -ERESTARTSYS;

	line_len = build_line_ascii(line_buf);
	line_pos = 0;
	atomic_set(&beuato_data_ready, 0);

	n = min(count, line_len);
	if(copy_to_user(buf, line_buf, n))
		return -EFAULT;
	line_pos = n;
	return n;
}


ssize_t skel_read(struct file *file, char __user *buf,
                  size_t count, loff_t *ppos)
{
	// oneshot reading only supported
	//char line_buf[MAX_PROC_TEXT_SIZE];
	//size_t len;

	struct usb_skel *pDev = file->private_data;

	if(wait_event_interruptible(beuato_waitq, atomic_read(&beuato_data_ready))) {
		DMESG_INFO("wait_event failed.\n");
		return -ERESTARTSYS;
	}
	if (pDev->mode == BEUATO_MODE_BINARY) {
		line_len = build_line_binary(line_buf);
		DMESG_INFO("respond binary mode.\n");
	}
	else {
		line_len = build_line_ascii(line_buf);
		DMESG_INFO("respond ascii mode.\n");
	}

	atomic_set(&beuato_data_ready, 0);
	if(count < (size_t)line_len) {
		DMESG_INFO("read() requests %d bytes but should return more byte length %d.\n",count, line_len);
		return -EINVAL;
	}
	if(copy_to_user(buf, line_buf, line_len)) {
		DMESG_INFO("copy_to_user failed.\n");
		return -EFAULT;
	}
	return line_len;
}


ssize_t skel_read_org2(struct file *file, char __user *buf,
                  size_t count, loff_t *ppos)
{
    static char line_buf[MAX_PROC_TEXT_SIZE];
    static size_t pos = 0, len = 0;

    /* 行バッファを使い切っていたら新しいデータ到着を待つ */
    if (pos >= len) {
        /* ブロッキング待ち（^C で中断可） */
        if (wait_event_interruptible(beuato_waitq,
                atomic_read(&beuato_data_ready)))
            return -ERESTARTSYS;

        /* read_results → ASCII へ変換 */
        len = build_line_ascii(line_buf);
        pos = 0;

        /* フラグをクリアして次回受信を待てるように */
        atomic_set(&beuato_data_ready, 0);
    }

    /* 残りを必要量だけコピー */
    size_t ncopy = min(count, len - pos);
    if (copy_to_user(buf, line_buf + pos, ncopy))
        return -EFAULT;

    pos += ncopy;
    return ncopy;
}

ssize_t skel_read_org (struct file* file, char __user *buf, size_t count, loff_t *f_pos) 
{
	char temp_buff[MAX_PROC_TEXT_SIZE];
	char result_buff[MAX_PROC_TEXT_SIZE];
	memset(temp_buff, 0, MAX_PROC_TEXT_SIZE);

	int len_head = 0;
	len_head = sprintf(temp_buff, "r %x ", read_results.datasize);
	
	int offset = len_head;
	for(int i = 0 ; i < read_results.datasize; i ++ ) 
	{
		int len_part = sprintf(temp_buff + offset, "%x ", read_results.buffer[i]);
		offset += len_part;
	}

	int len_all = sprintf(result_buff, "%s\n", temp_buff);

	int ret = copy_to_user(buf, result_buff, len_all);
	if(ret != 0)
	{
		DMESG_ERR("copy_to_info failed:%d", ret);
		return -EFAULT;
	}

	f_pos += len_all;

	return len_all;
}

__poll_t skel_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &beuato_waitq, wait);
	return atomic_read(&beuato_data_ready) ? POLLIN|POLLRDNORM : 0;
}

long skel_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct usb_skel *pDev = file->private_data;
	int val, mode;

	switch(cmd) {
	case BEUATO_IOC_SET_DEBUG:
		if( copy_from_user(&val, (int __user *)arg, sizeof(int)))
			return -EFAULT;
		atomic_set(&beuato_dbg, val ? 1 : 0);
		if( val ) {
			DMESG_INFO("Driver DEBUG LOG ON.\n");
		} else {
			DMESG_INFO("Driver DEBUG LOG OFF.\n");
		}
		return 0;
	case BEUATO_IOC_SET_MODE:
        	if (copy_from_user(&mode, (int __user *)arg, sizeof(int)))
	            return -EFAULT;
	        if (mode != BEUATO_MODE_ASCII && mode != BEUATO_MODE_BINARY)
	            return -EINVAL;
	        pDev->mode = mode;
		if( mode == BEUATO_MODE_ASCII ) {
			DMESG_INFO("ASCII Mode ON.\n");
		} else if( mode == BEUATO_MODE_BINARY ) {
			 DMESG_INFO("BINARY Mode ON.\n");
		}
	        return 0;

	default:
		return -ENOTTY;
	}
}



int skel_flush(struct file *file, fl_owner_t id)
{
	DMESG_INFO("Flush called but no effect on current implement.\n");
/*
	// このコメントを外すとハングアップを誘う。flushでデバイス状態クリアは危険。disconnectでやるべき。
	DMESG_INFO("Flush rest buffer at fd closed");
	struct usb_skel *pDev = file->private_data;
	if(pDev) {
		DMESG_INFO("received len: %d  expected len: %d", pDev->received_len, pDev->expected_len);
		DMESG_INFO("beuato_ready: %d", beuato_data_ready);
		clear_device_state(pDev);
		line_pos = line_len = 0; //reset
	}
*/
	return 0;
}

MODULE_DEVICE_TABLE(usb, skel_table);
module_usb_driver(skel_driver);

MODULE_LICENSE("GPL");
