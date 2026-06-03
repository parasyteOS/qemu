#ifndef SYSTEM_PARASYTE_H
#define SYSTEM_PARASYTE_H

#include "qemu/accel.h"
#include "exec/hwaddr.h"

/* Kernel ABI */

#define PARASYTE_MSG_TYPE_IO_REQUEST    0
#define PARASYTE_MSG_TYPE_SOFTIRQ       1

#define PARASYTE_MSG_FLAG_COMPLETE      (1ULL << 63)
#define PARASYTE_MSG_FLAG_PENDING       (1ULL << 62)
#define PARASYTE_MSG_FLAG_ERROR         (1ULL << 61)
#define PARASYTE_MSG_FLAG_SYNC          (1ULL << 60)
#define PARASYTE_MSG_FLAG_IO_READ       (1ULL << 0)
#define PARASYTE_MSG_FLAG_IO_USE_PTR    (1ULL << 1)

#define PARASYTE_MSG_STATUS_SHUTDOWN    (1 << 0)
#define PARASYTE_MSG_STATUS_PENDING     (1 << 1)

#define PARASYTE_FLAGS_SHUTDOWN_REQ    (1 << 0)
#define PARASYTE_FLAGS_SHUTDOWN_ACK    (1 << 1)

#define PARASYTE_HIVE_ANY_CPU           (UINT32_MAX)

struct parasyte_io_request {
        __u64 address;
        __u64 size;
        union {
                __u64 value;
                __u64 ptr;
        } data;
} __attribute__((__packed__));

struct parasyte_softirq {
        __u64 nr;
} __attribute__((__packed__));

union parasyte_msg_payload {
        struct parasyte_io_request io_request;
        struct parasyte_softirq softirq;
} __attribute__((__packed__));

struct parasyte_msg {
        __u64 cid;
        __u32 cpu;
        __u32 padding;
        __u64 type;
        __u64 flags;
        union parasyte_msg_payload payload;
} __attribute__((__packed__));

struct parasyte_msg_queue {
        __u64 producer_head;
        __u64 consumer_head;
        __u64 tail;
        __u64 capacity;
        struct parasyte_msg ring[];
} __attribute__((__packed__));

#define PARASYTE_MSG_QSIZE(capacity) (sizeof(struct parasyte_msg_queue) + capacity * sizeof(struct parasyte_msg))

struct parasyte_alloc_params {
	/* Input */
	char* cpus;
	__u64 cpus_len;
	__u64 ram_size;
	/* Output */
	int ram_fd;
	__u64 ram_paddr;
	int fdt_fd;
	__u64 fdt_size;
    int hb_fd;
    __u64 hb_size;
};

struct parasyte_setup_params {
	__u64 kimage_offset;
	__u64 flags_offset;
	__u64 hive_queue_offset;
	__u64 spore_queue_offset;
};

#define PARASYTE_IOCTL_TYPE	0xEE

#define PARASYTE_IOCTL_ALLOC	_IOWR(PARASYTE_IOCTL_TYPE, 0x00, struct parasyte_alloc_params)
#define PARASYTE_IOCTL_SETUP	_IOW(PARASYTE_IOCTL_TYPE, 0x01, struct parasyte_setup_params)
#define PARASYTE_IOCTL_START	_IO(PARASYTE_IOCTL_TYPE, 0x02)
#define PARASYTE_IOCTL_SHUTDOWN	_IO(PARASYTE_IOCTL_TYPE, 0x03)
#define PARASYTE_IOCTL_WAIT_MSG _IO(PARASYTE_IOCTL_TYPE, 0x04)
#define PARASYTE_IOCTL_NOTIFY	_IO(PARASYTE_IOCTL_TYPE, 0x05)

/* Macros */
#define MSG_PENDING(flags) (!!(flags & PARASYTE_MSG_FLAG_PENDING))
#define MSG_CLEAR_PENDING(flags) flags &= ~PARASYTE_MSG_FLAG_PENDING
#define MSG_COMPLETE(flags) (!!(flags & PARASYTE_MSG_FLAG_COMPLETE))
#define MSG_SET_COMPLETE(flags) flags |= PARASYTE_MSG_FLAG_COMPLETE
#define MSG_ERROR(flags) (!!(flags & PARASYTE_MSG_FLAG_ERROR))
#define MSG_SET_ERROR(flags) flags |= PARASYTE_MSG_FLAG_ERROR
#define MSG_SYNC(flags) (!!(flags & PARASYTE_MSG_FLAG_SYNC))
#define IO_MSG_READ(flags) (!!(flags & PARASYTE_MSG_FLAG_IO_READ))
#define IO_MSG_USE_PTR(flags) (!!(flags & PARASYTE_MSG_FLAG_IO_USE_PTR))

/* Qemu Types */
struct ParasyteState;
typedef struct ParasyteState ParasyteState;

#define TYPE_PARASYTE_ACCEL ACCEL_CLASS_NAME("parasyte")
DECLARE_INSTANCE_CHECKER(ParasyteState, PARASYTE_STATE, TYPE_PARASYTE_ACCEL)

/* Qemu API */
void parasyte_alloc(ParasyteState* ps, char *cpus, uint64_t ram_size, uint64_t queue_size);
int parasyte_ram_fd(ParasyteState* ps);
void *parasyte_ram_ptr(ParasyteState* ps);
MemMapEntry *parasyte_ram_entry(ParasyteState* ps);
MemMapEntry *parasyte_hive_queue_entry(ParasyteState* ps);
MemMapEntry *parasyte_spore_queue_entry(ParasyteState* ps);
MemMapEntry *parasyte_flags_entry(ParasyteState* ps);
void *parasyte_fdt_ptr(ParasyteState *ps);
size_t parasyte_fdt_size(ParasyteState *ps);
hwaddr parasyte_kimage_offset(ParasyteState *ps);
void parasyte_set_irq(ParasyteState* ps, int irq, int level);

#endif
