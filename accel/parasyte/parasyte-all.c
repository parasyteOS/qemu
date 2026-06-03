/*
 * Copyright (C) 2025       mlatus
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 *
 * Contributions after 2012-01-13 are licensed under the terms of the
 * GNU GPL, version 2 or (at your option) any later version.
 */

#include <sys/ioctl.h>
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/accel.h"
#include "qemu/thread.h"
#include "qapi/error.h"
#include "accel/dummy-cpus.h"
#include "accel/accel-ops.h"
#include "accel/accel-cpu-ops.h"
#include "system/memory.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "system/parasyte.h"
#include "hw/boards.h"

bool parasyte_allowed;

struct ParasyteState {
    AccelState parent_obj;
    int dev_fd;

    int hb_fd;
    void *hb;
    int ram_fd;
    void *ram;
    int fdt_fd;
    void *fdt;
    __u64 fdt_size;
    MemMapEntry ram_entry;
    MemMapEntry hive_queue_entry;
    MemMapEntry spore_queue_entry;
    MemMapEntry flags_entry;
    __u64 kimage_offset;

    struct parasyte_msg_queue *consuming_queue;
    struct parasyte_msg_queue *producing_queue;
    QemuMutex msg_produce_lock;

    bool running;
    QemuThread *io_thread;
    QemuThread *hb_thread;
};

static void *map_physmem(ParasyteState *ps, __u64 phys_addr)
{
    if (phys_addr < ps->ram_entry.base || phys_addr >= ps->ram_entry.base + ps->ram_entry.size)
        return NULL;
    return ps->ram + (phys_addr - ps->ram_entry.base);
}

static inline void send_notify(ParasyteState* ps, uint32_t cpu)
{
    if (ioctl(ps->dev_fd, PARASYTE_IOCTL_NOTIFY, cpu))
        error_report("Failed to invoke ioctl notify: %d", errno);
}

static void process_msg(ParasyteState *ps, struct parasyte_msg *msg)
{
    __u64 type = qatomic_read(&msg->type);
    __u64 flags = qatomic_read(&msg->flags);
    struct parasyte_io_request *io_request;
    __u64 address, size, ptr;
    void *buf;
    MemTxResult ret;
    bool err = false, read, use_ptr;

    switch (type) {
    case PARASYTE_MSG_TYPE_IO_REQUEST:
        read = IO_MSG_READ(flags);
        use_ptr = IO_MSG_USE_PTR(flags);

        io_request = &msg->payload.io_request;
        address = qatomic_read(&io_request->address);
        size = qatomic_read(&io_request->size);

        if (use_ptr) {
            ptr = qatomic_read(&io_request->data.ptr);
            buf = map_physmem(ps, ptr);
            if (!buf) {
                error_report("Ptr out of range: 0x%llx", ptr);
                err = true;
                break;
            }
        } else {
            buf = &io_request->data.value;
        }

        ret = address_space_rw(&address_space_memory,
                               address, MEMTXATTRS_UNSPECIFIED,
                               buf, size, !read);
        if (ret != MEMTX_OK) {
            error_report("Error during IO: %d", ret);
            err = true;
        }
        break;
    default:
        error_report("Unknown message type: 0x%llx", type);
        err = true;
    }

    if (err) {
        MSG_SET_ERROR(flags);
        qatomic_set(&msg->flags, flags);
    }
}

static void consume_msg(ParasyteState *ps)
{
    struct parasyte_msg_queue *queue = ps->consuming_queue;
    __u64 capacity = queue->capacity;
    __u64 consumer_head = qatomic_load_acquire(&queue->consumer_head);
    __u64 producer_head = qatomic_load_acquire(&queue->producer_head);
    __u64 tail = qatomic_load_acquire(&queue->tail);
    __u64 idx = consumer_head, flags;
    struct parasyte_msg *msg;

loop:
    while (idx != producer_head) {
        msg = &queue->ring[idx];
        flags = qatomic_read(&msg->flags);

        if (!MSG_PENDING(flags))
            error_report("Consumed message ahead of consumer head: %llx, producer_head: %llx, tail: %llx, type: %llx, flags: %llx",
                         consumer_head, producer_head, tail, qatomic_read(&msg->type), flags);
        else {
            process_msg(ps, msg);
            flags = qatomic_read(&msg->flags);
            MSG_CLEAR_PENDING(flags);
            qatomic_set(&msg->flags, flags);
        }

        idx = (idx + 1) % capacity;
        consumer_head = idx;

        if (MSG_SYNC(flags)) {
            qatomic_store_release(&queue->consumer_head, consumer_head);
            send_notify(ps, msg->cpu);
        }
    }

    producer_head = qatomic_load_acquire(&queue->producer_head);
    if (idx != producer_head) goto loop;

    qatomic_store_release(&queue->consumer_head, consumer_head);
    send_notify(ps, PARASYTE_HIVE_ANY_CPU);
}

static void post_produce_msg(ParasyteState *ps)
{
    struct parasyte_msg_queue *queue = ps->producing_queue;
    __u64 tail = qatomic_load_acquire(&queue->tail);
    __u64 head = qatomic_load_acquire(&queue->consumer_head);
    __u64 capacity = queue->capacity;
    bool moving_tail = true;
    __u64 idx = tail;
    struct parasyte_msg *msg;
    __u64 flags;

loop:
    while (idx != head) {
        msg = &queue->ring[idx];
        flags = qatomic_read(&msg->flags);

        if (MSG_PENDING(flags)) {
            error_report("Pending message left behind consumer head");
            MSG_CLEAR_PENDING(flags);
            MSG_SET_ERROR(flags);
            qatomic_set(&msg->flags, flags);
        }

        if (!MSG_COMPLETE(flags)) {
            if (MSG_SYNC(flags)) {
                error_report("Unknown message with sync flag set");
                MSG_SET_COMPLETE(flags);
                qatomic_set(&msg->flags, flags);
            } else {
                error_report("Incomplete message without sync flag set");
                MSG_SET_COMPLETE(flags);
                qatomic_set(&msg->flags, flags);
            }
        }

        idx = (idx + 1) % capacity;
        if (moving_tail) tail = idx;
    }

    head = qatomic_load_acquire(&queue->consumer_head);
    if (idx != head) goto loop;

    qatomic_store_release(&queue->tail, tail);
}

static void *parasyte_io_thread_fn(void *arg)
{
    ParasyteState *ps = (ParasyteState *)arg;
    int rc;

    while (ps->running) {
        rc = ioctl(ps->dev_fd, PARASYTE_IOCTL_WAIT_MSG);
        if (rc < 0) {
            error_report("Failed to invoke ioctl: %d", errno);
            break;
        }
        if (!rc) {
            error_report("WAIT_MSG returned with empty status.");
            continue;
        }
        if (rc & PARASYTE_MSG_STATUS_SHUTDOWN) {
            info_report("Shutting down");
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            break;
        }
        if (rc & PARASYTE_MSG_STATUS_PENDING) {
            consume_msg(ps);
            post_produce_msg(ps);
        }
    }

    return NULL;
}

// static void *parasyte_hb_thread_fn(void *arg)
// {
//     ParasyteState *ps = (ParasyteState *)arg;
//     __u64 *hb = ps->hb;
//     info_report("Try to read from %p", hb);
//     __u64 prev_state = qatomic_read(hb);
//     __u64 state;
// 
//     info_report("Parasyte Heartbeat: %llx", prev_state);
// 
//     while (true) {
//         state = qatomic_read(hb);
//         if (state != prev_state) {
//             info_report("Parasyte Heartbeat: %llx", state);
//             prev_state = state;
//         }
//     }
// 
//     return NULL;
// }

static void parasyte_setup_post(AccelState *as)
{
    ParasyteState *ps = PARASYTE_STATE(as);
    struct parasyte_setup_params setup_params = {
        .kimage_offset = ps->kimage_offset,
        .flags_offset = ps->flags_entry.base - ps->ram_entry.base,
        .hive_queue_offset = ps->hive_queue_entry.base - ps->ram_entry.base,
        .spore_queue_offset = ps->spore_queue_entry.base - ps->ram_entry.base,
    };

    if (ioctl(ps->dev_fd, PARASYTE_IOCTL_SETUP, &setup_params) < 0) {
        error_report("Failed to setup parasyte kernel.");
        exit(1);
    }

    // info_report("start parasyte hb thread");
    // ps->hb_thread = g_malloc0(sizeof(QemuThread));
    // qemu_thread_create(ps->hb_thread, "PARASYTE HB THREAD", parasyte_hb_thread_fn,
    //                    ps, QEMU_THREAD_JOINABLE);

    if (ioctl(ps->dev_fd, PARASYTE_IOCTL_START) < 0) {
        error_report("Failed to start parasyte kernel.");
        exit(1);
    }

    info_report("Starting parasyte I/O thread");
    ps->running = true;
    ps->io_thread = g_malloc0(sizeof(QemuThread));
    qemu_thread_create(ps->io_thread, "PARASYTE IO THREAD", parasyte_io_thread_fn,
                       ps, QEMU_THREAD_JOINABLE);
}

static int parasyte_init(AccelState *as, MachineState *ms)
{
    MachineClass *mc = MACHINE_GET_CLASS(ms);
    ParasyteState *ps = PARASYTE_STATE(as);

    /*
     * opt out of system RAM being allocated by generic code
     */
    mc->default_ram_id = NULL;
    ps->dev_fd = qemu_open("/dev/parasyte", O_RDWR, &error_fatal);

    qemu_mutex_init(&ps->msg_produce_lock);
    ps->running = false;

    return 0;
}

static void parasyte_accel_class_init(ObjectClass *oc, const void *data)
{
    AccelClass *ac = ACCEL_CLASS(oc);

    ac->name = "parasyte";
    ac->init_machine = parasyte_init;
    ac->setup_post = parasyte_setup_post;
    ac->allowed = &parasyte_allowed;
}

static const TypeInfo parasyte_accel_type = {
    .name = TYPE_PARASYTE_ACCEL,
    .parent = TYPE_ACCEL,
    .class_init = parasyte_accel_class_init,
    .instance_size = sizeof(ParasyteState),
};

static void parasyte_accel_ops_class_init(ObjectClass *oc, const void *data)
{
    AccelOpsClass *ops = ACCEL_OPS_CLASS(oc);

    ops->create_vcpu_thread = dummy_start_vcpu_thread;
    ops->handle_interrupt = generic_handle_interrupt;
}

static const TypeInfo parasyte_accel_ops_type = {
    .name = ACCEL_OPS_NAME("parasyte"),
    .parent = TYPE_ACCEL_OPS,
    .class_init = parasyte_accel_ops_class_init,
    .abstract = true,
};

static void parasyte_type_init(void)
{
    type_register_static(&parasyte_accel_type);
    type_register_static(&parasyte_accel_ops_type);
}
type_init(parasyte_type_init);

void parasyte_alloc(ParasyteState* ps, char *cpus, uint64_t ram_size, uint64_t queue_capacity)
{
    hwaddr kimage_addr;
    struct parasyte_alloc_params alloc_params = {
        .cpus = cpus,
        .cpus_len = strlen(cpus) + 1,
        .ram_size = ram_size,
    };
    int ret;

    ret = ioctl(ps->dev_fd, PARASYTE_IOCTL_ALLOC, &alloc_params);
    if (ret) {
        error_report("Failed to setup parasyte: %d", errno);
        exit(1);
    }

    ps->hb_fd = alloc_params.hb_fd;
    ps->hb = mmap(NULL, alloc_params.hb_size, PROT_READ|PROT_WRITE, MAP_SHARED, alloc_params.hb_fd, 0);
    if (ps->hb == MAP_FAILED) {
        error_report("Mmap hb failed: %d", errno);
        exit(1);
    }

    ps->ram_fd = alloc_params.ram_fd;
    ps->ram = mmap(NULL, ram_size, PROT_READ|PROT_WRITE, MAP_SHARED, alloc_params.ram_fd, 0);
    if (ps->ram == MAP_FAILED) {
        error_report("Mmap ram failed: %d", errno);
        exit(1);
    }

    ps->fdt_fd = alloc_params.fdt_fd;
    ps->fdt = mmap(NULL, alloc_params.fdt_size, PROT_READ|PROT_WRITE, MAP_SHARED, alloc_params.fdt_fd, 0);
    if (ps->fdt == MAP_FAILED) {
        error_report("Mmap fdt failed: %d", errno);
        exit(1);
    }
    ps->fdt_size = alloc_params.fdt_size;

    ps->ram_entry.base = alloc_params.ram_paddr;
    ps->ram_entry.size = ram_size;

    ps->hive_queue_entry.base = ps->ram_entry.base;
    ps->hive_queue_entry.size = PARASYTE_MSG_QSIZE(queue_capacity);
    ps->producing_queue = ps->ram;
    qatomic_set(&ps->producing_queue->producer_head, 0);
    qatomic_set(&ps->producing_queue->consumer_head, 0);
    qatomic_set(&ps->producing_queue->tail, 0);
    qatomic_set(&ps->producing_queue->capacity, queue_capacity);

    ps->spore_queue_entry.base = ps->hive_queue_entry.base + ps->hive_queue_entry.size;
    ps->spore_queue_entry.size = PARASYTE_MSG_QSIZE(queue_capacity);
    ps->consuming_queue = ps->ram + ps->hive_queue_entry.size;
    qatomic_set(&ps->consuming_queue->producer_head, 0);
    qatomic_set(&ps->consuming_queue->consumer_head, 0);
    qatomic_set(&ps->consuming_queue->tail, 0);
    qatomic_set(&ps->consuming_queue->capacity, queue_capacity);

    ps->flags_entry.base = ps->spore_queue_entry.base + ps->spore_queue_entry.size;
    ps->flags_entry.size = sizeof(__u64);

#define SZ_2M   0x00200000
    kimage_addr = ROUND_UP(ps->spore_queue_entry.base + ps->spore_queue_entry.size, SZ_2M);
    ps->kimage_offset = kimage_addr - ps->ram_entry.base;
}

MemMapEntry *parasyte_ram_entry(ParasyteState* ps)
{
    return &ps->ram_entry;
}

void *parasyte_ram_ptr(ParasyteState* ps)
{
    return ps->ram;
}

int parasyte_ram_fd(ParasyteState* ps)
{
    return ps->ram_fd;
}

MemMapEntry *parasyte_hive_queue_entry(ParasyteState* ps)
{
    return &ps->hive_queue_entry;
}

MemMapEntry *parasyte_spore_queue_entry(ParasyteState* ps)
{
    return &ps->spore_queue_entry;
}

MemMapEntry *parasyte_flags_entry(ParasyteState* ps)
{
    return &ps->flags_entry;
}

void *parasyte_fdt_ptr(ParasyteState *ps)
{
    return ps->fdt;
}

size_t parasyte_fdt_size(ParasyteState *ps)
{
    return ps->fdt_size;
}

hwaddr parasyte_kimage_offset(ParasyteState *ps)
{
    return ps->kimage_offset;
}

void parasyte_set_irq(ParasyteState* ps, int irq, int level)
{
    struct parasyte_msg_queue *queue = ps->producing_queue;
    __u64 head, next_head, tail, capacity = queue->capacity;
    struct parasyte_msg* msg;
    union parasyte_msg_payload *payload;

    qemu_mutex_lock(&ps->msg_produce_lock);

    head = qatomic_load_acquire(&queue->producer_head);
    next_head = (head + 1) % capacity;
    tail = qatomic_load_acquire(&queue->tail);

    if (next_head == tail) {
        error_report("Producing queue is full");
        qemu_mutex_unlock(&ps->msg_produce_lock);
        return;
    }

    msg = &queue->ring[head];
    payload = &msg->payload;

    qatomic_set(&payload->softirq.nr, irq);
    qatomic_set(&msg->cid, 0);
    qatomic_set(&msg->type, PARASYTE_MSG_TYPE_SOFTIRQ);
    qatomic_set(&msg->flags, PARASYTE_MSG_FLAG_PENDING | PARASYTE_MSG_FLAG_COMPLETE);

    qatomic_store_release(&queue->producer_head, next_head);

    qemu_mutex_unlock(&ps->msg_produce_lock);

    send_notify(ps, PARASYTE_HIVE_ANY_CPU);
}
