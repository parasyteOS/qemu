/*
 * QEMU Parasyte hypha machine
 *
 * Copyright (c) 2025 mlatus
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include <libfdt.h>
#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/units.h"
#include "qemu/cacheflush.h"
#include "qemu/guest-random.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "hw/boards.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/loader.h"
#include "system/device_tree.h"
#include "system/runstate.h"
#include "system/address-spaces.h"
#include "system/parasyte.h"

#define HOST_FDT_PATH "/sys/firmware/fdt"

#define TYPE_HYPHA_MACHINE MACHINE_TYPE_NAME("hypha")
OBJECT_DECLARE_TYPE(HyphaMachineState, HyphaMachineClass,
                    HYPHA_MACHINE)

struct HyphaMachineClass {
    MachineClass parent;
};

struct HyphaMachineState {
    MachineState parent;

    char *cpus;
    uint64_t queue_size;

    uint32_t address_cells;
    uint32_t size_cells;
    char *gic_node_path;

    int virtio_mmio_num;
    int virtio_mmio_irq_base;
    MemMapEntry virtio_mmio_entry;

    MemoryRegion ram;
};

static void hypha_set_cpus(Object *obj, const char *value, Error **errp)
{
    char *colon;
    HyphaMachineState *hms = HYPHA_MACHINE(obj);
    g_free(hms->cpus);
    hms->cpus = g_strdup(value);

    colon = strchr(hms->cpus, '/');
    while (colon) {
        *colon = ',';
        colon = strchr(colon, '/');
    }
}

static char *hypha_get_cpus(Object *obj, Error **errp)
{
    HyphaMachineState *hms = HYPHA_MACHINE(obj);
    return g_strdup(hms->cpus);
}

static void hypha_set_queue_size(Object *obj, Visitor *v,
                                 const char *name, void *opaque,
                                 Error **errp)
{
    HyphaMachineState *hms = HYPHA_MACHINE(obj);
    uint64_t value;
    if (!visit_type_size(v, name, &value, errp))
        return;
    hms->queue_size = value;
}

static void hypha_get_queue_size(Object *obj, Visitor *v,
                                 const char *name, void *opaque,
                                 Error **errp)
{
    HyphaMachineState *hms = HYPHA_MACHINE(obj);
    uint64_t value = hms->queue_size;
    visit_type_uint64(v, name, &value, errp);
}

static void hypha_set_irq(void *opaque, int irq, int level)
{
    ParasyteState *ps = (ParasyteState *)opaque;
    parasyte_set_irq(ps, irq, level);
}

G_GNUC_PRINTF(1, 0)
static void fatal_report(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    error_vreport(fmt, ap);
    va_end(ap);

    exit(1);
}

static const void *copy_property(void *host_fdt, void *fdt, const char *node_path, const char *property,
                                 bool optional, int *val_len)
{
    int prop_len;
    const void *val;
    Error *err = NULL;

    if (strcmp(node_path, "/"))
        qemu_fdt_add_path(fdt, node_path);

    val = qemu_fdt_getprop(host_fdt, node_path, property, &prop_len, &err);
    if (val) {
        qemu_fdt_setprop(fdt, node_path, property, val, prop_len);
        if (val_len) *val_len = prop_len;
    } else {
        if (prop_len == -FDT_ERR_NOTFOUND)
            error_free(err);
        else
            error_report_err(err);
        if (!optional)
            fatal_report("Failed to found non-optional property from host");
    }

    return val;
}

static void copy_node_by_offset(void *host_fdt, void *fdt, int host_offset, int offset, bool recursive)
{
    int property, subnode, newnode, len, ret;
    const struct fdt_property *pdata;
    const char *name;

    fdt_for_each_property_offset(property, host_fdt, host_offset) {
        pdata = fdt_get_property_by_offset(host_fdt, property, &len);
        name = fdt_string(host_fdt, fdt32_to_cpu(pdata->nameoff));
        ret = fdt_setprop(fdt, offset, name, pdata->data, len);
        if (ret) fatal_report("Failed to copy property %s: %s", name, fdt_strerror(ret));
    }
    if (property != -FDT_ERR_NOTFOUND)
        fatal_report("Failed to find next property: %s", fdt_strerror(property));

    if (!recursive) return;

    fdt_for_each_subnode(subnode, host_fdt, host_offset) {
        name = fdt_get_name(host_fdt, subnode, &len);
        if (!name)
            fatal_report("Failed to get name of subnode: %s", fdt_strerror(len));

        newnode = fdt_add_subnode(fdt, offset, name);
        if (newnode < 0)
            fatal_report("Failed to add subnode: %s", fdt_strerror(newnode));

        copy_node_by_offset(host_fdt, fdt, subnode, newnode, recursive);
    }
    if (subnode != -FDT_ERR_NOTFOUND)
        fatal_report("Failed to find next subnode: %s", fdt_strerror(subnode));
}

static int fdt_path_offset_exact(const void *fdt, const char *path)
{
    const char *p = path;
    int offset = 0;

    if (!path || path[0] != '/')
        return -FDT_ERR_BADPATH;

    /* Root node */
    if (path[1] == '\0')
        return 0;

    while (*p == '/')
        p++;

    while (*p) {
        const char *end;
        int len;
        int child;
        int found = 0;

        end = p;
        while (*end && *end != '/')
            end++;

        len = end - p;

        fdt_for_each_subnode(child, fdt, offset) {
            int nlen;
            const char *name = fdt_get_name(fdt, child, &nlen);

            if (!name)
                continue;

            /*
             * Exact full-name match:
             * "serial@1000" != "serial@2000"
             * "serial"      != "serial@1000"
             */
            if (nlen == len && !memcmp(name, p, len)) {
                offset = child;
                found = 1;
                break;
            }
        }

        if (!found)
            return -FDT_ERR_NOTFOUND;

        p = end;
        while (*p == '/')
            p++;
    }

    return offset;
}

static void copy_node(void *host_fdt, void *fdt, const char *node_path, bool recursive, bool optional)
{
    int host_offset = fdt_path_offset_exact(host_fdt, node_path);
    if (host_offset < 0) {
        if (optional) {
            warn_report("No such node path, skip: %s", node_path);
            return;
        } else
            fatal_report("No such node path: %s", node_path);
    }
    int offset = qemu_fdt_add_path(fdt, node_path);
    if (offset < 0)
        fatal_report("Failed to add node %s", node_path);

    copy_node_by_offset(host_fdt, fdt, host_offset, offset, recursive);
}

static char **find_node_path(void *host_fdt, const char *compat)
{
    char **node_path = qemu_fdt_node_path(host_fdt, NULL, compat, &error_fatal);
    if (!node_path || !node_path[0]) {
        error_report("Failed to find host fdt node matching %s", compat);
        g_strfreev(node_path);
        node_path = NULL;
    } else if (node_path[1])
        fatal_report("More than one host fdt node matching %s", compat);

    return node_path;
}

static void copy_node_compat(void *host_fdt, void *fdt, const char *compat[], int num, bool recursive, bool optional)
{
    char **node_path;
    for (int i = 0; i < num; i++) {
        node_path = find_node_path(host_fdt, compat[i]);
        if (node_path) break;
    }
    if (!node_path) {
        if (optional) {
            warn_report("target node not found, skip");
            return;
        } else
            fatal_report("target node not found.");
    }

    copy_node(host_fdt, fdt, node_path[0], recursive, false);
    g_strfreev(node_path);
}

static void copy_basic_properties(HyphaMachineState *hms, void *host_fdt)
{
    void *fdt = MACHINE(hms)->fdt;
    const void *val;
    int val_len;

    val = copy_property(host_fdt, fdt, "/", "#address-cells", false, &val_len);
    if (val_len != 4)
        fatal_report("/#address-cells is not a cell?");
    hms->address_cells = fdt32_to_cpu(*(fdt32_t *)val);

    val = copy_property(host_fdt, fdt, "/", "#size-cells", false, &val_len);
    if (val_len != 4)
        fatal_report("/#size-cells is not a cell?");
    hms->size_cells = fdt32_to_cpu(*(fdt32_t *)val);

    copy_property(host_fdt, fdt, "/", "compatible", true, NULL);
    copy_property(host_fdt, fdt, "/", "dma-coherent", true, NULL);
    copy_property(host_fdt, fdt, "/", "model", true, NULL);
    copy_property(host_fdt, fdt, "/", "interrupt-parent", false, NULL);
}

static void setup_gic_node(HyphaMachineState *hms, void *host_fdt)
{
    MachineState *ms = MACHINE(hms);
    char **node_path;

    node_path = find_node_path(host_fdt, "arm,gic-v3");
    if (!node_path)
        fatal_report("GICv3 node not found.");

    hms->gic_node_path = g_strdup(node_path[0]);
    g_strfreev(node_path);

    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "compatible", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "#interrupt-cells", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "interrupt-controller", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "#address-cells", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "#size-cells", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "ranges", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "redistributor-stride", true, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "#redistributor-regions", true, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "reg", false, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "interrupts", true, NULL);
    copy_property(host_fdt, ms->fdt, hms->gic_node_path, "phandle", false, NULL);
    qemu_fdt_setprop(ms->fdt, hms->gic_node_path, "parasyte", NULL, 0);
}

static void setup_cpus_node(HyphaMachineState *hms, void *host_fdt)
{
    MachineState *ms = MACHINE(hms);
    const char *device_type, *name;
    const void *val;
    uint32_t cells = 0, size;
    uint64_t affi_cells[256];
    gchar **cpu_idxv;
    long cpu_idx;
    void *fdt = MACHINE(hms)->fdt, *buf;
    char *node_path, *cpu_path;
    bool match = false;
    int cpu_num, host_cpu_num = 0, buf_len, cpus, cpu, len, i = 0;

    cpu_idxv = g_strsplit(hms->cpus, ",", 0);
    if (!cpu_idxv[0])
        fatal_report("Failed to parse cpus");
    cpu_num = g_strv_length(cpu_idxv);

    val = copy_property(host_fdt, fdt, "/cpus", "#size-cells", false, NULL);
    cells += fdt32_to_cpu(*(fdt32_t *)val);
    val = copy_property(host_fdt, fdt, "/cpus", "#address-cells", false, NULL);
    cells += fdt32_to_cpu(*(fdt32_t *)val);

    if (cells > 2 || cells < 1)
        fatal_report("CPU cells invalid: %d", cells);

    if (qemu_fdt_setprop_cell(ms->fdt, hms->gic_node_path, "#parasyte-affinity-cells", cells) < 0)
        fatal_report("Failed to set #parasyte-affinity-cells");

    size = cells * 4;
    buf_len = size * cpu_num;
    buf = g_malloc(buf_len);

    for (i = 0; i < cpu_num; i++) {
        cpu_path = g_strdup_printf("/sys/devices/system/cpu/cpu%s/of_node/reg", cpu_idxv[i]);
        len = load_image_size(cpu_path, buf + i * size, size);
        if (len < 0) fatal_report("Failed to read %s", cpu_path);
    }

    cpus = fdt_path_offset_exact(host_fdt, "/cpus");
    fdt_for_each_subnode(cpu, host_fdt, cpus) {
        device_type = (const char *)fdt_getprop(host_fdt, cpu, "device_type", NULL);
        if (device_type && !strcmp("cpu", device_type)) {
            val = fdt_getprop(host_fdt, cpu, "reg", &len);
            if (!val) fatal_report("Failed to get reg of a cpu node: %s", fdt_strerror(len));

            match = false;
            for (i = 0; i < cpu_num; i++) {
                if (memcmp(val, buf + i * size, size))
                    continue;
                match = true;
                name = fdt_get_name(host_fdt, cpu, NULL);
                node_path = g_strdup_printf("/cpus/%s", name);
                copy_node(host_fdt, fdt, node_path, true, false);
                errno = 0;
                cpu_idx = strtol(cpu_idxv[i], NULL, 10);
                if (errno == ERANGE && (cpu_idx == LONG_MIN || cpu_idx == LONG_MAX))
                    fatal_report("Illegal cpu index");
                qemu_fdt_setprop_cell(fdt, node_path, "parasyte-hive-index", (uint32_t)cpu_idx);
                g_free(node_path);
                break;
            }
            if (!match) {
                affi_cells[host_cpu_num * 2] = cells;
                affi_cells[host_cpu_num * 2 + 1] = cells == 1 ? fdt32_to_cpu(*(fdt32_t *)val) : fdt64_to_cpu(*(fdt64_t *)val);
                host_cpu_num++;
            }
        }
    }

    if (qemu_fdt_setprop_sized_cells_from_array(ms->fdt, hms->gic_node_path, "parasyte-affinities", host_cpu_num, affi_cells) < 0)
        fatal_report("Failed to set parasyte-affinities");

    g_free(buf);
    g_strfreev(cpu_idxv);
}

static void copy_host_fdt_properties(HyphaMachineState *hms, void *host_fdt)
{
    void *fdt = MACHINE(hms)->fdt;
    const char *psci_compat[] = { "arm,psci-1.0", "arm,psci-0.2", "arm,psci" };
    const char *timer_compat[] = { "arm,armv8-timer", "arm,armv7-timer" };
    const char *timer_mem_compat[] = { "arm,armv8-timer-mem", "arm,armv7-timer-mem" };
    copy_basic_properties(hms, host_fdt);
    copy_node(host_fdt, fdt, "/soc", false, true);
    copy_node_compat(host_fdt, fdt, psci_compat, 3, true, false);
    copy_node_compat(host_fdt, fdt, timer_compat, 2, true, false);
    copy_node_compat(host_fdt, fdt, timer_mem_compat, 2, true, true);
    setup_gic_node(hms, host_fdt);
    setup_cpus_node(hms, host_fdt);
}

static void create_memory_node(HyphaMachineState *hms, ParasyteState *ps)
{
    void *fdt = MACHINE(hms)->fdt;
    MemMapEntry *ram_entry = parasyte_ram_entry(ps);
    MemMapEntry *hive_queue_entry = parasyte_hive_queue_entry(ps);
    MemMapEntry *spore_queue_entry = parasyte_spore_queue_entry(ps);

    char *node_path = g_strdup_printf("/memory@%lx", ram_entry->base);
    qemu_fdt_add_path(fdt, node_path);
    qemu_fdt_setprop_sized_cells(fdt, node_path, "reg", hms->address_cells, ram_entry->base, hms->size_cells, ram_entry->size);
    qemu_fdt_setprop_string(fdt, node_path, "device_type", "memory");
    g_free(node_path);

    qemu_fdt_add_path(fdt, "/reserved-memory");
    qemu_fdt_setprop_cell(fdt, "/reserved-memory", "#address-cells", 2);
    qemu_fdt_setprop_cell(fdt, "/reserved-memory", "#size-cells", 2);
    qemu_fdt_setprop(fdt, "/reserved-memory", "ranges", NULL, 0);

    node_path = g_strdup_printf("/reserved-memory/hive_queue@%lx", hive_queue_entry->base);
    qemu_fdt_add_path(fdt, node_path);
    qemu_fdt_setprop_sized_cells(fdt, node_path, "reg", 2, hive_queue_entry->base, 2, hive_queue_entry->size);
    qemu_fdt_setprop_string(fdt, node_path, "compatible", "parasyte,msg-queue-hive");
    g_free(node_path);

    node_path = g_strdup_printf("/reserved-memory/spore_queue@%lx", spore_queue_entry->base);
    qemu_fdt_add_path(fdt, node_path);
    qemu_fdt_setprop_sized_cells(fdt, node_path, "reg", 2, spore_queue_entry->base, 2, spore_queue_entry->size);
    qemu_fdt_setprop_string(fdt, node_path, "compatible", "parasyte,msg-queue-spore");
    g_free(node_path);
}

static void hypha_init_fdt(HyphaMachineState *hms, ParasyteState *ps)
{
    MachineState *ms = MACHINE(hms);
    void *host_fdt;
    int fdt_size, host_fdt_size, ret;
    g_autoptr(GError) err = NULL;

    info_report("Loading host device tree");

    host_fdt = load_device_tree(HOST_FDT_PATH, &host_fdt_size);
    if (!host_fdt)
        fatal_report("Failed to load host fdt.");

    ms->fdt = create_device_tree(&fdt_size);
    copy_host_fdt_properties(hms, host_fdt);

    ret = qemu_fdt_add_subnode(ms->fdt, "/chosen");
    if (ret < 0)
        fatal_report("Failed to add /chosen subnode");

    if (ms->kernel_cmdline && *ms->kernel_cmdline) {
        ret = qemu_fdt_setprop_string(ms->fdt, "/chosen", "bootargs", ms->kernel_cmdline);
        if (ret < 0) fatal_report("Failed to set bootargs");
    }

    struct {
        uint64_t kaslr;
        uint8_t rng[32];
    } seed;

    if (qemu_guest_getrandom(&seed, sizeof(seed), NULL)) {
        warn_report("Failed to create randomness");
        return;
    }
    qemu_fdt_setprop_u64(ms->fdt, "/chosen", "kaslr-seed", seed.kaslr);
    qemu_fdt_setprop(ms->fdt, "/chosen", "rng-seed", seed.rng, sizeof(seed.rng));
}

static void hypha_init_ram(HyphaMachineState *hms,
                           ParasyteState *ps,
                           MemoryRegion *sysmem)
{
    MemMapEntry *ram_entry = parasyte_ram_entry(ps);
    void *ram_ptr = parasyte_ram_ptr(ps);
    memory_region_init_ram_ptr(&hms->ram, NULL, "parasyte.ram", ram_entry->size, ram_ptr);
    memory_region_add_subregion(sysmem, ram_entry->base, &hms->ram);
    create_memory_node(hms, ps);
}

static void hypha_setup_virtio(HyphaMachineState *hms, ParasyteState *ps)
{
    int i;
    hwaddr base;
    uint32_t softirq;
    qemu_irq irq;
    char *node_path;
    void *fdt = MACHINE(hms)->fdt;

    /*
     * We create the transports in reverse order. Since qbus_realize()
     * prepends (not appends) new child buses, the decrementing loop below will
     * create a list of virtio-mmio buses with increasing base addresses.
     *
     * When a -device option is processed from the command line,
     * qbus_find_recursive() picks the next free virtio-mmio bus in forwards
     * order.
     *
     */
    for (i = hms->virtio_mmio_num - 1; i >= 0; i--) {
        base = hms->virtio_mmio_entry.base + i * hms->virtio_mmio_entry.size;
        softirq = hms->virtio_mmio_irq_base + i;
        irq = qemu_allocate_irq(hypha_set_irq, ps, softirq);

        sysbus_create_simple("virtio-mmio", base, irq);

        node_path = g_strdup_printf("/virtio@%" PRIx64, base);
        qemu_fdt_add_subnode(fdt, node_path);
        qemu_fdt_setprop_string(fdt, node_path, "compatible", "virtio,parasyte");
        qemu_fdt_setprop_cell(fdt, node_path, "softirq", softirq);
        qemu_fdt_setprop_sized_cells(fdt, node_path, "reg",
                                     hms->address_cells, base,
                                     hms->size_cells, hms->virtio_mmio_entry.size);
        g_free(node_path);
    }
}

static ssize_t load_image(const char *filename, void *addr)
{
    void *f;
    int fd = open(filename, O_RDONLY | O_BINARY);
    int64_t size = get_image_size(filename);

    info_report("Loading %s", filename);

    if (fd < 0 || size < 0) {
        error_report("Failed to open %s", filename);
        return -1;
    }

    f = mmap(NULL, size, PROT_READ, MAP_SHARED, fd, 0);
    if (f == MAP_FAILED) {
        error_report("Failed to mmap: %d", errno);
        close(fd);
        return -1;
    }

    memcpy(addr, f, size);
    msync(addr, size, MS_SYNC);
    munmap(f, size);
    close(fd);
    flush_idcache_range((uintptr_t)addr, (uintptr_t)addr, size);

    return size;
}

static void hypha_load_image(HyphaMachineState *hms, ParasyteState *ps)
{
    MachineState *ms = MACHINE(hms);
    MemMapEntry *ram_entry = parasyte_ram_entry(ps);
    hwaddr initrd_paddr;
    void *ram = parasyte_ram_ptr(ps);
    void *kimg = ram + parasyte_kimage_offset(ps);
    void *initrd;
    ssize_t ksize, rdsize;
    int ret;

    ksize = load_image(ms->kernel_filename, kimg);
    if (ksize < 0)
        fatal_report("Failed to load kernel image");

#define SZ_2M	0x00200000
#define SZ_32M  0x02000000

    if (ms->initrd_filename) {
        initrd_paddr = ROUND_UP(ram_entry->base + parasyte_kimage_offset(ps) + ksize + SZ_32M, SZ_2M);
        initrd = ram + (initrd_paddr - ram_entry->base);
        rdsize = load_image(ms->initrd_filename, initrd);
        if (rdsize < 0)
            fatal_report("Failed to load Initrd");

        ret = qemu_fdt_setprop_sized_cells(ms->fdt, "/chosen", "linux,initrd-start", hms->address_cells, initrd_paddr);
        if (ret < 0)
            fatal_report("Failed to set linux,initrd-start");

        ret = qemu_fdt_setprop_sized_cells(ms->fdt, "/chosen", "linux,initrd-end", hms->address_cells, initrd_paddr + rdsize);
        if (ret < 0)
            fatal_report("Failed to set linux,initrd-end");
    }


    ret = fdt_pack(ms->fdt);
    if (ret)
        fatal_report("fdt_pack before move failed: %s", fdt_strerror(ret));

    ret = fdt_move(ms->fdt, parasyte_fdt_ptr(ps), parasyte_fdt_size(ps));
    if (ret)
        fatal_report("fdt_move failed: %s", fdt_strerror(ret));

    ret = fdt_pack(parasyte_fdt_ptr(ps));
    if (ret)
        fatal_report("fdt_pack failed: %s", fdt_strerror(ret));
}

static void hypha_init(MachineState *ms)
{
    HyphaMachineState *hms = HYPHA_MACHINE(ms);
    ParasyteState *ps = PARASYTE_STATE(current_accel());
    MemoryRegion *sysmem = get_system_memory();

    if (!hms->cpus)
        fatal_report("CPUs not specified.");

    if (!hms->queue_size)
        hms->queue_size = 0x1000;

    if (!ms->ram_size)
        fatal_report("RAM size specified.");

    if (!ms->kernel_filename)
        fatal_report("Kernel image is not provided.");

    parasyte_alloc(ps, hms->cpus, ms->ram_size, hms->queue_size);
    hypha_init_fdt(hms, ps);
    hypha_init_ram(hms, ps, sysmem);
    hypha_setup_virtio(hms, ps);
    hypha_load_image(hms, ps);
}

static void hypha_class_init(ObjectClass *oc, const void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->init = hypha_init;

    mc->desc = "Parasyte Hypha machine";
    mc->max_cpus = 1;
    mc->default_machine_opts = "accel=parasyte";
    /* Set to zero to make sure that the real ram size is passed. */
    mc->default_ram_size = 0;

    object_class_property_add_str(oc, "cpus", hypha_get_cpus, hypha_set_cpus);
    object_class_property_add(oc, "queue_size", "uint64_t",
                              hypha_get_queue_size, hypha_set_queue_size,
                              NULL, NULL);
}

#define VIRTIO_MMIO_DEV_BASE    0xA0000000
#define VIRTIO_MMIO_DEV_SIZE    0x200
#define NR_VIRTIO_MMIO_DEVICES  10
#define VIRTIO_MMIO_IRQ_BASE    33

static void hypha_instance_init(Object *obj)
{
    HyphaMachineState *hms = HYPHA_MACHINE(obj);

    /* Default values.  */
    hms->virtio_mmio_num = NR_VIRTIO_MMIO_DEVICES;
    hms->virtio_mmio_irq_base = VIRTIO_MMIO_IRQ_BASE;
    hms->virtio_mmio_entry.base = VIRTIO_MMIO_DEV_BASE;
    hms->virtio_mmio_entry.size = VIRTIO_MMIO_DEV_SIZE;
}

static const TypeInfo hypha_info = {
    .name = TYPE_HYPHA_MACHINE,
    .parent = TYPE_MACHINE,
    .instance_size = sizeof(HyphaMachineState),
    .class_size = sizeof(HyphaMachineClass),
    .class_init = hypha_class_init,
    .instance_init = hypha_instance_init,
};

static void hypha_register_types(void)
{
    type_register_static(&hypha_info);
}

type_init(hypha_register_types);
