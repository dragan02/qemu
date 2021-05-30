/**
 * @file custom_i2c.c
 * @brief Custom memory mapped sensor component
 *
 * File represents custom memory mapped sensor component which stores
 * shared memory value in its' register after specific period of time.
 *
 * @date 2021
 * @author Dragan Bozinovic (bozinovicdragan96@gmail.com)
 *
 * @version [1.0 @ 05/2021] Initial version
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/main-loop.h"
#include "qemu/log.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "hw/register.h"

/* Aditional includes, needed for shared memory */
#include <fcntl.h>      /* Defines O_* constants */
#include <sys/stat.h>   /* Defines mode constants */
#include <sys/mman.h>   /* Defines mmap flags */

#define TYPE_CUSTOM_MM_SENS "custom.mmsens"

#define CUSTOM_MM_SENS(obj) \
    OBJECT_CHECK(CustomMMSensor, (obj), TYPE_CUSTOM_MM_SENS)


#ifndef CUSTOM_MM_SENS_ERR_DEBUG
#define CUSTOM_MM_SENS_ERR_DEBUG 0
#endif

#define DB_PRINT(fmt, args...) do { \
    if (CUSTOM_MM_SENS_ERR_DEBUG) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

/* Registers */
REG32(CTRL, 0x00)
    FIELD(CTRL,     EN,         0,  1)      /* Component enable */
    FIELD(CTRL,     IEN,        1,  1)      /* Interrupt enable */

REG32(STATUS, 0x04)
    FIELD(STATUS,   IFG,        1,  1)      /* Interrupt flag */

REG32(DATA, 0x08)
    FIELD(DATA,     SAMPLE,     0,  8)      /* Current value */

/* Number of registers */
#define R_MAX   ((R_DATA) + 1)

/* Frequency of updating data register with value from shared memory */
#define DATA_UPDATE_FREQ    (1)

/* Function which makes shared memory segment and returns pointer to it */
static void * get_shmem_pointer(void)
{
    int fd, seg_size;
    void *addr;

    DB_PRINT("MMS shared memory initialization\n");

    /* Auxiliary variable for storing size of shared memory */
    seg_size = sizeof(uint32_t);

    /* Create new shared memory object */
    fd = shm_open("mmsens", O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);
    if (fd == -1){
        DB_PRINT("Function shm_open failed\n");
    }

    /* Set sh. mem. segment size */
    if (ftruncate(fd, seg_size) == -1){
        DB_PRINT("Truncating shared memory failed\n");
    }

    /* Map shared memory object to process virtual address space */
    addr = (uint32_t*)mmap(NULL, seg_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (addr == MAP_FAILED) {
        DB_PRINT("Memory mapping failed\n");
    }

    /* Close appropriate sh. mem. object */
    if (close(fd) == -1){
        DB_PRINT("Closing shmem file descriptor failed\n");
    }

    return addr;
}

/* Simple memory mapped sensor which reads value from shared memory. */
typedef struct CustomMMSensor {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    ptimer_state *timer;
    QEMUBH *bh;
    
    uint8_t *shmem;                 /* Shared memory pointer */

    uint32_t regs[R_MAX];
    RegisterInfo regs_info[R_MAX];
} CustomMMSensor;

/*
 * IRQ generator. If alarm is enabled and is set, trigger interrupt.
 */
static void custom_mm_sens_update_irq(CustomMMSensor *s)
{
    bool pending = s->regs[R_CTRL] & s->regs[R_STATUS] & R_CTRL_IEN_MASK;

    DB_PRINT("Interrupt %s\n", pending ? "generated" : "none");

    qemu_set_irq(s->irq, pending);
}

/*
 * Update measured data. Read data from shared memory and update data register.
 */
static void custom_mm_sens_update_data(void *opaque)
{
    CustomMMSensor *s = CUSTOM_MM_SENS(opaque);
    
    s->regs[R_DATA] = *s->shmem;

    s->regs[R_STATUS] |= R_STATUS_IFG_MASK;

    DB_PRINT("Updating data %d\n", s->regs[R_DATA]);

    custom_mm_sens_update_irq(s);
}

/*
 * Reset component registers and variables.
 */
static void custom_mm_sens_reset(DeviceState *dev)
{
    CustomMMSensor *s = CUSTOM_MM_SENS(dev);
    int i;

    for (i = 0; i < R_MAX; ++i) {
        register_reset(&s->regs_info[i]);
    }
}

/*
 * CTRL register updates
 * 
 * If component is enabled, start timer, else stop timer.
 * If interrupt is enabled, check if interrupt needs to be generated.
 */
static void r_ctrl_post_write(RegisterInfo *reg, uint64_t val)
{
    CustomMMSensor *s = CUSTOM_MM_SENS(reg->opaque);
    

    if (s->regs[R_CTRL] & R_CTRL_EN_MASK) {
        /* Start timer if not started*/
        ptimer_run(s->timer, 0);

        if (s->regs[R_CTRL] & R_CTRL_IEN_MASK) {
            /* Check if alarm should be triggered */
            custom_mm_sens_update_irq(s);
        }
    } else {
        /* Stop timer */
        ptimer_stop(s->timer);
    }
}

/*
 * STATUS register updates
 * 
 * Clear interrupt flag
 */
static void r_status_post_write(RegisterInfo *reg, uint64_t val)
{
    CustomMMSensor *s = CUSTOM_MM_SENS(reg->opaque);

    DB_PRINT("Wrote %lu to STATUS\n", val);

    custom_mm_sens_update_irq(s);
}

static const RegisterAccessInfo custom_mm_sens_regs_info[] = {
    {   .name = "CTRL",           .addr = A_CTRL,
        .reset = 0,
        .rsvd = ~(R_CTRL_EN_MASK | R_CTRL_IEN_MASK),
        .post_write = r_ctrl_post_write,
    },
    {   .name = "STATUS",           .addr = A_STATUS,
        .reset = 0,
        .rsvd = ~R_STATUS_IFG_MASK,
        .post_write = r_status_post_write,
    },
    {   .name = "DATA",         .addr = A_DATA,
        .reset = 0,
        .rsvd = ~R_DATA_SAMPLE_MASK,
        .ro = R_DATA_SAMPLE_MASK,
    },
};

static const MemoryRegionOps custom_mm_sens_reg_ops = {
    .read = register_read_memory,
    .write = register_write_memory,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static const VMStateDescription vmstate_custom_mm_sens = {
    .name = "custom_mm_sens_cmd",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(regs, CustomMMSensor, R_MAX),
        VMSTATE_PTIMER(timer, CustomMMSensor),
        VMSTATE_END_OF_LIST()
    }
};

static void custom_mm_sens_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    CustomMMSensor *s = CUSTOM_MM_SENS(obj);
    RegisterInfoArray *reg_array;

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init(&s->iomem, obj, "custom.mmsens", R_MAX * 4);
    reg_array = register_init_block32(DEVICE(obj), custom_mm_sens_regs_info,
                                      ARRAY_SIZE(custom_mm_sens_regs_info),
                                      s->regs_info, s->regs,
                                      &custom_mm_sens_reg_ops,
                                      CUSTOM_MM_SENS_ERR_DEBUG,
                                      R_MAX * 4);
    memory_region_add_subregion(&s->iomem,
                                A_CTRL,
                                &reg_array->mem);

    sysbus_init_mmio(sbd, &s->iomem);
    
    s->shmem = get_shmem_pointer();

    s->bh = qemu_bh_new(custom_mm_sens_update_data, s);
    s->timer = ptimer_init(s->bh, PTIMER_POLICY_CONTINUOUS_TRIGGER);
    ptimer_set_freq(s->timer, DATA_UPDATE_FREQ);
}

static void custom_mm_sens_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = custom_mm_sens_reset;
    dc->vmsd = &vmstate_custom_mm_sens;
}

static const TypeInfo custom_mm_sens_info = {
    .name           = TYPE_CUSTOM_MM_SENS,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(CustomMMSensor),
    .instance_init  = custom_mm_sens_init,
    .class_init     = custom_mm_sens_class_init,
};

static void custom_mm_sens_register_types(void)
{
    type_register_static(&custom_mm_sens_info);
}

type_init(custom_mm_sens_register_types)
