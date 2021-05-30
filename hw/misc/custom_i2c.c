/**
 * @file custom_i2c.c
 * @brief Custom I2C sensor component
 *
 * File represents custom I2C slave component which stores shared memory value
 * in its' register and transmits it to master when needed.
 *
 * @date 2021
 * @author Dragan Bozinovic (bozinovicdragan96@gmail.com)
 *
 * @version [1.0 @ 05/2021] Initial version
 */

#include "qemu/osdep.h"
#include "qemu-common.h"
#include "qemu/log.h"
#include "hw/i2c/i2c.h"

/* Aditional includes, needed for shared memory */
#include <fcntl.h>      /* Defines O_* constants */
#include <sys/stat.h>   /* Defines mode constants */
#include <sys/mman.h>   /* Defines mmap flags */

#define TYPE_CUSTOM_I2C_SENS "custom.i2csens"

#define CUSTOM_I2C_SENS(obj) \
    OBJECT_CHECK(CustomI2CSensor, (obj), TYPE_CUSTOM_I2C_SENS)

#ifndef DEBUG_CUSTOM_I2C_SENS
#define DEBUG_CUSTOM_I2C_SENS 0
#endif

#define DB_PRINT(fmt, args...) do { \
    if (DEBUG_CUSTOM_I2C_SENS) { \
        qemu_log("%s: " fmt, __func__, ## args); \
    } \
} while (0)

/* Registers */
#define REG_CTRL_OFFSET                 (0x0)
#define REG_DATA_OFFSET                 (0x1)
#define NUM_REGS                         (2)
#define REG_CTRL_EN_MASK                (0x01)

/* Function which makes shared memory segment and returns pointer to it */
static void * get_shmem_pointer(void)
{
    int fd, seg_size;
    void *addr;

    DB_PRINT("I2C shared memory initialization\n");

    /* Auxiliary variable for storing size of shared memory */
    seg_size = sizeof(uint32_t);

    /* Create new shared memory object */
    fd = shm_open("i2c", O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);
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

/* Simple I2C slave which reads value from shared memory. */
typedef struct CustomI2CSensor {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/
    uint8_t regs[NUM_REGS];         // peripheral registers
    uint8_t count;                  // counter used for tx/rx
    uint8_t ptr;                    // current register index
    uint8_t *shmem;                 // shared memory pointer
} CustomI2CSensor;

/* Reset counter and current register index */
static void custom_i2c_sens_reset(DeviceState *ds)
{
    CustomI2CSensor *s = CUSTOM_I2C_SENS(ds);

    memset(s->regs, 0, NUM_REGS);
    s->count = 0;
}

/* Check for read event for master. If peripheral is enabled, read value
 * from shared memory, otherwise load 0x00.
 */
static int custom_i2c_sens_event(I2CSlave *i2c, enum i2c_event event)
{
    CustomI2CSensor *s = CUSTOM_I2C_SENS(i2c);

    if (event == I2C_START_RECV) {
        if ((s->ptr == REG_DATA_OFFSET))
        {
            if (s->regs[REG_CTRL_OFFSET] & REG_CTRL_EN_MASK)
            {
                s->regs[REG_DATA_OFFSET] = *s->shmem;
            }
            else
            {
                s->regs[REG_DATA_OFFSET] = 0x00;
            }
        }
    }

    s->count = 0;

    return 0;
}

/* Called when master requests read */
static int custom_i2c_sens_rx(I2CSlave *i2c)
{
    CustomI2CSensor *s = CUSTOM_I2C_SENS(i2c);

    if (s->ptr >= NUM_REGS)
    {
        return 0xff;
    }
    else
    {
        return s->regs[s->ptr++];
    }
}

/* Called when master sends write */
static int custom_i2c_sens_tx(I2CSlave *i2c, uint8_t data)
{
    CustomI2CSensor *s = CUSTOM_I2C_SENS(i2c);

    if (s->count == 0)
    {
        /* Store register address */
        s->ptr = data;
        s->count++;
    }
    else
    {
        if (s->ptr == REG_CTRL_OFFSET)
        {
            s->regs[s->ptr++] = data;
        }
    }

    return 0;
}

/* Initialization */
static void custom_i2c_sens_init(Object *obj)
{
    CustomI2CSensor *s = CUSTOM_I2C_SENS(obj);

    memset(s->regs, 0, NUM_REGS);
    s->count = 0;
    s->ptr = 0;
    
    s->shmem = get_shmem_pointer();

    return;
}

static const VMStateDescription vmstate_custom_i2c_sens = {
    .name = TYPE_CUSTOM_I2C_SENS,
    .version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(regs, CustomI2CSensor, NUM_REGS),
        VMSTATE_UINT8(count, CustomI2CSensor),
        VMSTATE_UINT8(ptr, CustomI2CSensor),
        VMSTATE_END_OF_LIST()
    }
};



static void custom_i2c_sens_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    I2CSlaveClass *isc = I2C_SLAVE_CLASS(oc);

    dc->reset = custom_i2c_sens_reset;
    dc->vmsd = &vmstate_custom_i2c_sens;
    isc->event = custom_i2c_sens_event;
    isc->recv = custom_i2c_sens_rx;
    isc->send = custom_i2c_sens_tx;
}

static TypeInfo custom_i2c_sens_info = {
    .name = TYPE_CUSTOM_I2C_SENS,
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(CustomI2CSensor),
    .instance_init = custom_i2c_sens_init,
    .class_init = custom_i2c_sens_class_init
};

static void custom_i2c_sens_register_devices(void)
{
    type_register_static(&custom_i2c_sens_info);
}

type_init(custom_i2c_sens_register_devices);

