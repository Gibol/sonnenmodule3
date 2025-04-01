#include "can.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>

#define RX_THREAD_STACK_SIZE 512
#define RX_THREAD_PRIORITY 2
#define STATE_POLL_THREAD_STACK_SIZE 512
#define STATE_POLL_THREAD_PRIORITY 2
#define SLEEP_TIME K_MSEC(250)

K_THREAD_STACK_DEFINE(rx_thread_stack, RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_state_stack, STATE_POLL_THREAD_STACK_SIZE);

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

struct k_thread rx_thread_data;
struct k_thread poll_state_thread_data;
struct k_work state_change_work;
enum can_state current_state;
struct can_bus_err_cnt current_err_cnt;

CAN_MSGQ_DEFINE(counter_msgq, 64);

void tx_irq_callback(const struct device *dev, int error, void *arg)
{
    char *sender = (char *)arg;

    ARG_UNUSED(dev);

    if (error != 0)
    {
        printf("Callback! error-code: %d\nSender: %s\n",
               error, sender);
    }
}

void rx_thread(void *arg1, void *arg2, void *arg3)
{
    (void) arg2;
    (void) arg3;

    CAN_RxCallback rx_callback = arg1;

    const struct can_filter filters[2] = {
        // Filter 1: Match exactly 0x4200 (extended ID)
        {
            .flags = CAN_FILTER_IDE,
            .id = 0x4200,
            .mask = CAN_EXT_ID_MASK,  // Full match on all bits
        },
        // Filter 2: Match any ID of the form 0x11DDxxxx
        {
            .flags = CAN_FILTER_IDE,
            .id = 0x11DD0000,
            .mask = 0xFFFF0000,  // Only match the top 16 bits
        }
    };

    struct can_frame frame;

    int filter_id;

    filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filters[0]);
    //printf("Counter filter id: %d\n", filter_id);

    filter_id = can_add_rx_filter_msgq(can_dev, &counter_msgq, &filters[1]);
    //printf("Counter filter id: %d\n", filter_id);

    while (1)
    {
        k_msgq_get(&counter_msgq, &frame, K_FOREVER);
        // printk("flags: %d\n", frame.flags);
        int rtr = (frame.flags & CAN_FRAME_RTR);

        rx_callback(frame.id, rtr, frame.data, frame.dlc);
    }
}

char *state_to_str(enum can_state state)
{
    switch (state)
    {
    case CAN_STATE_ERROR_ACTIVE:
        return "error-active";
    case CAN_STATE_ERROR_WARNING:
        return "error-warning";
    case CAN_STATE_ERROR_PASSIVE:
        return "error-passive";
    case CAN_STATE_BUS_OFF:
        return "bus-off";
    case CAN_STATE_STOPPED:
        return "stopped";
    default:
        return "unknown";
    }
}

void poll_state_thread(void *unused1, void *unused2, void *unused3)
{
    struct can_bus_err_cnt err_cnt = {0, 0};
    struct can_bus_err_cnt err_cnt_prev = {0, 0};
    enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;
    enum can_state state;
    int err;

    while (1)
    {
        err = can_get_state(can_dev, &state, &err_cnt);
        if (err != 0)
        {
            printf("Failed to get CAN controller state: %d", err);
            k_sleep(K_MSEC(100));
            continue;
        }

        if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
            err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt ||
            state_prev != state)
        {

            err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
            err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
            state_prev = state;
            printf("state: %s\n"
                   "rx error count: %d\n"
                   "tx error count: %d\n",
                   state_to_str(state),
                   err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);
        }
        else
        {
            k_sleep(K_MSEC(100));
        }
    }
}

void state_change_work_handler(struct k_work *work)
{
    printf("State Change ISR\nstate: %s\n"
           "rx error count: %d\n"
           "tx error count: %d\n",
           state_to_str(current_state),
           current_err_cnt.rx_err_cnt, current_err_cnt.tx_err_cnt);
}

void state_change_callback(const struct device *dev, enum can_state state,
                           struct can_bus_err_cnt err_cnt, void *user_data)
{
    struct k_work *work = (struct k_work *)user_data;

    ARG_UNUSED(dev);

    current_state = state;
    current_err_cnt = err_cnt;
    k_work_submit(work);
}

k_tid_t rx_tid, get_state_tid;

int CAN_Initialize(CAN_RxCallback rxCallback)
{
    int ret;
    if (!device_is_ready(can_dev))
    {
        printk("CAN: Device %s not ready.\n", can_dev->name);
        return CAN_ERROR;
    }

    ret = can_start(can_dev);
    if (ret != 0)
    {
        printf("Error starting CAN controller [%d]", ret);
        return CAN_ERROR;
    }

    k_work_init(&state_change_work, state_change_work_handler);

    rx_tid = k_thread_create(&rx_thread_data, rx_thread_stack,
                             K_THREAD_STACK_SIZEOF(rx_thread_stack),
                             rx_thread, rxCallback, NULL, NULL,
                             RX_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!rx_tid)
    {
        printk("ERROR spawning rx thread\n");
        return CAN_ERROR;
    }

    get_state_tid = k_thread_create(&poll_state_thread_data,
                                    poll_state_stack,
                                    K_THREAD_STACK_SIZEOF(poll_state_stack),
                                    poll_state_thread, NULL, NULL, NULL,
                                    STATE_POLL_THREAD_PRIORITY, 0,
                                    K_NO_WAIT);
    if (!get_state_tid)
    {
        printk("ERROR spawning poll_state_thread\n");
        return CAN_ERROR;
    }

    can_set_state_change_callback(can_dev, state_change_callback, &state_change_work);

    printk("Finished CAN init.\n");

    return CAN_SUCCESS;
}

int CAN_Send(uint32_t id, uint8_t *data, uint8_t dataLen)
{
    //printk("CAN_Send to: %x\n", id);
    struct can_frame frame = {
        .flags = CAN_FRAME_IDE,
        .id = id,
        .dlc = dataLen};

    memcpy(frame.data, data, dataLen);

    int retry = 5;
    int ret = -1;

    while (retry && ret)
    {
        ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
        if (ret != 0)
        {
            printk("can_send failed: %d\n", ret);
        }
    }

    if (ret != 0)
    {
        return CAN_ERROR;
    }
    return CAN_SUCCESS;
}
