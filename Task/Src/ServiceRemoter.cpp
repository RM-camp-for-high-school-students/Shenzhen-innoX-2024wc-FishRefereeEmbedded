#include "main.h"
#include "usart.h"
#include "RefMessage.h"
#include "om.h"

uint8_t data_rx_buf[20];

TX_THREAD RemoterThread;
uint8_t RemoterThreadStack[1024] = {0};
TX_SEMAPHORE RemoterThreadSem;

[[noreturn]] void RemoterThreadFun(ULONG initial_input) {
    UNUSED(initial_input);
    /* Remoter Topic */
    om_topic_t *remoter_topic = om_find_topic("Remoter", UINT32_MAX);

    Msg_Remoter_Judge_t msg_remoter{};
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_rx_buf, 20);

    for (;;) {
        if (tx_semaphore_get(&RemoterThreadSem, 100) != TX_SUCCESS) {
            msg_remoter.connected = false;
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
            tx_semaphore_get(&RemoterThreadSem, TX_WAIT_FOREVER);
        } else {
            msg_remoter.connected = true;
            om_publish(remoter_topic, &msg_remoter, sizeof(msg_remoter), true, false);
        }
    }
}

extern TX_SEMAPHORE RefereeRXSem;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, data_rx_buf, 20);
        if (Size == 18) {
            tx_semaphore_put(&RemoterThreadSem);
        }
    }
    if (huart == &huart4){
        sys_state.rx_len = Size;
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, sys_state.uart_rx_buf, sys_state.buf_len);
        tx_semaphore_put(&RefereeRXSem);
    }
}
