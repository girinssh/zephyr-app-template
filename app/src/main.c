/* main.c */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usbd.h>  // Zephyr 4.x
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/logging/log.h>

#include "led.h"

// // LOG_MODULE_REGISTER(l2cap_rx, // LOG_LEVEL_INF);

enum BLE_STATE {
    OFF,
    BLE_FAILED,
    ADV,
    ADV_FAILED,
    L2CAP_CONN,
    UART_SEND_FAILED,
} ble_state;

const enum BLINK_STATE led_combination[][3] = {
    {BLINK_STOP_LED_OFF , BLINK_STOP_LED_OFF , BLINK_STOP_LED_OFF   }, // OFF          / OFF
    {BLINK_STOP_LED_ON  , BLINK_STOP_LED_OFF , BLINK_STOP_LED_ON    }, // BLE_FAILED   / RED-BLUE BLINK
    {BLINK_STOP_LED_OFF , BLINK_START        , BLINK_STOP_LED_OFF   }, // ADV          / GREEN BLINK
    {BLINK_STOP_LED_ON  , BLINK_START        , BLINK_STOP_LED_OFF   }, // ADV_FAILED   / RED-GREEN BLINK
    {BLINK_STOP_LED_OFF , BLINK_STOP_LED_OFF , BLINK_STOP_LED_ON    }, // L2CAP_CONN   / BLUE BLINK
    {BLINK_START        , BLINK_STOP_LED_OFF , BLINK_STOP_LED_OFF   }, // L2CAP_CONN   / RED BLINK
};

void set_ble_state(enum BLE_STATE state){
    ble_state = state;
    for (int i = 0; i < 3; i++){
        set_led_state(i, led_combination[state][i]);
    }
}


/* Protocol & MTU Config */
#define L2CAP_COC_PSM 0x0080
#define L2CAP_COC_MTU 253
#define PACKET_SIZE 218

#define RING_BUF_SIZE 4360  // 218 * (20) 버퍼
static uint8_t ring_buf_mem[RING_BUF_SIZE];
static struct ring_buf tx_ringbuf;
static K_SEM_DEFINE(tx_done_sem, 0, 1);  // 전송 완료 세마포어

/* Memory Pool */
// NET_BUF_POOL_DEFINE(l2cap_rx_pool, 20, 256, 0, NULL);

/* 전역 채널 객체 (LE 전용 구조체 사용) */
static struct bt_l2cap_le_chan my_le_chan;

static struct bt_conn *current_conn = NULL; // 현재 연결된 기기 추적
static struct k_work adv_work;

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param *adv_param = BT_LE_ADV_CONN_FAST_1;

static void adv_work_handler(struct k_work *work)
{
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err && err != -EALREADY) { // 이미 광고 중인 에러는 무시
        set_ble_state(ADV_FAILED);
    } else {
        // // LOG_INF("Advertising started...");
        set_ble_state(ADV); // 광고 중중
    }
}

static void start_advertising(void)
{
    // // LOG_INF("Bluetooth Disconnected. Advertising...");
	k_work_submit(&adv_work);
    // set_blink_interval(500);
    // blink_start();
}

/* --- [2] Connection Callback (재연결 및 속도 제어 핵심) --- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        // // LOG_ERR("Connection failed (err 0x%02x)", err);
        if (current_conn) {
            bt_conn_unref(current_conn);
            current_conn = NULL;
        }
        return;
    }

    // // LOG_INF("Bluetooth Central Connected!");
    /* 기존 참조가 있다면 해제 (방어 코드) */
    if (current_conn) {
        bt_conn_unref(current_conn);
    }
	
	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    start_advertising();
}

/* Connection Callbacks 등록 */
struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
};

#define UART_DEVICE_NODE DT_NODELABEL(board_cdc_acm_uart)
// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

uint8_t buf[PACKET_SIZE] = {0};

static void uart_tx_isr(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    if (!uart_irq_update(dev)) {
        return;
    }

    if (uart_irq_tx_ready(dev)) {
        int len = ring_buf_get(&tx_ringbuf, buf, PACKET_SIZE * sizeof(uint8_t));
        
        if (len == 0) {
            /* 링버퍼 비었음 - TX interrupt 끄기 */
            uart_irq_tx_disable(dev);
            k_sem_give(&tx_done_sem);  // 전송 완료 신호
            return;
        }

        /* UART FIFO에 쓰기 */
        int sent = uart_fifo_fill(dev, buf, PACKET_SIZE);
        if (sent < PACKET_SIZE) {
            /* 못 보낸 데이터 다시 넣기 (보통 발생 안 함) */
            ring_buf_put(&tx_ringbuf, &buf[sent], PACKET_SIZE - sent);
        }
    }
}

/* 218바이트 패킷 전송 함수 */
static int send_packet(const uint8_t *data, size_t len)
{
    /* 1. Ring buffer에 전체 패킷 넣기 */
    uint32_t key = irq_lock();
    int put = ring_buf_put(&tx_ringbuf, data, len);
    irq_unlock(key);

    if (put < len) {
        /* 버퍼 부족 - 에러 처리 */
        return -ENOMEM;
    }

    /* 2. TX interrupt 활성화 (자동 전송 시작) */
    uart_irq_tx_enable(uart_dev);

    return k_sem_take(&tx_done_sem, K_MSEC(10));
}



static int l2cap_recv(struct bt_l2cap_chan *chan, struct net_buf *buf)
{
    send_packet(buf->data, buf->len);

    return 0;

    // return uart_tx(uart_dev, buf->data, buf->len, 1000);
}

/* [Callback 2] Connected */
static void l2cap_connected(struct bt_l2cap_chan *chan)
{
    /* Base 포인터를 LE 구조체로 변환하여 MTU 정보 등을 확인 */
    // struct bt_l2cap_le_chan *le_chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);

    set_ble_state(L2CAP_CONN);
    // LOG_INF("L2CAP Connected!");
    // LOG_INF(" - RX MTU: %u", le_chan->rx.mtu);
    // LOG_INF(" - TX MTU: %u", le_chan->tx.mtu);
}

/* [Callback 3] Disconnected */
static void l2cap_disconnected(struct bt_l2cap_chan *chan)
{
    // LOG_INF("L2CAP Disconnected");
	chan->conn = NULL;
}

/* [Callback 4] Alloc Buffer */
// static struct net_buf *l2cap_alloc_buf(struct bt_l2cap_chan *chan)
// {
//     /* L2CAP 전용 풀에서 할당 */
//     return net_buf_alloc(&l2cap_rx_pool, K_NO_WAIT);
// }

/* Operations VTable */
static const struct bt_l2cap_chan_ops l2cap_ops = {
    // .alloc_buf    = l2cap_alloc_buf,
    .recv         = l2cap_recv,
    .connected    = l2cap_connected,
    .disconnected = l2cap_disconnected,
};

static int l2cap_accept(struct bt_conn *conn, struct bt_l2cap_server *server, struct bt_l2cap_chan **chan)
{
    // LOG_INF("Incoming L2CAP Connection Request");

	/* [수정 1] 이미 채널이 사용 중(연결됨)이라면 거절해야 안전합니다 */
	if (my_le_chan.chan.conn) {
		// LOG_ERR("Channel struct is still busy!");
		return -ENOMEM;
	}
	memset(&my_le_chan, 0, sizeof(my_le_chan));
    /* 2. Base 멤버 설정 */
    my_le_chan.chan.ops = &l2cap_ops;
    my_le_chan.rx.mtu = L2CAP_COC_MTU;
    
    /* 4. Base 포인터 반환 (Casting) */
    *chan = &my_le_chan.chan;

    return 0; /* Accept */
}

/* Server Definition */
static struct bt_l2cap_server server = {
    .psm       = L2CAP_COC_PSM,
    .sec_level = BT_SECURITY_L1,
    .accept    = l2cap_accept,
};


int main(void)
{
    int err;
    uint32_t dtr = 0;

	// LOG_INF("L2CAP Receiver Start");
    configure_led();
    /* 1. USB 초기화 (Console over USB) */
    if (!device_is_ready(uart_dev)) {
        return -1;
    }

    const struct uart_config uart_cfg = {
        .baudrate = 921600,
        .parity = UART_CFG_PARITY_NONE,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .data_bits = UART_CFG_DATA_BITS_8,
        .stop_bits = UART_CFG_STOP_BITS_1,
    };

    err = uart_configure(uart_dev, &uart_cfg);

    ring_buf_init(&tx_ringbuf, sizeof(ring_buf_mem), ring_buf_mem);

    /* UART interrupt 설정 */
    uart_irq_callback_set(uart_dev, uart_tx_isr);

    k_msleep(1000);

    err = bt_conn_cb_register(&conn_callbacks);

    /* Bluetooth Init */
    err = bt_enable(NULL);
    if (err) {
        // LOG_ERR("bt_enable failed (err %d)", err);
        return 0;
    }

    /* Register Server */
    err = bt_l2cap_server_register(&server);

    if (err) {
        // LOG_ERR("Server register failed (err %d)", err);
    }
	else {
 		// LOG_INF("L2CAP Server registered (PSM 0x%04x)", L2CAP_COC_PSM);

		k_sleep(K_MSEC(100));

        k_work_init(&adv_work, adv_work_handler);
		start_advertising();
		while(true){
			// // LOG_INF("Receiver is Running...");
			if(ble_state == ADV_FAILED){
                set_ble_state((enum BLE_STATE)OFF);
                k_work_submit(&adv_work);
            }
            k_sleep(K_SECONDS(1));
		}
	}
    return 0;
}