/* main.c */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usbd.h>  // Zephyr 4.x
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/usb/class/usb_cdc.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/logging/log.h>

#include "led.h"

// // LOG_MODULE_REGISTER(l2cap_rx, // LOG_LEVEL_INF);

volatile uint32_t dtr = 0;

enum BLE_STATE { // GREEN + RED
    BLE_OFF,
    ADV,
    BLE_FAILED,
    L2CAP_CONN,
} ble_state = BLE_OFF;

enum UART_STATE { // BLUE + RED
    UART_OFF,
    UART_CONNECTING,
    UART_FAILED,
    UART_CONN,
    UART_CHECK
} uart_state = UART_OFF;

const enum BLINK_STATE led_combination[4][5][3] = {
    { // BLE_OFF
        STATE_LED_OFF,              // BLE_OFF      UART_OFF,
        STATE_BLUE_BLINK,           // BLE_OFF      UART_CONNECTING,
        STATE_MAGENTA_BLINK,        // BLE_OFF      UART_FAILED,
        STATE_BLUE_ON,              // BLE_OFF      UART_CONN
        STATE_WHITE_BLINK,              // BLE_OFF      UART_CONN
    },
    { // ADV
        STATE_GREEN_BLINK,          // ADV          UART_OFF,
        STATE_GREEN_BLUE_BLINK,     // ADV          UART_CONNECTING,
        STATE_GREEN_MAGENTA_BLINK,  // ADV          UART_FAILED,
        STATE_BLUE_CYAN_BLINK,      // ADV          UART_CONN
    },
    { // BLE FAILED
        STATE_YELLOW_BLINK,    // ADV FAILED   UART_OFF,
        STATE_BLUE_YELLOW_BLINK,    // ADV FAILED   UART_CONNECTING,
        STATE_YELLOW_MAGENTA_BLINK, // ADV FAILED   UART_FAILED,
        STATE_BLUE_WHITE_BLINK,     // ADV FAILED   UART_CONN
    },
    { // L2CAP_CONN
        STATE_GREEN_ON,             // L2CAP_CONN      UART_OFF,
        STATE_GREEN_CYAN_BLINK,     // L2CAP_CONN      UART_CONNECTING,
        STATE_GREEN_WHITE_BLINK,    // L2CAP_CONN      UART_FAILED,
        STATE_CYAN_ON,              // L2CAP_CONN      UART_CONN
        STATE_GREEN_YELLOW_BLINK    // L2CAP_CONN      UART_CHECK
    }
};

void update_led_state(){
    for (int i = 0; i < 3; i++){
        set_led_state(i, led_combination[ble_state][uart_state][i]);
    }
}

void set_ble_state(enum BLE_STATE state){
    if(ble_state == state) return;
    ble_state = state;
    update_led_state();
}
void set_uart_state(enum UART_STATE state){
    if(uart_state == state) return;
    uart_state = state;
    update_led_state();
}

/* Protocol & MTU Config */
#define L2CAP_COC_PSM 0x0080
#define L2CAP_COC_MTU 253
#define L2CAP_COC_MPS 247
#define L2CAP_COC_CREDITS 10    // 초기 크레딧 (충분한 버퍼 여유)
#define PACKET_SIZE 220

#define RING_BUF_SIZE 2200  // 218 * (10) 버퍼
static uint8_t ring_buf_mem[RING_BUF_SIZE];
static struct ring_buf tx_ringbuf;
static K_SEM_DEFINE(tx_done_sem, 0, 1);  // 전송 완료 세마포어

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

static void adv_work_handler(struct k_work *work) {
    int err = bt_le_adv_start(adv_param, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err && err != -EALREADY) { // 이미 광고 중인 에러는 무시
        set_ble_state(BLE_FAILED);
    } else {
        set_ble_state(ADV); // 광고 중중
    }
}

static void start_advertising(void) {
	k_work_submit(&adv_work);
}

static void connected(struct bt_conn *conn, uint8_t err) {
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
// #define UART_DEVICE_NODE DT_NODELABEL(cdc_acm_uart0)
// #define UART_DEVICE_NODE DT_NODELABEL(uart0)
// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);


K_SEM_DEFINE(tx_data_sem, 0, 1);
static void uart_tx_func() {
    uint8_t* buf;
    set_uart_state(UART_CHECK);
    int len;
    int sent;

    while(true){
        k_sem_take(&tx_data_sem, K_FOREVER);
        len = ring_buf_get_claim(&tx_ringbuf, &buf, PACKET_SIZE * sizeof(uint8_t)); // 얻는 동시에 링버퍼에서 제거
        
        if (len > 0) {      
            for (uint32_t i = 0; i < len; i++) {
                uart_poll_out(uart_dev, buf[i]);
            }

            if(ring_buf_get_finish(&tx_ringbuf, len) < 0){
                set_uart_state(UART_FAILED);
            } 
        }
        if (!ring_buf_is_empty(&tx_ringbuf)) {
            k_sem_give(&tx_data_sem);
        }

    }

}

K_THREAD_DEFINE(uart_tx_thread, 1024, uart_tx_func, NULL, NULL, NULL, 128, 0, 0);

/* 218바이트 패킷 전송 함수 */
static void send_packet(const uint8_t *data, size_t len)
{
    /* 1. Ring buffer에 전체 패킷 넣기 */
    int ret;
    const uint8_t terminator[2] = {0x0D, 0x0A};  // "\r\n"
    // uint32_t key = irq_lock();

    int put = ring_buf_put(&tx_ringbuf, data, len);
    // irq_unlock(key);

    if (put < len) {
        set_uart_state(UART_FAILED);
        return;
    }
    put = ring_buf_put(&tx_ringbuf, terminator, 2);
    if (put < 2) {
        set_uart_state(UART_FAILED);
        return;
    }
    k_sem_give(&tx_data_sem);  
    
    return;
}

static int l2cap_recv(struct bt_l2cap_chan *chan, struct net_buf *buf){
    uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr); 
    if(dtr){
        send_packet(buf->data, buf->len);
    }
    return 0;
}

/* [Callback 2] Connected */
static void l2cap_connected(struct bt_l2cap_chan *chan) {
    set_ble_state(L2CAP_CONN);
}

/* [Callback 3] Disconnected */
static void l2cap_disconnected(struct bt_l2cap_chan *chan) {
	chan->conn = NULL;
}
#define L2CAP_RX_BUF_COUNT 4
#define L2CAP_RX_BUF_SIZE 256

NET_BUF_POOL_FIXED_DEFINE(l2cap_rx_pool, 
                          L2CAP_RX_BUF_COUNT,
                          L2CAP_RX_BUF_SIZE, 
                          CONFIG_BT_CONN_TX_USER_DATA_SIZE,
                          NULL);
/* [추가] alloc_buf 콜백 - 수신 버퍼 할당 */
static struct net_buf *l2cap_alloc_buf(struct bt_l2cap_chan *chan)
{
    /* 풀에서 버퍼 할당 */
    return net_buf_alloc(&l2cap_rx_pool, K_NO_WAIT);
}
/* Operations VTable */
static const struct bt_l2cap_chan_ops l2cap_ops = {
    .alloc_buf    = l2cap_alloc_buf,  // [필수] 추가
    .recv         = l2cap_recv,
    .connected    = l2cap_connected,
    .disconnected = l2cap_disconnected,
};

static int l2cap_accept(struct bt_conn *conn, struct bt_l2cap_server *server, struct bt_l2cap_chan **chan) {
	if (my_le_chan.chan.conn) {
		return -ENOMEM;
	}
	memset(&my_le_chan, 0, sizeof(my_le_chan));
    /* 2. Base 멤버 설정 */
    my_le_chan.chan.ops = &l2cap_ops;

    my_le_chan.rx.mtu = L2CAP_COC_MTU;
    my_le_chan.rx.mps = L2CAP_COC_MPS;  // [추가] MPS도 같은 값으로
    atomic_set(&my_le_chan.rx.credits, L2CAP_COC_CREDITS);  // [필수] 초기 크레딧
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

int configureUART(){
    int err;
    err = !device_is_ready(uart_dev);
    if (err) {
        set_uart_state(UART_FAILED);
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
    if (err) {
        set_uart_state(UART_FAILED);
        return -1;
    }

    ring_buf_init(&tx_ringbuf, sizeof(ring_buf_mem), ring_buf_mem);

    set_uart_state(UART_CONNECTING);
    k_thread_start(&uart_tx_thread);
    return 0;
}

int main(void) {
    int err;

	// LOG_INF("L2CAP Receiver Start");
    configure_led();
    /* 1. USB 초기화 (Console over USB) */
    err = configureUART();

    if(err){
        return err;
    }
    err = bt_conn_cb_register(&conn_callbacks);

    /* Bluetooth Init */
    err = bt_enable(NULL);
    if (err) {
        set_ble_state(BLE_FAILED);
        return 0;
    }

    err = bt_l2cap_server_register(&server);

    if (err) {
        set_ble_state(BLE_FAILED);
        return 0;
    }
	else {
		k_sleep(K_MSEC(100));

        k_work_init(&adv_work, adv_work_handler);
		start_advertising();

        k_sleep(K_MSEC(1000));

		while(true){
            uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr); 
            if(!dtr && uart_state != UART_CONNECTING) { 
                set_uart_state(UART_CONNECTING);
                uart_irq_tx_disable(uart_dev);
                ring_buf_reset(&tx_ringbuf);
                k_sem_reset(&tx_done_sem);

            } else if(dtr && uart_state == UART_CONNECTING){
                set_uart_state(UART_CONN);
            }
        
			if(ble_state == BLE_FAILED){
                k_work_submit(&adv_work);
            }
            k_sleep(K_MSEC(100));
		}
	}
    return 0;
}