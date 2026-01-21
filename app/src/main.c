/* main.c */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(l2cap_rx, LOG_LEVEL_INF);

/* Protocol & MTU Config */
#define L2CAP_COC_PSM 0x0080
#define L2CAP_COC_MTU 253

/* Memory Pool */
NET_BUF_POOL_DEFINE(l2cap_rx_pool, 10, 256, 0, NULL);

/* 전역 채널 객체 (LE 전용 구조체 사용) */
static struct bt_l2cap_le_chan my_le_chan;

static struct bt_conn *current_conn = NULL; // 현재 연결된 기기 추적

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};
#define BT_LE_ADV_CONN BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONN, \
	BT_GAP_ADV_FAST_INT_MIN_2, \
	BT_GAP_ADV_FAST_INT_MAX_2, NULL)
/* Advertising 재시작 헬퍼 함수 */
static void start_advertising(void)
{
    /* BT_LE_ADV_CONN: 타임아웃 없이 무한히 광고 */
    int err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
    } else {
        LOG_INF("Advertising started...");
    }
}

/* --- [2] Connection Callback (재연결 및 속도 제어 핵심) --- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        start_advertising(); // 실패 시 다시 광고
        return;
    }

    LOG_INF("Bluetooth Central Connected!");
    current_conn = bt_conn_ref(conn);

    /* [Critical] 속도 향상 요청 (15ms 대응)
     * Min Interval: 7.5ms (6 * 1.25)
     * Max Interval: 15ms  (12 * 1.25)
     * Latency: 0
     * Timeout: 400ms
     */
    struct bt_le_conn_param param = {
        .interval_min = 6, 
        .interval_max = 12, 
        .latency = 0,
        .timeout = 400,
    };
    
    int ret = bt_conn_le_param_update(conn, &param);
    if (ret) {
        LOG_WRN("Connection param update request failed: %d", ret);
    } else {
        LOG_INF("Requested Connection Interval Update (7.5ms ~ 15ms)");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    /* [Critical] 연결이 끊어지면 즉시 다시 광고 시작 */
    start_advertising();
}

/* Connection Callbacks 등록 */
BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* * [Callback 1] Data Received 
 * 파라미터로 넘어오는 'chan'은 Base 구조체이므로 rx/tx 멤버가 없음.
 */
static int l2cap_recv(struct bt_l2cap_chan *chan, struct net_buf *buf)
{
    // 필요하다면 여기서 CONTAINER_OF를 써서 LE 채널 정보에 접근
    // struct bt_l2cap_le_chan *le_chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
    
    /* 데이터 수신 확인 */
	// memcpy(my_buffer, buf->data, buf->len);
    LOG_INF("Rx Data: len %u", buf->len);

    /* * 주의: 실제 데이터 처리는 여기서 memcpy 등을 수행.
     * return 0을 하면 Zephyr 스택이 버퍼 소유권을 가져가서 해제함.
     */
    return 0;
}

/* [Callback 2] Connected */
static void l2cap_connected(struct bt_l2cap_chan *chan)
{
    /* Base 포인터를 LE 구조체로 변환하여 MTU 정보 등을 확인 */
    struct bt_l2cap_le_chan *le_chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);

    LOG_INF("L2CAP Connected!");
    LOG_INF(" - RX MTU: %u", le_chan->rx.mtu);
    LOG_INF(" - TX MTU: %u", le_chan->tx.mtu);
}

/* [Callback 3] Disconnected */
static void l2cap_disconnected(struct bt_l2cap_chan *chan)
{
    LOG_INF("L2CAP Disconnected");
}

/* [Callback 4] Alloc Buffer */
static struct net_buf *l2cap_alloc_buf(struct bt_l2cap_chan *chan)
{
    /* L2CAP 전용 풀에서 할당 */
    return net_buf_alloc(&l2cap_rx_pool, K_NO_WAIT);
}

/* Operations VTable */
static const struct bt_l2cap_chan_ops l2cap_ops = {
    .alloc_buf    = l2cap_alloc_buf,
    .recv         = l2cap_recv,
    .connected    = l2cap_connected,
    .disconnected = l2cap_disconnected,
};

/* * [Callback 5] Connection Request Acceptor
 * 여기서 LE 구조체의 'rx' 멤버를 초기화해야 함.
 */
static int l2cap_accept(struct bt_conn *conn, struct bt_l2cap_chan **chan)
{
    LOG_INF("Incoming L2CAP Connection Request");

    /* 1. 구조체 초기화 (재연결 시 잔여 데이터 제거) */
    memset(&my_le_chan, 0, sizeof(my_le_chan));

    /* 2. Base 멤버 설정 */
    my_le_chan.chan.ops = &l2cap_ops;

    /* 3. LE Specific 멤버 설정 (여기가 핵심) */
    /* Base인 'chan'에는 rx가 없지만, 'bt_l2cap_le_chan'에는 rx가 있음 */
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


	LOG_INF("L2CAP Receiver Start");

    /* Bluetooth Init */
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return 0;
    }

    /* Register Server */
    err = bt_l2cap_server_register(&server);
    if (err) {
        LOG_ERR("Server register failed (err %d)", err);
        return 0;
    }
    LOG_INF("L2CAP Server registered (PSM 0x%04x)", L2CAP_COC_PSM);

	start_advertising();
	while(true){
		LOG_INF("Receiver is Running...");
		k_sleep(K_MSEC(1000));
	}


    return 0;
}