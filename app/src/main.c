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

/* Advertising Data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
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

    /* Start Advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) {
        LOG_ERR("Advertising failed (err %d)", err);
        return 0;
    }
    LOG_INF("Advertising started. Waiting for connection...");

	while(true){
		LOG_INF("Receiver is Running...");
		k_sleep(K_MSEC(1000));
	}


    return 0;
}