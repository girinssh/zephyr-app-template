#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(l2cap_sender, LOG_LEVEL_INF);

/* ---------------- Configuration ---------------- */
#define PEER_PSM         0x0080  // [중요] 수신측(Server)과 반드시 동일해야 함
#define DATA_SIZE        218     // 전송할 데이터 크기
#define TX_INTERVAL_MS   500      // 전송 주기 (최대한 빠르게)
#define TARGET_DEVICE_NAME "Zephyr_L2CAP_Rx_L"
#define TARGET_NAME_LEN    (sizeof(TARGET_DEVICE_NAME) - 1)

/* ---------------- Globals ---------------- */
static struct bt_conn *default_conn;
static struct bt_l2cap_le_chan l2cap_chan; // L2CAP 채널 객체
static uint8_t data_buffer[DATA_SIZE];     // 더미 데이터 버퍼

/* ---------------- L2CAP Callbacks ---------------- */
static void l2cap_chan_connected(struct bt_l2cap_chan *chan)
{
    LOG_INF("L2CAP Channel Connected!");
}

static void l2cap_chan_disconnected(struct bt_l2cap_chan *chan)
{
    LOG_WRN("L2CAP Channel Disconnected");
    // 필요 시 재연결 로직 추가 가능
}

static void l2cap_chan_status(struct bt_l2cap_chan *chan, atomic_t *status)
{
    LOG_INF("L2CAP Channel Status Changed: %lu", *status);
}

// 송신용 메모리 풀 정의 (데이터 패킷용)
NET_BUF_POOL_DEFINE(tx_pool, 16, DATA_SIZE, 4, NULL);

static const struct bt_l2cap_chan_ops l2cap_ops = {
    .connected = l2cap_chan_connected,
    .disconnected = l2cap_chan_disconnected,
    .status = l2cap_chan_status,
    // 수신은 하지 않으므로 alloc_buf/recv는 NULL 또는 기본값 처리
};

static bool found_target = false;

/* AD(Advertising Data)를 파싱하기 위한 콜백 함수 */
uint8_t dev_name[100] = {0};
static bool eir_found(struct bt_data *data, void *user_data)
{
    // 데이터 타입이 "Complete Local Name" 또는 "Shortened Local Name" 인지 확인
	bool result = true;

    if (data->type == BT_DATA_NAME_COMPLETE || data->type == BT_DATA_NAME_SHORTENED) {
		if(data->data_len < 99){
			memcpy(dev_name, data->data, data->data_len); 
			LOG_INF("Target Name %s", dev_name);	
		}
		// 길이와 내용이 일치하는지 확인
        if (data->data_len == TARGET_NAME_LEN &&
            memcmp(data->data, TARGET_DEVICE_NAME, TARGET_NAME_LEN) == 0) {
            
            found_target = true; // 찾았음 표시
            result = false; // 파싱 중단 (더 볼 필요 없음)
        }
    }
    return result; // 계속 파싱
}

/* ---------------- Scanning Logic ---------------- */
static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                         struct net_buf_simple *ad)
{
    char addr_str[BT_ADDR_LE_STR_LEN];
    int err;
    
    if (default_conn) {
		LOG_INF("device_found: Already Connected");
		return; // 이미 연결 중이면 무시
	}
    /* 1. 필터링 초기화 */
    found_target = false;

    /* 2. 광고 데이터 파싱 시작 -> eir_found 함수가 호출됨 */
    bt_data_parse(ad, eir_found, NULL);

    /* 3. 타겟을 찾았을 때만 연결 시도 */
    if (found_target) {
        bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
        LOG_INF("device_found: Target Found: %s (RSSI %d). Connecting...", addr_str, rssi);

        err = bt_le_scan_stop();
        if (err) {
			LOG_INF("device_found: Failed to Stop Scanning");
			return;
		}
        err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, 
                                BT_LE_CONN_PARAM_DEFAULT, &default_conn);
        if (err) {
            LOG_ERR("device_found: Create conn failed (err %d)", err);
            bt_le_scan_start(BT_LE_SCAN_PASSIVE, NULL);
        } else {
			LOG_INF("device_found: Success Creating Conn");
		}
    }
}

/* ---------------- Connection Callbacks ---------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        return;
    }

    LOG_INF("ACL Connected");
    default_conn = bt_conn_ref(conn);

    // 1. Connection Parameter Update (속도 향상)
    struct bt_le_conn_param param = BT_LE_CONN_PARAM_INIT(6, 6, 0, 400); // 7.5ms interval
    bt_conn_le_param_update(conn, &param);

    // 2. PHY Update (2Mbps)
    const struct bt_conn_le_phy_param phy_param = {
        .options = BT_CONN_LE_PHY_OPT_NONE,
        .pref_tx_phy = BT_GAP_LE_PHY_2M,
        .pref_rx_phy = BT_GAP_LE_PHY_2M,
    };
    bt_conn_le_phy_update(conn, &phy_param);

    // 3. Initiate L2CAP CoC Connection
    l2cap_chan.chan.ops = &l2cap_ops;
    l2cap_chan.rx.mtu = 23; // 수신은 안 하므로 작게 설정 가능

    int ret = bt_l2cap_chan_connect(conn, &l2cap_chan.chan, PEER_PSM);
    if (ret < 0) {
        LOG_ERR("L2CAP Connect failed (err %d)", ret);
    } else {
        LOG_INF("L2CAP Connect req sent");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected (reason 0x%02x)", reason);
    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
    // 연결 끊김 시 다시 스캔 시작하도록 할 수 있음
}

static void recycled_cb(void){
	int err;
    LOG_INF("Bluetooth recycled. Scanning...");
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err) {
        LOG_ERR("Scanning failed (err %d)", err);
        return;
    }
}

struct bt_conn_cb conn_callbacks = {
	.recycled = recycled_cb,
    .connected = connected,
    .disconnected = disconnected,
};

/* ---------------- Main Logic ---------------- */
int main(void)
{
    int err;


    LOG_INF("Starting L2CAP CoC Sender on XIAO BLE");

    // Dummy Data Init
    for(int i=0; i<DATA_SIZE; i++) data_buffer[i] = (uint8_t)i;
    
	err = bt_conn_cb_register(&conn_callbacks);
    if (err) {
        LOG_ERR("Connection callback register failed (err %d)", err);
        return 0;
    }
	k_sleep(K_MSEC(100));
    err = bt_enable(NULL);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    LOG_INF("Bluetooth initialized. Scanning...");
    err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
    if (err) {
        LOG_ERR("Scanning failed (err %d)", err);
        return 0;
    }

    struct net_buf *buf;

    while (1) {
        // L2CAP 채널이 연결된 상태인지 확인
        // atomic_get(&l2cap_chan.chan.status) 등을 더 정교하게 체크할 수 있습니다.
        if (default_conn && *(l2cap_chan.chan.status) == BT_L2CAP_CONNECTED) {
            LOG_INF("CHECK POINT");
            // 1. Allocate Buffer
            // 헤드룸 예약이 필수입니다 (L2CAP 헤더 공간)
            buf = net_buf_alloc(&tx_pool, K_NO_WAIT);
            if (!buf) {
                LOG_WRN("Tx pool empty");
                k_sleep(K_MSEC(1)); 
                continue;
            }

            // 2. L2CAP 헤더 공간 확보
            net_buf_reserve(buf, BT_L2CAP_SDU_BUF_SIZE(0));

            // 3. 데이터 복사
            net_buf_add_mem(buf, data_buffer, DATA_SIZE);

            // 4. 전송 (비동기)
            // L2CAP CoC는 Credit이 없으면 -EAGAIN을 반환하거나 대기합니다.
            int ret = bt_l2cap_chan_send(&l2cap_chan.chan, buf);
            if (ret < 0) {
                // 전송 실패 (주로 Credit 부족) 시 버퍼 해제 필요
                LOG_DBG("Send fail/busy: %d", ret);
                net_buf_unref(buf); 
            } else {
                // 전송 성공 시 net_buf는 스택이 알아서 해제함
                LOG_DBG("Sent %d bytes", DATA_SIZE);
            }
        }
		
	    LOG_INF("L2CAP CoC Sender is running... ");

        k_sleep(K_MSEC(TX_INTERVAL_MS));
    }
    return 0;
}