#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h> 

#define LED_RED_NODE DT_ALIAS(r_led)
#define LED_GREEN_NODE DT_ALIAS(g_led)
#define LED_BLUE_NODE DT_ALIAS(b_led)

static const struct gpio_dt_spec led_devs[3] = {
    GPIO_DT_SPEC_GET(LED_RED_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_GREEN_NODE, gpios),
    GPIO_DT_SPEC_GET(LED_BLUE_NODE, gpios)
};

enum LED_COLOR {
    RED,
    GREEN,
    BLUE
};

enum BLINK_STATE {
    BLINK_STOP_LED_OFF,
    BLINK_STOP_LED_ON,
    BLINK_START,
};

static K_SEM_DEFINE(led_state_sem, 1, 1);

static enum BLINK_STATE blink_state[3] = {
    BLINK_STOP_LED_OFF, 
    BLINK_STOP_LED_OFF,
    BLINK_STOP_LED_OFF
};

static int interval = 500;

static void toggle_led(enum LED_COLOR e_color){
    int err;
    err = gpio_pin_toggle_dt(&led_devs[e_color]);
}

// 여기에 외부에서 LED 세팅을 하기위한 함수 필요. 

void set_blink_interval(int new_interval){
    interval = new_interval;
    // LOG_INF("SET LED BLINK INTERVAL %d", interval);
}

void set_led_state(enum LED_COLOR e_color, enum BLINK_STATE state){
    k_sem_take(&led_state_sem, K_FOREVER);
    blink_state[e_color] = state;

    if(state == BLINK_STOP_LED_OFF){
        gpio_pin_set_dt(&led_devs[(int)e_color], 0);
    } else if(state == BLINK_STOP_LED_ON){
        gpio_pin_set_dt(&led_devs[(int)e_color], 1);
    }

    k_sem_give(&led_state_sem);
}

static void blink_led(){
    int i = 0;
    while(1){
        k_sem_take(&led_state_sem, K_FOREVER);
        for (i = 0; i < 3; i++){    
            if(blink_state[i] == BLINK_START){
                toggle_led((enum BLINK_STATE)i);
            }
        }
        k_sem_give(&led_state_sem);
        k_msleep(interval);
    }
}

K_THREAD_DEFINE(led_blink_thread, 256, blink_led, NULL, NULL, NULL, 256, 0, 0);

// void blink_start(){
//     k_sem_take(&led_state_sem, K_MSEC(1));
//     blink_state[i] = BLINK_START;
//     k_sem_give(&led_state_sem);
// }

void configure_led(){
    int err;
    int i = 0;
    for (i = 0; i < 3; i++){
        if (!gpio_is_ready_dt(&led_devs[i])) {
            blink_state[i] = BLINK_STOP_LED_OFF;
        }
        else {
            err = gpio_pin_configure_dt(&led_devs[i], GPIO_OUTPUT_ACTIVE);
            if(err){
                // LOG_INF("CONFIG_LED_FAIELD with %d", err);
                return;
            }
        }
    }
    k_thread_start(led_blink_thread);
    // LOG_INF("LED CONFIG END");
}