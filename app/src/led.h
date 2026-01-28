#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h> 

#define STATE_LED_OFF {LED_OFF, LED_OFF, LED_OFF}           // OFF

#define STATE_RED_ON {LED_ON, LED_OFF, LED_OFF}          // TBD
#define STATE_GREEN_ON {LED_OFF, LED_ON, LED_OFF}        // ADVERTIZE
#define STATE_BLUE_ON {LED_OFF, LED_OFF, LED_ON}         // L2CAP
#define STATE_YELLOW_ON {LED_ON, LED_ON, LED_OFF}        // UART
#define STATE_CYAN_ON {LED_OFF, LED_ON, LED_ON}      
#define STATE_MAGENTA_ON {LED_ON, LED_OFF, LED_ON}
#define STATE_WHITE_ON {LED_ON, LED_ON, LED_ON}

#define STATE_RED_BLINK {BLINK1, LED_OFF, LED_OFF}          // TBD
#define STATE_GREEN_BLINK {LED_OFF, BLINK1, LED_OFF}        // ADVERTIZE
#define STATE_BLUE_BLINK {LED_OFF, LED_OFF, BLINK1}         // L2CAP
#define STATE_YELLOW_BLINK {BLINK1, BLINK1, LED_OFF}        // UART
#define STATE_CYAN_BLINK {LED_OFF, BLINK1, BLINK1}      
#define STATE_MAGENTA_BLINK {BLINK1, LED_OFF, BLINK1}
#define STATE_WHITE_BLINK {BLINK1, BLINK1, BLINK1}

#define STATE_RED_GREEN_BLINK {BLINK1, BLINK2, LED_OFF}     // ADVERTIZE
#define STATE_RED_BLUE_BLINK {BLINK1, LED_OFF, BLINK2}      // L2CAP
#define STATE_RED_YELLOW_BLINK {LED_ON, BLINK1, LED_OFF}    // UART
#define STATE_RED_CYAN_BLINK {BLINK1, BLINK2, BLINK2}       
#define STATE_RED_MAGENTA_BLINK {LED_ON, LED_OFF, BLINK1}
#define STATE_RED_WHITE_BLINK {LED_ON, BLINK1, BLINK1}

#define STATE_GREEN_BLUE_BLINK {LED_OFF, BLINK1, BLINK2}
#define STATE_GREEN_CYAN_BLINK  {LED_OFF, LED_ON, BLINK1}
#define STATE_GREEN_YELLOW_BLINK {BLINK1, LED_ON, LED_OFF}
#define STATE_GREEN_MAGENTA_BLINK {BLINK1, BLINK2, BLINK1}
#define STATE_GREEN_WHITE_BLINK {BLINK1, LED_ON, BLINK1}

#define STATE_BLUE_YELLOW_BLINK {BLINK1, BLINK1, LED_ON}
#define STATE_BLUE_CYAN_BLINK {LED_OFF, BLINK1, LED_ON}
#define STATE_BLUE_WHITE_BLINK {BLINK1, BLINK1, LED_ON}

#define STATE_YELLOW_MAGENTA_BLINK {LED_ON, BLINK1, BLINK2}

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
    LED_OFF,
    LED_ON,
    BLINK1,
    BLINK2,
};

static K_SEM_DEFINE(led_state_sem, 1, 1);

static enum BLINK_STATE blink_state[3] = {
    LED_OFF, 
    LED_OFF,
    LED_OFF
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

    if(state == LED_OFF || state == BLINK1){
        gpio_pin_set_dt(&led_devs[(int)e_color], 0);
    } else if(state == LED_ON || state == BLINK2){
        gpio_pin_set_dt(&led_devs[(int)e_color], 1);
    } 
    k_sem_give(&led_state_sem);
}

static void blink_led(){
    int i = 0;
    while(1){
        k_sem_take(&led_state_sem, K_FOREVER);
        for (i = 0; i < 3; i++){    
            if(blink_state[i] == BLINK1){
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
            blink_state[i] = LED_OFF;
        }
        else {
            err = gpio_pin_configure_dt(&led_devs[i], GPIO_OUTPUT_ACTIVE);
            if(err){
                // LOG_INF("CONFIG_LED_FAIELD with %d", err);
                return;
            }
            set_led_state(i, LED_ON);
        }
    }
    k_thread_start(led_blink_thread);
    // LOG_INF("LED CONFIG END");
}