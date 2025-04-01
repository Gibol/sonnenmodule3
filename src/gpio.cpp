#include "gpio.h"

#define LED0_NODE DT_ALIAS(led0)
#if !DT_NODE_HAS_STATUS_OKAY(LED0_NODE)
#error "BOARD does not define LED0"
#endif

#define LED1_NODE DT_ALIAS(led1)
#if !DT_NODE_HAS_STATUS_OKAY(LED1_NODE)
#error "BOARD does not define LED1"
#endif

#define LED2_NODE DT_ALIAS(led2)
#if !DT_NODE_HAS_STATUS_OKAY(LED2_NODE)
#error "BOARD does not define LED2"
#endif

#define WATCHDOG_RESET_NODE DT_ALIAS(watchdogreset)
#if !DT_NODE_HAS_STATUS_OKAY(WATCHDOG_RESET_NODE)
#error "BOARD does not define WATCHDOG_RESET"
#endif

GPIO::GPIO() 
{
	led0_spec = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
	led1_spec = GPIO_DT_SPEC_GET(LED1_NODE, gpios);
	led2_spec = GPIO_DT_SPEC_GET(LED2_NODE, gpios);
    watchdogreset_spec = GPIO_DT_SPEC_GET(WATCHDOG_RESET_NODE, gpios); 

	/* Verify that each GPIO device is ready */
	if (!device_is_ready(led0_spec.port) ||
	    !device_is_ready(led1_spec.port) ||
        !device_is_ready(led2_spec.port) ||
	    !device_is_ready(watchdogreset_spec.port) ) {
		printk("Error: One or more GPIO devices are not ready\n");
		return;
	}

	/* Configure LED pins as outputs.
	 * Using GPIO_OUTPUT_INACTIVE sets the initial state; adjust if needed.
	 */
	gpio_pin_configure_dt(&led0_spec, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led1_spec, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure_dt(&led2_spec, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&watchdogreset_spec, GPIO_OUTPUT_INACTIVE);
}

void GPIO::Set(Name name, bool state)
{
	switch (name) {
        case Name::LED0:
            gpio_pin_set_dt(&led0_spec, state);
            break;
        case Name::LED1:
            gpio_pin_set_dt(&led1_spec, state);
            break;
        case Name::LED2:
            gpio_pin_set_dt(&led2_spec, state);
            break;
        case Name::WATCHDOG:
            gpio_pin_set_dt(&watchdogreset_spec, state);
            break;
        default:
            printk("Invalid gpio: %d\n", (int) name);
            break;
        }
}

bool GPIO::Get(Name name)
{
	switch (name) {
        case Name::LED0:
            return gpio_pin_get_dt(&led0_spec);
        case Name::LED1:
            return gpio_pin_get_dt(&led1_spec);
        case Name::LED2:
            return gpio_pin_get_dt(&led2_spec);
        case Name::WATCHDOG:
            return gpio_pin_get_dt(&watchdogreset_spec);
            break;
        default:
            printk("Invalid LED number: %d\n", (int) name);
            return false;
        }
}

void GPIO::Toggle(Name name) 
{
    Set(name, !Get(name));
}
