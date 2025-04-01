#ifndef LEDS_H
#define LEDS_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

class GPIO 
{
public:
    enum class Name {LED0, LED1, LED2, WATCHDOG};
	GPIO();

	bool Get(Name name);
    void Toggle(Name name);
    void Set(Name name, bool value);

private:
	struct gpio_dt_spec led1_spec;
	struct gpio_dt_spec led2_spec;
	struct gpio_dt_spec led0_spec;
	struct gpio_dt_spec watchdogreset_spec;
};

#endif // LEDS_H
