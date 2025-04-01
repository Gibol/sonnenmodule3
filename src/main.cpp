#include <zephyr/kernel.h>

#include "gpio.h"
#include "can.h"
#include "slave.h"
#include "master.h"


#define MODULE_ID 0 // Master has id 0

GPIO gpio;

#if MODULE_ID == 0
	K_MSGQ_DEFINE(master_queue, sizeof(Request), 4, 1);
	MasterBMS  master(gpio);
	ModuleData moduleDatas[NUM_MODULES];
#else
	ModuleData moduleDatas[1];
#endif

void messageReceived(uint32_t id, bool rtr, uint8_t *data, uint8_t dataLen)
{
	#if MODULE_ID == 0
	// printk("Received message %x, rtr: %d, dataLen: %d\n", id, rtr, dataLen);

	if(id == 0x4200 && dataLen == 8) // Host request
	{

		constexpr Request e = Request::EnsembleInformation;
		constexpr Request s = Request::SystemEqipmentInformation;
		auto req = (Message::HostRequest*) data;
		k_msgq_put(&master_queue, (req->request == s ? &s : &e), K_NO_WAIT);

	}
	else
	{
		auto moduleId = (id & IdMask) / ModuleOffset;
		if(moduleId >= NUM_MODULES)
		{
			printk("Error: module out of range: %d\n", moduleId);
			return;
		}
		moduleDatas[moduleId].SetRawData(id, data);
		if(moduleDatas[moduleId].isComplete())
		{
			master.updateModuleData(moduleId, moduleDatas[moduleId]);
		}
	}
	#endif
}

void feed(GPIO& gpio)
{
	gpio.Set(GPIO::Name::WATCHDOG, true);
	k_msleep(1);
	gpio.Set(GPIO::Name::WATCHDOG, false);
}

int main(void)
{
	
	CAN_Initialize(messageReceived);
	Slave slave(moduleDatas[0], MODULE_ID, gpio);

	while(1)
	{
		feed(gpio);
		auto update = slave.worker();
		#if MODULE_ID == 0
		if(update)
		{
			master.updateModuleData(MODULE_ID, moduleDatas[0]);
		}
		master.worker(master_queue);
		#endif
	}
	return 0;
}
