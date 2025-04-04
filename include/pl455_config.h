#ifndef CONFIGH
#define CONFIGH


#define NUM_CELLS 16
#define ADDR_SIZE 0 //0 is 8 bit register addresses (TI recommended), 1 is 16 bit (used by BMW)
#define MAX_MODULES 2 //maximum of 16 modules in a pack
#define COMM_TIMEOUT 1000 //ms, sets an error flag if we don't recieve a response to a request in this time

#define CELL_IGNORE_VOLT 5000 //ADC readings below this number will result in the cell being ignored for min and average etc calcuations. 5000 is 381mV, which should be plenty high enough to ignore disconnected cells
#define BALANCE_TOLERANCE 26 //Balance will not be enabled for cells <2mV away from the min cell voltage. 26 is 2mV
#define BALANCE_DUTYCYCLE 90 //Percent, approximate!
#define BALANCE_MIN_VOLT 39321 //ADC readings below this number will preclude balancing. 39321 is 3.0V
#define BALANCE_WHILE_CHARGE 1 //if 1, the BMS will always balance while charging (current < 0), even if voltages are low

#define BMS_CYCLE_PERIOD 500000 //microseconds, you get a voltage reading approximately this often (actually, the time between voltages will always be greater due to sampling times and delays)

#define REPORTING_PERIOD 1000 //milliseconds - you get an status output this frequently.
#define VOLTS_DECIMALS 3 //decimal points used for voltage reporting
#define TEMP_DECIMALS 1

#define UPS_VOLT_READ_PERIOD 500 //ms

#endif //CONFIGH