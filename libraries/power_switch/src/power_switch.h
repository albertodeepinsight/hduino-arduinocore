/*
 * power_switch.h
 *
 * Created: 01/02/2018 12:06:39
 *  Author: arm
 */ 


#ifndef POWER_SWITCH_H_
#define POWER_SWITCH_H_

enum system_battery_power_switch {
	/** The backup domain is always supplied by main power */
	SYSTEM_BATTERY_POWER_SWITCH_NONE      = SUPC_BBPS_CONF_NONE_Val,
	/** The power switch is handled by the automatic power switch */
	SYSTEM_BATTERY_POWER_SWITCH_AUTOMATIC = SUPC_BBPS_CONF_APWS_Val,
	/** The backup domain is always supplied by battery backup power */
	SYSTEM_BATTERY_POWER_SWITCH_FORCED    = SUPC_BBPS_CONF_FORCED_Val,
	/** The power switch is handled by the BOD33 */
	SYSTEM_BATTERY_POWER_SWITCH_BOD33     = SUPC_BBPS_CONF_BOD33_Val,
};

struct system_battery_backup_power_switch_config {
	/** Enable device wake up when BBPS switches from
		battery backup power to main power */
	bool wake_enabled;
	/** Battery backup power switch configuration */
	enum system_battery_power_switch battery_power_switch;
};

static inline void system_battery_backup_power_switch_set_config(
struct system_battery_backup_power_switch_config *const config)
{
	//Assert(config);
	uint32_t new_config = SUPC->BBPS.reg & SUPC_BBPS_PSOKEN;

	if(config->wake_enabled) {
		new_config |= SUPC_BBPS_WAKEEN;
	}

	new_config |= SUPC_BBPS_CONF(config->battery_power_switch);

	SUPC->BBPS.reg = new_config;

	if (config->battery_power_switch == SYSTEM_BATTERY_POWER_SWITCH_AUTOMATIC) {
		while (!(SUPC->STATUS.reg & SUPC_STATUS_APWSRDY)) {
			;
		}
	}
}




#endif /* POWER_SWITCH_H_ */