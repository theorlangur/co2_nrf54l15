/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <cmath>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/settings/settings.h>
#include <zephyr/drivers/adc.h>

#include <ram_pwrdn.h>

//#define ALARM_LIST_LOCK_TYPE thread::DummyLock
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>

#include <nrfzbcpp/zb_power_config_tools.hpp>
#include <nrfzbcpp/zb_poll_ctrl_tools.hpp>

#include <nrfzbcpp/zb_status_cluster_desc.hpp>

#include <nrfzbcpp/zb_alarm.hpp>
#include <nrfzbcpp/zb_settings.hpp>

#include <dk_buttons_and_leds.h>
#include "led.h"

//allows using _min_to_qs and similar stuff
using namespace zb::literals;

/**********************************************************************/
/* Configuration constants                                            */
/**********************************************************************/
constexpr bool kPowerSaving = true;//if this is change the MCU must be erased since that option persists otherwise
constexpr uint32_t kFactoryResetWaitMS = 5000;//5s if the dev doesn't join before that
constexpr int8_t kRestartCountToFactoryReset = 3;
constexpr uint32_t kRestartCounterResetTimeoutMS = 15000;//after 15s the restart counter is reset back to 3
constexpr uint32_t kKeepAliveTimeout = 1000*60*30;//30min

constexpr auto kInitialCheckInInterval = 30_min_to_qs;
constexpr auto kInitialLongPollInterval = 60_min_to_qs;//this has to be big in order for the device not to perform permanent parent requests

/**********************************************************************/
/* Zigbee Declarations and Definitions                                */
/**********************************************************************/
static bool g_ZigbeeReady = false;

/* Manufacturer name (32 bytes). */
#define INIT_BASIC_MANUF_NAME      "SFINAE"

/* Model number assigned by manufacturer (32-bytes long string). */
#define INIT_BASIC_MODEL_ID        "CO2-NG"


/* Button used to enter the Bulb into the Identify mode. */
#define IDENTIFY_MODE_BUTTON            DK_BTN1_MSK

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

/* Device endpoint, used to receive light controlling commands. */
constexpr uint8_t kCO2_EP = 1;
constexpr uint16_t kDEV_ID = 0xFEED;

/* Main application customizable context.
 * Stores all settings and static values.
 */
struct device_ctx_t{
    zb::zb_zcl_basic_names_t basic_attr;
    zb::zb_zcl_power_cfg_battery_info_t battery_attr;
    zb::zb_zcl_poll_ctrl_basic_t poll_ctrl;
    zb::zb_zcl_status_t status_attr;
    //add co2
};

//attribute shortcuts for template arguments
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrStatus2 = &zb::zb_zcl_status_t::status2;
constexpr auto kAttrStatus3 = &zb::zb_zcl_status_t::status3;

constexpr int kSleepSendStatusBit1 = 0;
constexpr int kSleepSendStatusCodeOffset = 1;
constexpr int kFlipSendStatusBit1 = 5;
constexpr int kFlipSendStatusCodeOffset = 6;
constexpr int kWakeUpSendStatusBit1 = 10;
constexpr int kWakeUpSendStatusCodeOffset = 11;

constexpr int kStatusCodeSize = 4;

/* Zigbee device application context storage. */
static constinit device_ctx_t dev_ctx{
    .basic_attr = {
	{
	    .zcl_version = ZB_ZCL_VERSION,
	    .power_source = zb::zb_zcl_basic_min_t::PowerSource::Battery
	},
	/*.manufacturer =*/ INIT_BASIC_MANUF_NAME,
	/*.model =*/ INIT_BASIC_MODEL_ID,
    },
	.poll_ctrl = {
	    .check_in_interval = kInitialCheckInInterval,
	    .long_poll_interval = kInitialLongPollInterval,
	    //.short_poll_interval = 1_sec_to_qs,
	},
};

//forward declare
//template<> struct zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>;
//using custom_accel_handler_t = zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>;

constinit static auto zb_ctx = zb::make_device(
	zb::make_ep_args<{.ep=kCO2_EP, .dev_id=kDEV_ID, .dev_ver=1}>(
	    dev_ctx.basic_attr
	    , dev_ctx.battery_attr
	    , dev_ctx.poll_ctrl
	    , dev_ctx.status_attr
	    )
	);

//a shortcut for a convenient access
constinit static auto &zb_ep = zb_ctx.ep<kCO2_EP>();

//magic handwaving to avoid otherwise necessary command handling boilerplate
//uses CRTP so that cluster_custom_handler_base_t would know the end type it needs to work with
//template<> 
//struct zb::cluster_custom_handler_t<device_ctx_t::accel_type, kACCEL_EP>: cluster_custom_handler_base_t<custom_accel_handler_t>
//{
//    //the rest will be done by cluster_custom_handler_base_t
//    static auto& get_device() { return zb_ctx; }
//};


/**********************************************************************/
/* Persisten settings                                                 */
/**********************************************************************/

#define SETTINGS_ZB_CO2_SUBTREE "zb_co2"
struct ZbSettingsEntries
{
    //had to define the strings like that or otherwise passing 'const char*' as a template parameter at a compile time doesn't work
    //it needs to have an extrenal linking
    inline static constexpr const char wake_sleep_threshold[] = SETTINGS_ZB_CO2_SUBTREE "/wake_sleep_threshold";
    inline static constexpr const char sleep_duration[] = SETTINGS_ZB_CO2_SUBTREE "/sleep_duration";
    inline static constexpr const char sleep_tracking_rate[] = SETTINGS_ZB_CO2_SUBTREE "/sleep_tracking_rate";
    inline static constexpr const char active_tracking_rate[] = SETTINGS_ZB_CO2_SUBTREE "/active_tracking_rate";
};

using settings_mgr = zb::persistent_settings_manager<
    sizeof(SETTINGS_ZB_CO2_SUBTREE)
>;

//helping constexpr template functions to wrap the change reaction logic into settings-storing logic
template<auto h>
constexpr zb::set_attr_value_handler_t to_settings_handler(const char *name)
{
    return settings_mgr::make_on_changed<zb::to_handler_v<h>>(name);
}

constexpr zb::set_attr_value_handler_t to_settings_handler(const char *name)
{
    return settings_mgr::make_on_changed<nullptr>(name);
}

/**********************************************************************/
/* End of settings section                                            */
/**********************************************************************/

/**********************************************************************/
/* ZephyrOS devices                                                   */
/**********************************************************************/

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct device *const co2_dev = DEVICE_DT_GET(DT_NODELABEL(co2sensor));
static const struct gpio_dt_spec led_dt = GPIO_DT_SPEC_GET(LED0_NODE, gpios);


/**********************************************************************/
/* Battery management                                                 */
/**********************************************************************/
#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

static constinit auto g_Battery = zb::make_battery_measurements_block<{
    .maxBatteryVoltage = 1600,//mV
    .minBatteryVoltage = 900//mV
}>(zb_ep, adc_channels);
constexpr zb_callback_t update_battery_state_zb = &decltype(g_Battery)::update_zb_callback;


/**********************************************************************/
/* End of battery management                                          */
/**********************************************************************/

void reconfigure_interrupts()
{
    //return;
}

void on_settings_changed(const uint32_t &v)
{
    //printk("Settings. Now: %X; Stored: %X\r\n", v, dev_ctx.settings.flags_dw);
    reconfigure_interrupts();
}

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;

    configure_poll_control<{
	.ep = kCO2_EP, 
	.callback_on_check_in = update_battery_state_zb, 
	.sleepy_end_device = kPowerSaving
    }>(dev_ctx.poll_ctrl);

    //should be there already, initial state
    //udpate_accel_values(0);
    g_Battery.update();
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
        zb_zdo_app_signal_hdr_t *pHdr;
        auto signalId = zb_get_app_signal(bufid, &pHdr);

	auto ret = zb::tpl_signal_handler<zb::sig_handlers_t{
	.on_leave = +[]{ 
	    zb_zcl_poll_control_stop(); 
	    k_sleep(K_MSEC(2100));
	    sys_reboot(SYS_REBOOT_COLD);
	},
	    .on_error = []{ led::show_pattern(led::kPATTERN_3_BLIPS_NORMED, 1000); },
	    .on_dev_reboot = on_zigbee_start,
	    .on_steering = on_zigbee_start,
	    .on_can_sleep = &zb_sleep_now
	   }>(bufid);
    const uint32_t LOCAL_ERR_CODE = (uint32_t) (-ret);	
    if (LOCAL_ERR_CODE != RET_OK) {				
	zb_osif_abort();				
    }							
}

void on_dev_cb_error(int err)
{
    printk("on_dev_cb_error: %d\r\n", err);
}

//settings_handler settings_zb_co2 = { 
//			      .name = SETTINGS_ZB_CO2_SUBTREE,
//                              .h_set = settings_mgr::zigbee_settings_set,
//                              .h_export = settings_mgr::zigbee_settings_export
//};

/**********************************************************************/
/* Factory Reset Handling                                             */
/**********************************************************************/
zb::ZbTimerExt g_FactoryResetDoneChecker;
/**@brief Callback for button events.
 *
 * @param[in]   button_state  Bitmask containing the state of the buttons.
 * @param[in]   has_changed   Bitmask containing buttons that have changed their state.
 */
static void button_changed(uint32_t button_state, uint32_t has_changed)
{
    if (FACTORY_RESET_BUTTON & has_changed) {
	if (FACTORY_RESET_BUTTON & button_state) {
	    /* Button changed its state to pressed */
	    g_FactoryResetDoneChecker.Setup([]{
		    if (was_factory_reset_done()) {
			/* The long press was for Factory Reset */
			led::show_pattern(led::kPATTERN_2_BLIPS_NORMED, 2000);
			return false;
		    }
		    return true;
	    }, 1000);
	} else {
	    /* Button changed its state to released */
	    if (!was_factory_reset_done()) {
		/* Button released before Factory Reset */
		g_FactoryResetDoneChecker.Cancel();
		led::show_pattern(led::kPATTERN_2_BLIPS_NORMED, 500);
	    }
	}
	check_factory_reset_button(button_state, has_changed);
    }
}

/**********************************************************************/
/* End Of Factory Reset Handling                                      */
/**********************************************************************/

int main(void)
{
    int ret;
    bool led_state = true;

    printk("Main start\r\n");
    if (led::setup() < 0)
	return 0;
    led::start();

    if (device_is_ready(co2_dev))
    {
	//if (dev_ctx.settings.active_odr == 0)
	//    dev_ctx.settings.active_odr = lis2du12_get_odr(accel_dev);
	//else
	//{
	//    if (lis2du12_set_odr(accel_dev, (lis2du12_odr_t)dev_ctx.settings.active_odr) < 0)
	//	dev_ctx.status_attr.status2 = -1;
	//}
	reconfigure_interrupts();
    }else
    {
	printk("Accelerometer is not ready\r\n");
	dev_ctx.status_attr.status1 = -1;
    }

    printk("Main: before configuring ADC\r\n");
    if (g_Battery.setup() < 0)
	return 0;

    printk("Main: before settings init\r\n");
    int err = settings_subsys_init();
    //settings_register(&settings_zb_co2);
    //settings_register(&settings_dev);

    err = settings_load();
    printk("Main: before zigbee erase persistent storage\r\n");

    //configure button handler
    err = dk_buttons_init(button_changed);
    //assign a button for a factory reset procedure
    register_factory_reset_button(FACTORY_RESET_BUTTON);

    zigbee_erase_persistent_storage(false);
    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(kKeepAliveTimeout));

    if constexpr (kPowerSaving)
    {
	zigbee_configure_sleepy_behavior(true);
	power_down_unused_ram();
    }

    /* Register callback for handling ZCL commands. */
    auto dev_cb = zb::tpl_device_cb<
	zb::dev_cb_handlers_desc{ .error_handler = on_dev_cb_error }
	//handler
	//   , zb::set_attr_val_gen_desc_t{
	//{
	//    .ep = kACCEL_EP,
	//    .cluster = zb::kZB_ZCL_CLUSTER_ID_ACCEL_SETTINGS,
	//    .attribute = zb::kZB_ATTR_ID_WAKE_SLEEP_THRESHOLD
	//},
	//to_settings_handler<on_wake_sleep_settings_changed>(ZbSettingsEntries::wake_sleep_threshold)
	//     }
    >;
    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    printk("Main: before zigbee enable\r\n");
    zigbee_enable();
    printk("Main: sleep forever\r\n");
    while (1) {
	k_sleep(K_FOREVER);
    }
    return 0;
}
