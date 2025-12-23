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
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/sensor/scd4x.h>

#include <ram_pwrdn.h>

//#define ALARM_LIST_LOCK_TYPE thread::DummyLock
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>

#include <nrfzbcpp/zb_power_config_tools.hpp>
#include <nrfzbcpp/zb_poll_ctrl_tools.hpp>

#include <nrfzbcpp/zb_status_cluster_desc.hpp>
#include <nrfzbcpp/zb_co2_cluster_desc.hpp>
#include <nrfzbcpp/zb_temp_cluster_desc.hpp>
#include <nrfzbcpp/zb_humid_cluster_desc.hpp>

#include <nrfzbcpp/zb_alarm.hpp>
#include <nrfzbcpp/zb_settings.hpp>

#include "zb/zb_scd4x_cluster_desc.hpp"

#include <nrf_general/lib_msgq_typed.hpp>

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
constexpr zb_time_t kWakeUpDurationMS = 8 * 1000;

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

#define WAKE_UP_BUTTON            DK_BTN2_MSK

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
    zb::zb_zcl_co2_basic_t co2_attr;
    zb::zb_zcl_temp_basic_t temp_attr;
    zb::zb_zcl_rel_humid_basic_t humid_attr;
    zb::zb_zcl_scd4x_t scd4x;
};

//attribute shortcuts for template arguments
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrStatus2 = &zb::zb_zcl_status_t::status2;
constexpr auto kAttrStatus3 = &zb::zb_zcl_status_t::status3;
constexpr auto kAttrCO2Value = &zb::zb_zcl_co2_basic_t::measured_value;
constexpr auto kAttrTempValue = &zb::zb_zcl_temp_basic_t::measured_value;
constexpr auto kAttrRelHValue = &zb::zb_zcl_rel_humid_basic_t::measured_value;

constexpr auto kAttrManMeasure = &zb::zb_zcl_scd4x_t::manual_measurements;
constexpr auto kAttrFactoryResets = &zb::zb_zcl_scd4x_t::factory_resets;
constexpr auto kAttrSensorVariant = &zb::zb_zcl_scd4x_t::sensor_variant;

constexpr int kSleepSendStatusBit1 = 0;
constexpr int kSleepSendStatusCodeOffset = 1;
constexpr int kFlipSendStatusBit1 = 5;
constexpr int kFlipSendStatusCodeOffset = 6;
constexpr int kWakeUpSendStatusBit1 = 10;
constexpr int kWakeUpSendStatusCodeOffset = 11;

constexpr int kStatusCodeSize = 4;

zb::CmdHandlingResult on_scd4x_factory_reset();

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
    .co2_attr{
	.measured_value = 0
    },
    .scd4x{
	.cmd_on_factory_reset = {.cb = on_scd4x_factory_reset}
    }
};

constinit static auto zb_ctx = zb::make_device(
	zb::make_ep_args<{.ep=kCO2_EP, .dev_id=kDEV_ID, .dev_ver=1}>(
	    dev_ctx.basic_attr
	    , dev_ctx.battery_attr
	    , dev_ctx.poll_ctrl
	    , dev_ctx.status_attr
	    , dev_ctx.co2_attr
	    , dev_ctx.temp_attr
	    , dev_ctx.humid_attr
	    , dev_ctx.scd4x
	    )
	);

/**********************************************************************/
/* Defining access to the global zigbee device context                */
/**********************************************************************/
//needed for proper command handling
struct zb::global_device
{
    static auto& get() { return zb_ctx; }
};

//a shortcut for a convenient access
constinit static auto &zb_ep = zb_ctx.ep<kCO2_EP>();

/**********************************************************************/
/* ZephyrOS devices                                                   */
/**********************************************************************/

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
static const struct device *const co2sensor = DEVICE_DT_GET(DT_NODELABEL(co2sensor));
static const struct gpio_dt_spec led_dt = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
constinit const struct device *co2_power = DEVICE_DT_GET(DT_NODELABEL(scd41_power));
constexpr const struct device *bat_measure = DEVICE_DT_GET(DT_NODELABEL(battery_measure));


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
    .maxBatteryVoltage = 3600,//mV
    .minBatteryVoltage = 900//mV
}>(zb_ep, adc_channels, bat_measure);
constexpr zb_callback_t update_battery_state_zb = &decltype(g_Battery)::update_zb_callback;


/**********************************************************************/
/* End of battery management                                          */
/**********************************************************************/

/**********************************************************************/
/* Message Queue definitions + commands                               */
/**********************************************************************/
enum class CO2Commands
{
    Initial
    , Fetch
    , ManualFetch
    , FactoryReset
};

using CO2Q = msgq::Queue<CO2Commands,4>;
K_MSGQ_DEFINE_TYPED(CO2Q, co2v2);

/**********************************************************************/
/* CO2 measuring thread                                               */
/**********************************************************************/
void co2_thread_entry(void *, void *, void *);
void update_co2_readings_in_zigbee(uint8_t id);

constexpr size_t CO2_THREAD_STACK_SIZE = 512;
constexpr size_t CO2_THREAD_PRIORITY=7;

K_THREAD_DEFINE(co2_thread, CO2_THREAD_STACK_SIZE,
	co2_thread_entry, NULL, NULL, NULL,
	CO2_THREAD_PRIORITY, 0, -1);


bool update_measurements()
{
    bool needsPowerCycle = false;
    bool res = false;
    if ((needsPowerCycle = !regulator_is_enabled(co2_power)))
    {
	regulator_enable(co2_power);
	device_init(co2sensor);
    }
    if (device_is_ready(co2sensor)) {
	int err = -EAGAIN;
	int max_attempts = 3;
	while((err == -EAGAIN) && max_attempts)
	{
	    err = sensor_sample_fetch(co2sensor);
	    if (err == -EAGAIN)
		k_sleep(K_MSEC(1000));
	    --max_attempts;
	}

	if (err == 0)
	{
	    if (needsPowerCycle) //after power up 2 fetches are needed
		sensor_sample_fetch(co2sensor);
	}

	if (!dev_ctx.scd4x.sensor_variant)
	    scd4x_get_variant(co2sensor, &dev_ctx.scd4x.sensor_variant, false);

	res = true;
    }

    //if (zb::qs_to_s(dev_ctx.poll_ctrl.check_in_interval) >= kPowerCycleThresholdSeconds)//seconds
    {
	//check in interval is big enough to power down
	regulator_disable(co2_power);
    }
    return res;
}

static constinit bool g_CO2ErrorState = false;
void co2_thread_entry(void *, void *, void *)
{
    CO2Commands cmd;
    bool needsPowerCycle = false;
    static uint32_t g_last_mark = 0; 
    while(1)
    {
	co2v2 >> cmd;
	switch(cmd)
	{
	    using enum CO2Commands;
	    case FactoryReset:
	    {
		zb::RegRAII co2Reg(co2_power);
		device_init(co2sensor);
		if (device_is_ready(co2sensor)) {
		    scd4x_factory_reset(co2sensor);
		    led::show_pattern(led::kPATTERN_2_BLIPS_NORMED, 2000);
		}
	    }
	    break;
	    case Initial:
	    {
		g_CO2ErrorState = !update_measurements();
		zigbee_enable();
	    }
	    break;
	    case Fetch:
	    {
		auto now = k_uptime_seconds();
		if (g_last_mark && ((now - g_last_mark) < (zb::qs_to_s(dev_ctx.poll_ctrl.check_in_interval) / 2)))
		{
		    //to often
		    continue;
		}
		g_last_mark = now;
	    }
	    [[fallthrough]];
	    case ManualFetch:
	    g_CO2ErrorState = !update_measurements();
	    //post to zigbee thread
	    zigbee_schedule_callback(update_co2_readings_in_zigbee, 0);
	    break;
	}
    }
}

void update_co2_readings_in_zigbee(uint8_t id)
{
    g_Battery.update();
    if (co2sensor && !g_CO2ErrorState)
    {
	sensor_value v;
	sensor_channel_get(co2sensor, SENSOR_CHAN_CO2, &v);
	zb_ep.attr<kAttrCO2Value>() = float(v.val1) / 1'000'000.f;
	sensor_channel_get(co2sensor, SENSOR_CHAN_AMBIENT_TEMP, &v);
	zb_ep.attr<kAttrTempValue>() = zb::zb_zcl_temp_t::FromC(float(v.val1) + float(v.val2) / 1000'000.f);
	sensor_channel_get(co2sensor, SENSOR_CHAN_HUMIDITY, &v);
	zb_ep.attr<kAttrRelHValue>() = zb::zb_zcl_rel_humid_t::FromRelH(float(v.val1) + float(v.val2) / 1000'000.f);

	zb_ep.attr<kAttrStatus1>() = 0;
	zb_ep.attr<kAttrStatus2>() = 0;
    }else
    {
	if (g_CO2ErrorState) zb_ep.attr<kAttrStatus1>() = -1;
	if (!co2sensor) zb_ep.attr<kAttrStatus2>() = -1;
    }
}

zb::CmdHandlingResult on_scd4x_factory_reset()
{
    zb_ep.attr<kAttrFactoryResets>() = dev_ctx.scd4x.factory_resets + 1;
    co2v2 << CO2Commands::FactoryReset;
    co2v2 << CO2Commands::ManualFetch;
    return {};
}


/**********************************************************************/
/* General Zigbee stuff                                               */
/**********************************************************************/

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;

    configure_poll_control<{
	.ep = kCO2_EP, 
	.callback_on_check_in = [](uint8_t){co2v2 << CO2Commands::Fetch;}, 
	.sleepy_end_device = kPowerSaving
    }>(dev_ctx.poll_ctrl);
    update_co2_readings_in_zigbee(0);
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
    }else if (WAKE_UP_BUTTON & has_changed)
    {
	if (!(WAKE_UP_BUTTON & button_state))
	{
	    //released
	    zb_zdo_pim_start_turbo_poll_continuous(kWakeUpDurationMS);
	    co2v2 << CO2Commands::ManualFetch;
	    led::show_pattern(led::kPATTERN_4_BLIPS_NORMED, 1000);
	    zb_ep.attr<kAttrManMeasure>() = dev_ctx.scd4x.manual_measurements + 1;
	}
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

    if (!device_is_ready(co2_power)) {
    	//printk("Power reg not ready");
    	return 0;
    }

    printk("Main: before configuring ADC\r\n");
    if (g_Battery.setup() < 0)
	return 0;

    printk("Main: before settings init\r\n");
    int err = settings_subsys_init();
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
    >;
    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    printk("Main: before zigbee enable\r\n");
    k_thread_start(co2_thread);
    co2v2 << CO2Commands::Initial;

    //zigbee_enable();
    printk("Main: sleep forever\r\n");
    while (1) {
	k_sleep(K_FOREVER);
    }
    return 0;
}
