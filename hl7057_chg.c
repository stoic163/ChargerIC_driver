#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros*/
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/rtc.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#include "hl7057.h"
#include<linux/usb/msm_hsusb.h>
#include<linux/qpnp/qpnp-adc.h>




const unsigned int VBAT_CVTH[] = {
	3500000, 3520000, 3540000, 3560000,
	3580000, 3600000, 3620000, 3640000,
	3660000, 3680000, 3700000, 3720000,
	3740000, 3760000, 3780000, 3800000,
	3820000, 3840000, 3860000, 3880000,
	3900000, 3920000, 3940000, 3960000,
	3980000, 4000000, 4020000, 4040000,
	4060000, 4080000, 4100000, 4120000,
	4140000, 4160000, 4180000, 4200000,
	4220000, 4240000, 4260000, 4280000,
	4300000, 4320000, 4340000, 4360000,
	4380000, 4400000, 4420000, 4440000
};

const unsigned int CSTH[] = {
	550000, 650000, 750000, 850000,
	1050000, 1150000, 1350000, 1450000,
	1550000, 1650000, 1750000, 1850000,
	2050000, 2150000, 2350000, 2450000
};

/*hl7057 REG00 IINLIM[5:0]*/
const unsigned int INPUT_CSTH[] = {
	100000, 500000, 1000000, 5000000
};

/* hl7057 REG0A BOOST_LIM[2:0], mA */
const unsigned int BOOST_CURRENT_LIMIT[] = {
	500, 750, 1200, 1400, 1650, 1875, 2150
};

#ifdef CONFIG_OF
#else
#define hl7057_SLAVE_ADDR_WRITE 0xD4
#define hl7057_SLAVE_ADDR_Read  0xD5
#ifdef I2C_SWITHING_CHARGER_CHANNEL
#define hl7057_BUSNUM I2C_SWITHING_CHARGER_CHANNEL
#else
#define hl7057_BUSNUM 2
#endif
#endif

#define BATT_CV  4400000
#define BATT_LOW_VOL  3400
#define BATT_WARM_CURRENT 600000
#define BATT_GOOD_CURRENT 2000000
#define BATT_COOL_CURRENT 600000

struct hl7057_charger {
	struct device *chg_dev;
	struct i2c_client *client;
	struct power_supply batt_psy;
	struct power_supply *usb_psy;
	struct workqueue_struct  *chg_workqueue;
	struct delayed_work chg_delay_work;

	bool	enable_chg;
	bool	recharge;
	unsigned int	chg_type;
	unsigned int 	battery_present;
	unsigned int 	online;
	unsigned int	status;
	unsigned int	ichg;
	unsigned int	aicr;
	unsigned int	cv_value;
	unsigned int 	batt_current;

	struct qpnp_vadc_chip	*vadc_dev;

	bool			batt_hot;
	bool			batt_warm;
	bool			batt_cool;
	bool			batt_cold;
	bool			batt_good;
	int				usb_psy_ma;
	struct mutex	icl_set_lock;
};

static struct wake_lock wlock;
static struct hl7057_charger *hl7057_chg;
static const struct i2c_device_id hl7057_i2c_id[] = { {"hl7057", 0}, {} };

//add by jason start
static int wlocked = 0;
static struct rtc_timer hl7057_timer;
static struct rtc_device *hl7057_rtc = NULL;
static int enable = 0;
static int charging_enable_gpio = 77;
//add by jason end


#ifdef  CONFIG_PM
static struct timespec suspend_time_before;
static struct timespec after;
static int suspend_resume_mark = 0;
#endif


unsigned int charging_value_to_parameter(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
	if (val < array_size)
		return parameter[val];
	pr_err("Can't find the parameter\n");
	return parameter[0];
}

unsigned int charging_parameter_to_value(const unsigned int *parameter, const unsigned int array_size, const unsigned int val)
{
	unsigned int i;

	pr_debug_ratelimited("array_size = %d\n", array_size);

	for (i = 0; i < array_size; i++) {
		if (val == *(parameter + i))
			return i;
	}

	pr_err("NO register value match\n");
	/* TODO: ASSERT(0);	// not find the value */
	return 0;
}

static unsigned int bmt_find_closest_level(const unsigned int *pList, unsigned int number, unsigned int level)
{
	unsigned int i;
	unsigned int max_value_in_last_element;

	if (pList[0] < pList[1])
		max_value_in_last_element = 1;
	else
		max_value_in_last_element = 0;

	if (max_value_in_last_element == 1) {
		for (i = (number - 1); i != 0; i--) {	/* max value in the last element */
			if (pList[i] <= level) {
				pr_debug_ratelimited("zzf_%d<=%d, i=%d\n", pList[i], level, i);
				return pList[i];
			}
		}
		pr_err("Can't find closest level\n");
		return pList[0];
		/* return 000; */
	} else {
		for (i = 0; i < number; i++) {	/* max value in the first element */
			if (pList[i] <= level)
				return pList[i];
		}
		pr_err("Can't find closest level\n");
		return pList[number - 1];
		/* return 000; */
	}
}

unsigned char hl7057_reg[HL7057_REG_NUM] = { 0 };
static DEFINE_MUTEX(hl7057_access_lock);

static int hl7057_read_byte(u8 reg_addr, u8 *rd_buf, int rd_len)
{
	int ret = 0;
	struct i2c_adapter *adap = hl7057_chg->client->adapter;
	struct i2c_msg msg[2];
	u8 *w_buf = NULL;
	u8 *r_buf = NULL;

	memset(msg, 0, 2 * sizeof(struct i2c_msg));

	w_buf = kzalloc(1, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;
	r_buf = kzalloc(rd_len, GFP_KERNEL);
	if (r_buf == NULL)
		return -1;

	*w_buf = reg_addr;

	msg[0].addr = hl7057_chg->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = w_buf;

	msg[1].addr = hl7057_chg->client->addr;
	msg[1].flags = 1;
	msg[1].len = rd_len;
	msg[1].buf = r_buf;

	ret = i2c_transfer(adap, msg, 2);

	memcpy(rd_buf, r_buf, rd_len);

	kfree(w_buf);
	kfree(r_buf);
	return ret;
}

int hl7057_write_byte(unsigned char reg_num, u8 *wr_buf, int wr_len)
{
	int ret = 0;
	struct i2c_adapter *adap = hl7057_chg->client->adapter;
	struct i2c_msg msg;
	u8 *w_buf = NULL;

	memset(&msg, 0, sizeof(struct i2c_msg));

	w_buf = kzalloc(wr_len, GFP_KERNEL);
	if (w_buf == NULL)
		return -1;

	w_buf[0] = reg_num;
	memcpy(w_buf + 1, wr_buf, wr_len);

	msg.addr = hl7057_chg->client->addr;
	msg.flags = 0;
	msg.len = wr_len;
	msg.buf = w_buf;

	ret = i2c_transfer(adap, &msg, 1);

	kfree(w_buf);

	return ret;
}

unsigned int hl7057_read_interface(unsigned char reg_num, unsigned char *val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char hl7057_reg = 0;
	unsigned int ret = 0;

	ret = hl7057_read_byte(reg_num, &hl7057_reg, 1);
	pr_debug_ratelimited("[hl7057_read_interface] Reg[%x]=0x%x\n", reg_num, hl7057_reg);
	hl7057_reg &= (MASK << SHIFT);
	*val = (hl7057_reg >> SHIFT);
	pr_debug_ratelimited("[hl7057_read_interface] val=0x%x\n", *val);

	return ret;
}

unsigned int hl7057_config_interface(unsigned char reg_num, unsigned char val, unsigned char MASK, unsigned char SHIFT)
{
	unsigned char hl7057_reg = 0;
	unsigned char hl7057_reg_ori = 0;
	unsigned int ret = 0;

	mutex_lock(&hl7057_access_lock);
	ret = hl7057_read_byte(reg_num, &hl7057_reg, 1);
	hl7057_reg_ori = hl7057_reg;
	hl7057_reg &= ~(MASK << SHIFT);
	hl7057_reg |= (val << SHIFT);

	/*clear bit7 of Reg04*/
	if (reg_num == HL7057_CON4)
		hl7057_reg &= ~(1 << CON4_RESET_SHIFT);

	ret = hl7057_write_byte(reg_num, &hl7057_reg, 2);
	mutex_unlock(&hl7057_access_lock);
	pr_debug_ratelimited("[hl7057_config_interface] write Reg[%x]=0x%x from 0x%x\n", reg_num, hl7057_reg, hl7057_reg_ori);

	/* Check */
	/* hl7057_read_byte(reg_num, &hl7057_reg, 1); */
	/* printk("[hl7057_config_interface] Check Reg[%x]=0x%x\n", reg_num, hl7057_reg); */

	return ret;
}

/* write one register directly */
unsigned int hl7057_reg_config_interface(unsigned char reg_num, unsigned char val)
{
	unsigned char hl7057_reg = val;

	return hl7057_write_byte(reg_num, &hl7057_reg, 2);
}

/******************operation register of hl7057************************/
void hl7057_set_tmr_rst(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_TMR_RST_MASK),
				(unsigned char)(CON0_TMR_RST_SHIFT)
				);
}

unsigned int hl7057_get_otg_status(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_OTG_MASK),
				(unsigned char)(CON0_OTG_SHIFT)
				);
	return val;
}

void hl7057_set_en_stat(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON0),
				(unsigned char)(val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
}

unsigned int hl7057_get_en_stat(void)
{
	unsigned char val = 0;
	hl7057_read_interface((unsigned char)(HL7057_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_EN_STAT_MASK),
				(unsigned char)(CON0_EN_STAT_SHIFT)
				);
	return val;	
}

unsigned int hl7057_get_chip_status(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_STAT_MASK),
				(unsigned char)(CON0_STAT_SHIFT)
				);
	return val;
}

unsigned int hl7057_get_boost_status(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_BOOST_MASK),
				(unsigned char)(CON0_BOOST_SHIFT)
				);
	return val;

}

unsigned int hl7057_get_fault_status(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON0),
				(unsigned char *)(&val),
				(unsigned char)(CON0_FAULT_MASK),
				(unsigned char)(CON0_FAULT_SHIFT)
				);
	return val;
}

void hl7057_set_input_charging_current(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);
}

unsigned int hl7057_get_input_charging_current(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON1),
				(unsigned char *)(&val),
				(unsigned char)(CON1_LIN_LIMIT_MASK),
				(unsigned char)(CON1_LIN_LIMIT_SHIFT)
				);

	return val;
}

void hl7057_set_v_low(unsigned int val)
{

	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_LOW_V_MASK),
				(unsigned char)(CON1_LOW_V_SHIFT)
				);
}

void hl7057_set_te(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_TE_MASK),
				(unsigned char)(CON1_TE_SHIFT)
				);
}

void hl7057_set_ce(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_CE_MASK),
				(unsigned char)(CON1_CE_SHIFT)
				);
}

void hl7057_set_hz_mode(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_HZ_MODE_MASK),
				(unsigned char)(CON1_HZ_MODE_SHIFT)
				);
}

void hl7057_set_opa_mode(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON1),
				(unsigned char)(val),
				(unsigned char)(CON1_OPA_MODE_MASK),
				(unsigned char)(CON1_OPA_MODE_SHIFT)
				);
}

void hl7057_set_oreg(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OREG_MASK),
				(unsigned char)(CON2_OREG_SHIFT)
				);
}
void hl7057_set_otg_pl(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_PL_MASK),
				(unsigned char)(CON2_OTG_PL_SHIFT)
				);
}
void hl7057_set_otg_en(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON2),
				(unsigned char)(val),
				(unsigned char)(CON2_OTG_EN_MASK),
				(unsigned char)(CON2_OTG_EN_SHIFT)
				);
}

unsigned int hl7057_get_vender_code(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_VENDER_CODE_MASK),
				(unsigned char)(CON3_VENDER_CODE_SHIFT)
				);
	return val;
}
unsigned int hl7057_get_pn(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_PIN_MASK),
				(unsigned char)(CON3_PIN_SHIFT)
				);
	return val;
}

unsigned int hl7057_get_revision(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON3),
				(unsigned char *)(&val),
				(unsigned char)(CON3_REVISION_MASK),
				(unsigned char)(CON3_REVISION_SHIFT)
				);
	return val;
}

void hl7057_set_reset(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_RESET_MASK),
				(unsigned char)(CON4_RESET_SHIFT)
				);
}

void hl7057_set_iocharge(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_CHR_MASK),
				(unsigned char)(CON4_I_CHR_SHIFT)
				);
}

void hl7057_set_iterm(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON4),
				(unsigned char)(val),
				(unsigned char)(CON4_I_TERM_MASK),
				(unsigned char)(CON4_I_TERM_SHIFT)
				);
}

void hl7057_set_dis_vreg(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_DIS_VREG_MASK),
				(unsigned char)(CON5_DIS_VREG_SHIFT)
				);
}

void hl7057_set_io_level(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_IO_LEVEL_MASK),
				(unsigned char)(CON5_IO_LEVEL_SHIFT)
				);
}

unsigned int hl7057_get_sp_status(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_SP_STATUS_MASK),
				(unsigned char)(CON5_SP_STATUS_SHIFT)
				);
	return val;
}

unsigned int hl7057_get_en_level(void)
{
	unsigned char val = 0;

	hl7057_read_interface((unsigned char)(HL7057_CON5),
				(unsigned char *)(&val),
				(unsigned char)(CON5_EN_LEVEL_MASK),
				(unsigned char)(CON5_EN_LEVEL_SHIFT)
				);
	return val;
}

void hl7057_set_vsp(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON5),
				(unsigned char)(val),
				(unsigned char)(CON5_VSP_MASK),
				(unsigned char)(CON5_VSP_SHIFT)
				);
}

void hl7057_set_i_safe(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_ISAFE_MASK),
				(unsigned char)(CON6_ISAFE_SHIFT)
				);
}

void hl7057_set_v_safe(unsigned int val)
{
	hl7057_config_interface((unsigned char)(HL7057_CON6),
				(unsigned char)(val),
				(unsigned char)(CON6_VSAFE_MASK),
				(unsigned char)(CON6_VSAFE_SHIFT)
				);
}

/************************ function of hl7057***********************************/
static int hl7057_dump_register(struct device *chg_dev)
{
	int i;

	pr_err("hl7057 dump register:\n");
	for (i = 0; i < HL7057_REG_NUM; i++) {
		hl7057_read_byte(i, &hl7057_reg[i], 1);
		pr_err("[0x%x]=0x%x\n", i, hl7057_reg[i]);
	}
	pr_err("\n");

	return 0;
}

static int hl7057_charge_enable(struct hl7057_charger *hl7057_chg, bool enable_chg)
{
	unsigned int status = 0;

	if (enable_chg) {
		hl7057_set_ce(0);
		hl7057_set_hz_mode(0);
		hl7057_set_opa_mode(0);
		hl7057_set_te(1);
	} else {
		hl7057_set_ce(1);
	}

	return status;

}

static int hl7057_get_charge_state(struct hl7057_charger *hl7057_chg)
{
	unsigned int ret_val;

	ret_val = hl7057_get_en_stat();

	return ret_val;
}

static int hl7057_get_battery_charging_done(struct hl7057_charger *hl7057_chg)
{
	unsigned int ret_val;

	ret_val = hl7057_get_chip_status();

	if (ret_val == 0x2)
		hl7057_chg->status = true;
	else
		hl7057_chg->status = false;

	return hl7057_chg->status;

}

static int hl7057_get_battery_current(struct hl7057_charger *hl7057_chg)
{
	unsigned int array_size;
	unsigned char reg_value;

	array_size = ARRAY_SIZE(CSTH);
	hl7057_read_interface(0x4, &reg_value, 0xf, 0x3); /* IINLIM */
	hl7057_chg->ichg = charging_value_to_parameter(CSTH, array_size, reg_value);

	return hl7057_chg->ichg;

}

static int hl7057_get_battery_input_current(struct hl7057_charger *hl7057_chg)
{
	unsigned int array_size;
	unsigned int register_value;

	array_size = ARRAY_SIZE(INPUT_CSTH);
	register_value = hl7057_get_input_charging_current();
	hl7057_chg->aicr = charging_value_to_parameter(INPUT_CSTH, array_size, register_value);

	return hl7057_chg->aicr;	
}

static int hl7057_get_battery_cv(struct hl7057_charger *hl7057_chg)
{
	unsigned int array_size;
	unsigned char reg_value;

	array_size = ARRAY_SIZE(VBAT_CVTH);
	hl7057_read_interface(0x2, &reg_value, 0x3f, 0x2);
	hl7057_chg->cv_value = charging_value_to_parameter(VBAT_CVTH, array_size, reg_value);

	return hl7057_chg->cv_value;

}

static int hl7057_set_battery_constant_voltage(struct hl7057_charger *hl7057_chg, unsigned int cv_value)
{
	int status = 0;
	unsigned short int array_size;
	unsigned int set_cv_voltage;
	unsigned short int register_value;

	/*static kal_int16 pre_register_value; */
	array_size = ARRAY_SIZE(VBAT_CVTH);
	/*pre_register_value = -1; */
	set_cv_voltage = bmt_find_closest_level(VBAT_CVTH, array_size, cv_value);

	register_value = charging_parameter_to_value(VBAT_CVTH, array_size, set_cv_voltage);
	pr_debug("charging_set_cv_voltage register_value=0x%x %d %d\n", register_value, cv_value, set_cv_voltage);
	hl7057_set_oreg(register_value);

	return status;

}

static int hl7057_set_battery_current(struct hl7057_charger *hl7057_chg, unsigned int batt_current)
{
	unsigned int set_batt_current;
	unsigned int array_size;
	unsigned int register_value;

	if (batt_current <= 35000) {
		hl7057_set_io_level(1);
	} else {
		hl7057_set_io_level(0);
		array_size = ARRAY_SIZE(CSTH);
		set_batt_current = bmt_find_closest_level(CSTH, array_size, batt_current);
		register_value = charging_parameter_to_value(CSTH, array_size, set_batt_current);
		hl7057_set_iocharge(register_value);
	}

	return 0;
}

static int hl7057_get_batt_present(struct hl7057_charger *hl7057_chg)
{	
	hl7057_chg->battery_present = 1;
	return  hl7057_chg->battery_present ? 1 : 0;
}

#define DEFAULT_TEMP  250
static int hl7057_get_battery_temp(struct hl7057_charger *hl7057_chg)
{
	int ret = 0;
	struct qpnp_vadc_result results;

	if (!hl7057_get_batt_present(hl7057_chg) || !hl7057_chg->vadc_dev) {
		return DEFAULT_TEMP;
	}

	ret = qpnp_vadc_read(hl7057_chg->vadc_dev, P_MUX4_1_1, &results);
	if (ret) {
		pr_err("Unable to read batt temperature rc=%d\n", ret);
		return DEFAULT_TEMP;
	}
	pr_debug("[hl7057][get_bat_temp] adc_code:%d, temp:%lld\n", results.adc_code, results.physical);

	return (int)results.physical;
}

#define HYSTERESIS_DECIDEGC 2
#define BAT_HOT_DECIDEGC  60
#define BAT_WARM_DECIDEGC 45
#define BAT_COOL_DECIDEGC 10
#define BAT_COLD_DECIDEGC 0
static int hl7057_battery_health(struct hl7057_charger *hl7057_chg)
{
	union power_supply_propval ret = {0, };
	int batt_temp = 0 ;

	batt_temp = hl7057_get_battery_temp(hl7057_chg);
	//batt_low_temp = batt_temp - HYSTERESIS_DECIDEGC

	if (batt_temp > BAT_HOT_DECIDEGC) {
		hl7057_chg->batt_hot = true;
	} else if ((batt_temp > BAT_WARM_DECIDEGC) && (batt_temp <= BAT_HOT_DECIDEGC)) {
		hl7057_chg->batt_warm = true;
	} else if ((batt_temp > BAT_COLD_DECIDEGC) && (batt_temp <= BAT_COOL_DECIDEGC)) {
		hl7057_chg->batt_cool = true;
	} else if (batt_temp <= BAT_COLD_DECIDEGC) {
		hl7057_chg->batt_cold = true;
	} else {
		hl7057_chg->batt_good = true;
	}

	if (hl7057_chg->batt_hot) {
		ret.intval = POWER_SUPPLY_HEALTH_OVERHEAT;
	} else if (hl7057_chg->batt_warm) {
		ret.intval = POWER_SUPPLY_HEALTH_WARM;
	} else if (hl7057_chg->batt_cool) {
		ret.intval = POWER_SUPPLY_HEALTH_COOL;
	} else if (hl7057_chg->batt_cold) {
		ret.intval = POWER_SUPPLY_HEALTH_COLD;
	} else {
		ret.intval = POWER_SUPPLY_HEALTH_GOOD;
	}
	pr_debug("[hl7057][battery_health] ret.intval:%d\n", ret.intval);
	
	return ret.intval;
}

static int hl7057_get_prop_battery_voltage_now(struct hl7057_charger *hl7057_chg)
{
	int ret = 0;
	struct qpnp_vadc_result results;

	if (!hl7057_chg->vadc_dev)
		return 0;

	ret = qpnp_vadc_read(hl7057_chg->vadc_dev, VBAT_SNS, &results);
	if (ret) {
		pr_err("Unable to read vbat rc=%d\n", ret);
		return 0;
	}

	return results.physical;
}

static void hl7057_charger_type_detect(struct hl7057_charger *hl7057_chg)
{
	int reg_value;

	struct msm_otg *motg = container_of(hl7057_chg->usb_psy, struct msm_otg, usb_psy);
	hl7057_chg->chg_type = motg->chg_type;

	if (hl7057_chg->chg_type == USB_DCP_CHARGER){
		reg_value = 0x03;
	}else if (hl7057_chg->chg_type == USB_INVALID_CHARGER) {
		reg_value = 0x00;
	}else {
		reg_value = 0x01;
	}

	/*set input charge current of USB */
	hl7057_set_input_charging_current(reg_value);
}

static void hl7057_charger_usb_detect(struct hl7057_charger *hl7057_chg )
{
	union power_supply_propval ret = {0, };

	if (hl7057_chg->usb_psy) {
		hl7057_chg->usb_psy->get_property(hl7057_chg->usb_psy, POWER_SUPPLY_PROP_ONLINE, &ret);
		hl7057_chg->online = ret.intval;
	}

}

#if 0
#define CURRENT_SUSPEND	2
#define CURRENT_100_MA	100
#define CURRENT_500_MA	500
static int hl7057_set_appopriate_usb_current(struct hl7057_charger *hl7057_chg)
{
	int current_ma,set_input_current;
	int usb_current = hl7057_chg->usb_psy_ma;
	int register_value;
	unsigned int array_size;

	if (usb_current <= CURRENT_SUSPEND) {
		current_ma = CURRENT_100_MA;
	} else {
		current_ma = CURRENT_500_MA;
	}

	/* set input current */
	array_size = ARRAY_SIZE(INPUT_CSTH);
	set_input_current = bmt_find_closest_level(INPUT_CSTH, array_size, current_ma);
	register_value = charging_parameter_to_value(INPUT_CSTH, array_size, set_input_current);
	printk("charging_set_input_currnet register_value=0x%x %d %d\n", register_value, current_ma, set_input_current);
	hl7057_set_input_charging_current(register_value);

	return 0;
}
#endif
#define BATT_MAX_VOL 4360
#define BATT_CUTOFF_VOL 3400
#define BATT_VOL_DIFF 20
#define BATT_SOC_DIFF 50 
int pre_batt_vol = 0;
int batt_soc_pre = 0;
static int hl7057_soc_base_voltage(struct hl7057_charger *hl7057_chg)
{
	int batt_vol;
	int batt_range_vol = 0;
	int batt_remain_vol = 0;
	int batt_soc = 0;

	batt_vol = hl7057_get_prop_battery_voltage_now(hl7057_chg) / 1000;
	if (!batt_vol) {
		pr_err("can't read battery voltage!\n");
		batt_soc = BATT_SOC_DIFF;
		return batt_soc;
	}

	if (abs(pre_batt_vol - batt_vol) <= BATT_VOL_DIFF) {
		batt_vol = pre_batt_vol;
	}
	pre_batt_vol = batt_vol;
	
	batt_range_vol = BATT_MAX_VOL - BATT_CUTOFF_VOL;
	batt_remain_vol = batt_vol - BATT_CUTOFF_VOL;
	batt_soc = (batt_remain_vol * 100) / batt_range_vol;
	if ((batt_soc - 100) >= 0) {
		batt_soc = 100;
	}
	if (batt_soc <= 0) {
		batt_soc = 0;
	}

	if (batt_soc_pre != 0) {
		if ((batt_soc_pre - batt_soc) >= 2) {
			batt_soc = batt_soc_pre - 1;
			batt_soc_pre = batt_soc;
		} else if ((batt_soc - batt_soc_pre) >= 2) {
			batt_soc = batt_soc_pre + 1;
			batt_soc_pre = batt_soc;
		} else {
			batt_soc_pre = batt_soc;
		}
	}else {
		batt_soc_pre = batt_soc;
	}
	return batt_soc;
}
//add by jason start
static void hl7057_set_timer(void)
{
	struct rtc_time tm;
	ktime_t now, expires;

	rtc_read_time(hl7057_rtc, &tm);
	now = rtc_tm_to_ktime(tm);
	expires = ktime_add(now, ktime_set(15, 0));

	rtc_alarm_irq_enable(hl7057_rtc, 1);
	rtc_timer_start(hl7057_rtc, &hl7057_timer, expires, ktime_set(0, 0));
	if (wlocked) {
		wake_unlock(&wlock);
		wlocked = 0;
	}
	
}
static void hl7057_init_gpio(void)
{
	if (gpio_is_valid(charging_enable_gpio)) {
		if (gpio_request(charging_enable_gpio, "hl7057_enable")) {
			pr_err("Failed to requeset gpio %d", charging_enable_gpio);
		} else {
			gpio_direction_output(charging_enable_gpio, 0);
			enable = 1;
			return ;
		}
	} else {
		pr_err("Charging enable gpio %d, isn't valid \n", charging_enable_gpio);
	}
	enable = 0;
	
}
//add by jason end

static int hl7057_charger_work(struct work_struct *work)
{
	struct delayed_work *delay_work;
	struct hl7057_charger *hl7057_chip;
	int batt_health = 0;
	int batt_vol = 0;
	int bat_cv = 0;
	int chg_state = 0;
	int batt_soc = 0;
	int batt_temp = 0;

	delay_work = container_of(work, struct delayed_work, work);
	hl7057_chip = container_of(delay_work, struct hl7057_charger, chg_delay_work);

	/*reset watch dog*/
	hl7057_set_tmr_rst(0x1);

	/* charger detect USB present */
	hl7057_charger_usb_detect(hl7057_chip);
	if (hl7057_chip->online) 
{
		hl7057_chip->enable_chg = true;
		hl7057_charge_enable(hl7057_chip, hl7057_chip->enable_chg);
		pr_debug("[hl7057]USB is online:%d, charger_enabled:%d\n", hl7057_chip->online, hl7057_chip->enable_chg);
	} else {
		hl7057_chip->enable_chg = false;
		hl7057_charge_enable(hl7057_chip, hl7057_chip->enable_chg);
		pr_err("USB is offline, charger disable! \n");
	}

	batt_health = hl7057_battery_health(hl7057_chip);
	if (batt_health == POWER_SUPPLY_HEALTH_OVERHEAT) {
		hl7057_chip->enable_chg = false;
		hl7057_charge_enable(hl7057_chip, hl7057_chip->enable_chg);
	} else if (batt_health == POWER_SUPPLY_HEALTH_WARM) {
		hl7057_chip->batt_current = BATT_WARM_CURRENT;
		hl7057_set_battery_current(hl7057_chip, hl7057_chip->batt_current);
	} else if (batt_health == POWER_SUPPLY_HEALTH_COOL) {
		hl7057_chip->batt_current = BATT_COOL_CURRENT;
		hl7057_set_battery_current(hl7057_chip, hl7057_chip->batt_current);
	} else if (batt_health == POWER_SUPPLY_HEALTH_COLD) {
		hl7057_chip->enable_chg = false;
		hl7057_charge_enable(hl7057_chip, hl7057_chip->enable_chg);
	} else if (batt_health == POWER_SUPPLY_HEALTH_GOOD) {
		hl7057_chip->batt_current = BATT_GOOD_CURRENT;
		hl7057_set_battery_current(hl7057_chip, hl7057_chip->batt_current);
	}

	/*get and set battery constant voltage*/
	bat_cv = hl7057_get_battery_cv(hl7057_chip);
	if (bat_cv != BATT_CV) {
		hl7057_chip->cv_value = BATT_CV;
		hl7057_set_battery_constant_voltage(hl7057_chip, hl7057_chip->cv_value);
	} 

	chg_state = hl7057_get_charge_state(hl7057_chip);
	batt_vol = hl7057_get_prop_battery_voltage_now(hl7057_chip);
	batt_vol = batt_vol / 1000;
	batt_soc =hl7057_soc_base_voltage(hl7057_chip);

	hl7057_charger_type_detect(hl7057_chip);
	hl7057_get_battery_current(hl7057_chip);
	hl7057_get_battery_input_current(hl7057_chip);
	batt_temp = hl7057_get_battery_temp(hl7057_chip);

	power_supply_changed(&hl7057_chip->batt_psy);

	pr_err("[hl7057_charger_work] type:%d, online:%d, state:%d, batt_cv:%d, batt_v:%d, batt_soc:%d, chg_curr:%d, batt_curr:%d, health:%d, temp:%d, chg_done:%d\n ", \
		  hl7057_chip->chg_type, hl7057_chip->online, chg_state, hl7057_chip->cv_value, batt_vol, batt_soc, hl7057_chip->aicr, \
		  hl7057_chip->ichg, batt_health, batt_temp, hl7057_chip->status);

	hl7057_dump_register(hl7057_chip->chg_dev);

	if ((hl7057_chip->chg_type == USB_SDP_CHARGER)) {
		queue_delayed_work(hl7057_chip->chg_workqueue, &hl7057_chip->chg_delay_work, msecs_to_jiffies(15000));
	} else {
		hl7057_set_timer(); //add by jason
	}

	return 0;

}
//add by jason start
static void hl7057_rtc_timer_work(void *data)
{
	wake_lock(&wlock);
	wlocked = 1;
	queue_delayed_work(hl7057_chg->chg_workqueue, &hl7057_chg->chg_delay_work, msecs_to_jiffies(200));
}
//add by jason end

static void hl7057_hw_init(struct hl7057_charger *hl7057_chg)
{

#if defined(HIGH_BATTERY_VOLTAGE_SUPPORT) 
		hl7057_reg_config_interface(0x06, 0x6a);	/* ISAFE = 2150mA, VSAFE = 4.4V */
#else
		hl7057_reg_config_interface(0x06, 0x70);
#endif
		hl7057_reg_config_interface(0x00, 0xC0);	/* kick chip watch dog */
		hl7057_reg_config_interface(0x01, 0x78);	/* weakbatery=3.7v,TE=1, CE=0, HZ_MODE=0, OPA_MODE=0 */
		hl7057_reg_config_interface(0x02, 0xae);	/*voltage=4.4V*/
		hl7057_reg_config_interface(0x05, 0x03);

	//	hl7057_reg_config_interface(0x04, 0x7A);	/* current=2450mA, te=150mA */
		hl7057_reg_config_interface(0x04, 0x01);   /*current = 550mA, te = 100mA*/
}

static enum power_supply_property hl7057_battery_properties[] = {
	POWER_SUPPLY_PROP_CHARGE_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_DONE,
	POWER_SUPPLY_PROP_CAPACITY_RAW,	
};

static int hl7057_battery_get_property(struct power_supply *psy,
			enum power_supply_property prop,
			union power_supply_propval *val) 
{
	struct hl7057_charger *hl7057_chg = container_of(psy, struct hl7057_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		val->intval = hl7057_chg->enable_chg;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = hl7057_chg->chg_type;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = hl7057_chg->online;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_RAW:
		val->intval = hl7057_soc_base_voltage(hl7057_chg);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = hl7057_battery_health(hl7057_chg);
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = hl7057_get_battery_temp(hl7057_chg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_DONE:
		val->intval = hl7057_get_battery_charging_done(hl7057_chg);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = hl7057_get_battery_current(hl7057_chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_NOW:
		val->intval = hl7057_get_battery_input_current(hl7057_chg);
		break;
	default:
		return (-EINVAL);
	}
	pr_debug("get_property: prop(%d) = %d\n", (int)prop, (int)val->intval);

	return 0;
}

#if 0
static int hl7057_battery_set_property(struct power_supply *psy,
			enum power_supply_property prop,
			const union power_supply_propval *val)
{
	struct hl7057_charger *hl7057_chg = container_of(psy, struct hl7057_charger, batt_psy);
	
	pr_err("set property: prop(%d) = %d\n", (int)prop, (int)val->intval);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		printk("[hl7507]hl7057_chg_enable_chg:%d\n", val->intval);
		hl7057_charge_enable(hl7057_chg, val->intval);
		break;
	default:
		return (-EINVAL);
	}	
	return 0;
}

static int hl7057_batt_property_is_writeable(struct power_supply *psy,
				enum power_supply_property psp)
{	
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_ENABLED:
		return 1;
	default:
		break;
	}

	return 0;
	
}
#endif
static int hl7057_set_appopriate_usb_current(struct hl7057_charger *hl7057_chg)
{
	return 0;
}

static void hl7057_external_power_changed(struct power_supply *psy)
{
	struct hl7057_charger *hl7057_chip  = container_of(psy, struct hl7057_charger, batt_psy);
	union power_supply_propval prop = {0,};
	int icl = 0, ret;

	ret = hl7057_chip->usb_psy->get_property(hl7057_chip->usb_psy, POWER_SUPPLY_PROP_CURRENT_MAX, &prop);
	if (ret < 0)
		pr_err("Get CURRENT_MAX from usb failed, ret=%d\n", ret);
	else
		icl = prop.intval / 1000;
		pr_debug("current_limit = %d\n", icl);

	if (hl7057_chip->usb_psy_ma != icl) {
		mutex_lock(&hl7057_chip->icl_set_lock);
		hl7057_chip->usb_psy_ma = icl;
		ret = hl7057_set_appopriate_usb_current(hl7057_chip);
		mutex_unlock(&hl7057_chip->icl_set_lock);
		if (ret < 0)
			pr_err("Set appropriate current failed, ret=%d\n", ret);
	}
	hl7057_rtc_timer_work(NULL);
}

static int hl7057_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct power_supply *usb_psy;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	pr_err("[hl7057_driver_probe]\n");
	hl7057_chg = devm_kzalloc(&client->dev, sizeof(struct hl7057_charger), GFP_KERNEL);
	if (!hl7057_chg)
		return -ENOMEM;

	hl7057_chg->client = client;
	hl7057_chg->chg_dev = &client->dev;
	hl7057_chg->usb_psy = usb_psy;

	if (of_find_property(hl7057_chg->chg_dev->of_node, "qcom,chg-vadc", NULL)) {
		/* early for VADC get, defer probe if needed */
		hl7057_chg->vadc_dev = qpnp_get_vadc(hl7057_chg->chg_dev, "chg");
		if (IS_ERR(hl7057_chg->vadc_dev)) {
			ret = PTR_ERR(hl7057_chg->vadc_dev);
			if (ret != -EPROBE_DEFER)
				pr_err("vadc property configured incorrectly\n");
			return ret;
		}
	}

	/* Register charger device for MTK platform */
//	hl7057_chg->chg_dev = charger_device_register(hl7057_chg->chg_dev_name, &client->dev, hl7057_chg, &hl7057_chg_ops, &hl7057_chg->chg_props);
   /*resgister battery power_supply for QUALCOMM */
	hl7057_chg->batt_psy.name		= "battery";
	hl7057_chg->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	hl7057_chg->batt_psy.get_property = hl7057_battery_get_property;
//	hl7057_chg->batt_psy.set_property = hl7057_battery_set_property;
//	hl7057_chg->batt_psy.property_is_writeable = hl7057_batt_property_is_writeable;
	hl7057_chg->batt_psy.properties	= hl7057_battery_properties;
	hl7057_chg->batt_psy.num_properties = ARRAY_SIZE(hl7057_battery_properties);
	hl7057_chg->batt_psy.external_power_changed = hl7057_external_power_changed;

	ret = power_supply_register(hl7057_chg->chg_dev, &hl7057_chg->batt_psy);
	if (ret < 0) {
		pr_err("Register power_supply failed, rc = %d\n", ret);
		return -ENODEV;
	}

	if (IS_ERR_OR_NULL(hl7057_chg->chg_dev)) {
		pr_err("%s: register charger device failed\n", __func__);
		ret = PTR_ERR(hl7057_chg->chg_dev);
		return ret;
	}

	ret = hl7057_get_vender_code();
	if ( ret != 2) {
		pr_err("%s: get vendor id failed\n", __func__);
		return -ENODEV;
	}
	
	hl7057_chg->chg_workqueue = create_singlethread_workqueue("hl7057_chg_work");
	INIT_DELAYED_WORK(&hl7057_chg->chg_delay_work, hl7057_charger_work);

	wake_lock_init(&wlock, WAKE_LOCK_SUSPEND, "hl7057_wlock");

	hl7057_hw_init(hl7057_chg);

	//add by jason start
	hl7057_rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	rtc_timer_init(&hl7057_timer, hl7057_rtc_timer_work, NULL);

	hl7057_init_gpio();

	mutex_init(&hl7057_chg->icl_set_lock);
	//add by jason end

	return 0;
}


static int hl7057_driver_remove(struct i2c_client *client)
{
	if (!hl7057_chg->online) {
		wake_unlock(&wlock);
	}
	mutex_destroy(&hl7057_chg->icl_set_lock);
	pr_err("[hl7057] charger IC remove!\n");	
	return 0;
}

#ifdef CONFIG_PM
static int hl7057_suspend(struct device *chg_dev)
{
	struct i2c_client *client  =to_i2c_client(chg_dev);
	struct hl7057_charger *hl7057_chg = i2c_get_clientdata(client);

	read_persistent_clock(&suspend_time_before);
	cancel_delayed_work_sync(&hl7057_chg->chg_delay_work);
	return 0;
}

static int hl7057_resume(struct device *chg_dev)
{
	struct i2c_client *client = to_i2c_client(chg_dev);
	struct hl7057_charger *hl7057_chg = i2c_get_clientdata(client);

	suspend_resume_mark = 1;

	read_persistent_clock(&after);
	after = timespec_sub(after, suspend_time_before);	
	queue_delayed_work(hl7057_chg->chg_workqueue, &hl7057_chg->chg_delay_work, msecs_to_jiffies(20));

	return 0;
}

static const struct dev_pm_ops hl7057_pm_ops = {
	.suspend	=hl7057_suspend,
	.resume		=hl7057_resume,
};
#endif

#ifdef CONFIG_OF
static const struct of_device_id hl7057_of_match[] = {
	{.compatible = "qcom,charger"},
	{},
};
#else
static struct i2c_board_hl7057_chg i2c_hl7057 __initdata = {
	I2C_BOARD_hl7057_chg("hl7057", (hl7057_SLAVE_ADDR_WRITE >> 1))
};
#endif

static struct i2c_driver hl7057_driver = {
	.driver = {
		.name = "hl7057",
#ifdef CONFIG_PM
	//	.pm		= &hl7057_pm_ops,
#endif
		.owner  =THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = hl7057_of_match,
#endif
		},
	.probe = hl7057_driver_probe,
	.remove = hl7057_driver_remove,
	.id_table = hl7057_i2c_id,
};
static int __init hl7057_init(void)
{

	if (i2c_add_driver(&hl7057_driver) != 0)
		pr_err("Failed to register hl7057 i2c driver.\n");
	else
		pr_err("Success to register hl7057 i2c driver.\n");

	return 0;
}

static void __exit hl7057_exit(void)
{
	i2c_del_driver(&hl7057_driver);
}

module_init(hl7057_init);
module_exit(hl7057_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("I2C hl7057 Driver");
MODULE_AUTHOR("Jason Yuan<stoic163@163.com>");
