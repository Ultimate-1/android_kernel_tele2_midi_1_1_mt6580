// lcm driver for Tele2 Midi 1.1 (not tested)

#include "lcm_drv.h"

#define Lcd_Log pr_debug


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF	//END OF REGISTERS MARKER

#define LCM_ID	0x9161

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[100];
};

static struct LCM_setting_table lcm_init_table[] = {
	{0xBF, 3, {0x91, 0x61, 0xF2}},
	{0xB3, 2, {0x00, 0xBB}},
	{0xB4, 2, {0x00, 0xBB}},
	{0xB8, 6, {0x00, 0xBF, 0x01, 0x00, 0xBF, 0x01}},
	{0xBA, 3, {0x34, 0x23, 0x00}},
	{0xC3, 1, {0x04}},
	{0xC4, 2, {0x30, 0x6A}},
	{0xC7, 9, {0x00, 0x01, 0x32, 0x05, 0x65, 0x2A, 0x12, 0xA5, 0xA5}},
	{0xC8, 38, {0x7C, 0x6D, 0x62, 0x58, 0x56, 0x32, 0x27, 0x11, 0x2C, 0x2F, 0x34, 0x59, 0x4E, 0x61, 0x5B, 0x64, 0x5F, 0x5A, 0x50, 0x7C, 0x6D, 0x62, 0x58, 0x56, 0x32, 0x27, 0x11, 0x2C, 0x2F, 0x34, 0x59, 0x4E, 0x61, 0x5B, 0x64, 0x5F, 0x5A, 0x50}},
	{0xD8, 20, {0x30, 0x00, 0x00, 0x10, 0x03, 0x10, 0x01, 0x02, 0x00, 0x01, 0x02, 0x06, 0x61, 0x00, 0x00, 0x71, 0x05, 0x06, 0x61, 0x0C}},
	{0xD9, 19, {0x00, 0x00, 0x00, 0x88, 0x00, 0x00, 0x06, 0x80, 0x00, 0x87, 0x00, 0x5B, 0x6B, 0x00, 0x00, 0x00, 0x00, 0x03, 0x80}},
	{0xBE, 1, {0x01}},
	{0xCC, 10, {0x34, 0x20, 0x38, 0x60, 0x11, 0x91, 0x00, 0x40, 0x00, 0x00}},
	{0xD7, 1, {0x40}},
	{0xC1, 1, {0x10}},
	{0xBE, 1, {0x00}},
	{0xCF, 1, {0x01}},
	/* Sleep Mode Off */
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	/* Display ON */
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{REGFLAG_END_OF_TABLE, 0, {}}
};

static struct LCM_setting_table lcm_suspend_table[] = {
	/* Display off sequence */
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	/* Sleep Mode On */
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_resume_table[] = {
	/* Sleep Mode Off */
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},

	/* Display ON */
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {

        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE :
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Video settings
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active				= 4;
    params->dsi.vertical_backporch					= 6;
    params->dsi.vertical_frontporch					= 12;
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    params->dsi.horizontal_sync_active				= 20;
    params->dsi.horizontal_backporch				= 20;
    params->dsi.horizontal_frontporch				= 20;
	params->dsi.horizontal_blanking_pixel			= 60;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 200;
	
    params->dsi.compatibility_for_nvk				= 0;
	// ESD
	params->dsi.lcm_esd_check_table[0].cmd			= 10;
	params->dsi.lcm_esd_check_table[0].count		= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0]	= -100;
	params->dsi.customization_esd_check_enable		= 1;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);

    push_table(lcm_init_table, sizeof(lcm_init_table) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_suspend_table, sizeof(lcm_suspend_table) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
}


static void lcm_resume(void)
{
	lcm_init();
//	push_table(lcm_resume_table, sizeof(lcm_resume_table) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void) // this function is working
{
	int   array[4];
	char  buffer[6];
	unsigned int id=0;

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	array[0] = 0x00023700;
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0x4,buffer,2); // buffer = 0x91 0x61

	// buffer[0] = 0x91,
	// buffer[0]<<8 = 0x9100,
	// buffer[1] = 0x61,
	// id = 0x9100 + 0x61,
	// id = 0x9161
	id=(buffer[0]<<8)+buffer[1];

	return (LCM_ID == id)?1:0;
}

LCM_DRIVER jd9161_fwvga_dsi_vdo_lcm_drv =
    {
        .name			= "jd9161_fwvga_dsi_vdo",
        .set_util_funcs = lcm_set_util_funcs,
        .get_params     = lcm_get_params,
        .init           = lcm_init,
        .suspend        = lcm_suspend,
        .resume         = lcm_resume,
        .compare_id     = lcm_compare_id
    };

