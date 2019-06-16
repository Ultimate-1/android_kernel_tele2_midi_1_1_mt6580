// working lcm driver for Tele2 Midi 1.1

#include "lcm_drv.h"

#define Lcd_Log pr_debug


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)

//#define PROBLEM_REVISION									// comment this line for use on non-problematic lcm

#define REGFLAG_DELAY             							0xFFE
#define REGFLAG_END_OF_TABLE      							0xFFF	//END OF REGISTERS MARKER

#define LCM_ID	0x7701

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
    unsigned char para_list[128];
};


static struct LCM_setting_table lcm_suspend_table[] = {
	/* Display off sequence */
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	/* Sleep Mode On */
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_resume_table[] = {
	/* Sleep Mode Off */
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	/* Display ON */
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 100, {}},
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

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    params->dsi.mode   = SYNC_PULSE_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video settings
    params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	#ifdef PROBLEM_REVISION
    params->dsi.vertical_sync_active				= 2;
    params->dsi.vertical_backporch					= 24;
    params->dsi.vertical_frontporch					= 18;
    params->dsi.horizontal_sync_active				= 12;
    params->dsi.horizontal_backporch				= 70;
	#else
    params->dsi.vertical_sync_active				= 10;
    params->dsi.vertical_backporch					= 20;
    params->dsi.vertical_frontporch					= 30;
    params->dsi.horizontal_sync_active				= 10;
    params->dsi.horizontal_backporch				= 80;
	#endif
    params->dsi.horizontal_frontporch				= 80;
	
    params->dsi.vertical_active_line				= FRAME_HEIGHT;
    params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	params->dsi.word_count = 1440;
	params->dsi.PLL_CLOCK = 220;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(100);

    //push_table(lcm_init_table, sizeof(lcm_init_table) / sizeof(struct LCM_setting_table), 1);

// +-------------------------------------------------------------------------+ //
// |                                                                         | //
// |                        !!! I AM A CHEATER !!!                           | //
// |                                                                         | //
// +-------------------------------------------------------------------------+ //
// © Ultimate-1
	push_table(lcm_resume_table, sizeof(lcm_resume_table) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_suspend_table, sizeof(lcm_suspend_table) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_resume(void)
{
//	lcm_init();

// +-------------------------------------------------------------------------+ //
// |                                                                         | //
// |                        !!! I AM A CHEATER !!!                           | //
// |                                                                         | //
// +-------------------------------------------------------------------------+ //
// © Ultimate-1
	push_table(lcm_resume_table, sizeof(lcm_resume_table) / sizeof(struct LCM_setting_table), 1); // it works!!!
}

static unsigned int lcm_compare_id(void) // this function is working
{
	int   array[4];
	char  buffer[6];
	unsigned int id=0;

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1,buffer,5); // buffer = 0x77 0x01 0xFF 0xFF 0xFF

	// buffer[0] = 0x77,
	// buffer[0]<<8 = 0x7700,
	// buffer[1] = 0x01,
	// id = 0x7700 + 0x01,
	// id = 0x7701
	id=(buffer[0]<<8)+buffer[1];

	return (LCM_ID == id)?1:0;
}

LCM_DRIVER st7701_fwvga_dsi_vdo_lcm_drv =
    {
        .name			= "st7701_fwvga_dsi_vdo",
        .set_util_funcs = lcm_set_util_funcs,
        .get_params     = lcm_get_params,
        .init           = lcm_init,
        .suspend        = lcm_suspend,
        .resume         = lcm_resume,
        .compare_id     = lcm_compare_id
    };

