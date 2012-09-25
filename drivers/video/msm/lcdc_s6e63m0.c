/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/irqs.h>
#include <mach/vreg.h>
#include "lcdc_s6e63m0.h"

#define SMART_DIMMING

#ifdef SMART_DIMMING
#include "smart_dimming.h"
#endif


static int ldi_enable = 0;
static int current_gamma_lvl = -1;
static int lcd_type = 0;
static int spi_cs;
static int spi_sclk;
static int spi_sdi;
static int lcd_reset;
static int delayed_backlight_value = -1;
static boolean First_Disp_Power_On = FALSE;
static struct msm_panel_common_pdata *lcdc_s6e63m0_pdata;
static struct device *lcd_dev;
extern struct class *sec_class;
#ifdef CONFIG_USES_ACL
static int acl_enable = 0;
static int cur_acl = 0;
static struct class *acl_class;
static struct device *switch_aclset_dev;
#endif
#ifdef GAMMASET_CONTROL
struct class *gammaset_class;
struct device *switch_gammaset_dev;
#endif
static int on_19gamma = 0;

#if defined(CONFIG_MACH_APACHE)
extern int board_hw_revision;
#endif
#define DEFAULT_LCD_ON_BACKLIGHT_LEVEL 23

static DEFINE_SPINLOCK(lcd_ctrl_irq_lock);
static DEFINE_SPINLOCK(bl_ctrl_lock);
static DEVICE_ATTR(lcd_power , 0664, s6e63m0_show_power, s6e63m0_store_power);
static DEVICE_ATTR(lcdtype_file_cmd , 0664, s6e63m0_show_lcd_type, NULL);
#ifdef CONFIG_USES_ACL
static DEVICE_ATTR(aclset_file_cmd,0664, aclset_file_cmd_show, aclset_file_cmd_store);
#endif
#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
static DEVICE_ATTR(gammaset_file_cmd,0664, gammaset_file_cmd_show, gammaset_file_cmd_store);
#endif


#if defined(CONFIG_MACH_APACHE)

static const unsigned short *p22Gamma_set_new_hw[] = {
    NULL,// display off
    s6e63m0_22gamma_30cd_new_hw,// 1
    s6e63m0_22gamma_40cd_new_hw,
    s6e63m0_22gamma_70cd_new_hw,
    s6e63m0_22gamma_90cd_new_hw,
    s6e63m0_22gamma_100cd_new_hw,// 5
    s6e63m0_22gamma_110cd_new_hw,
    s6e63m0_22gamma_120cd_new_hw,
    s6e63m0_22gamma_130cd_new_hw,
    s6e63m0_22gamma_140cd_new_hw,
    s6e63m0_22gamma_150cd_new_hw,// 10
    s6e63m0_22gamma_170cd_new_hw,
    s6e63m0_22gamma_180cd_new_hw,
    s6e63m0_22gamma_190cd_new_hw,
    s6e63m0_22gamma_200cd_new_hw,
    s6e63m0_22gamma_210cd_new_hw,// 15
    s6e63m0_22gamma_220cd_new_hw,
    s6e63m0_22gamma_230cd_new_hw,
    s6e63m0_22gamma_240cd_new_hw,
    s6e63m0_22gamma_250cd_new_hw,
    s6e63m0_22gamma_260cd_new_hw,// 20
    s6e63m0_22gamma_270cd_new_hw,
    s6e63m0_22gamma_280cd_new_hw,
    s6e63m0_22gamma_290cd_new_hw,
    s6e63m0_22gamma_300cd_new_hw,// 24
};

const unsigned short *p19Gamma_set_new_hw[] = {
    NULL,// display off
	s6e63m0_19gamma_30cd_new_hw,// 1
	s6e63m0_19gamma_40cd_new_hw,
	s6e63m0_19gamma_70cd_new_hw,
	s6e63m0_19gamma_90cd_new_hw,
	s6e63m0_19gamma_100cd_new_hw,// 5
	s6e63m0_19gamma_110cd_new_hw,
	s6e63m0_19gamma_120cd_new_hw,
	s6e63m0_19gamma_130cd_new_hw,
	s6e63m0_19gamma_140cd_new_hw,
	s6e63m0_19gamma_150cd_new_hw,// 10
	s6e63m0_19gamma_170cd_new_hw,
	s6e63m0_19gamma_180cd_new_hw,
	s6e63m0_19gamma_190cd_new_hw,
	s6e63m0_19gamma_200cd_new_hw,
	s6e63m0_19gamma_210cd_new_hw,// 15
	s6e63m0_19gamma_220cd_new_hw,
	s6e63m0_19gamma_230cd_new_hw,
	s6e63m0_19gamma_240cd_new_hw,
	s6e63m0_19gamma_250cd_new_hw,
	s6e63m0_19gamma_260cd_new_hw,// 20
	s6e63m0_19gamma_270cd_new_hw,
	s6e63m0_19gamma_280cd_new_hw,
	s6e63m0_19gamma_290cd_new_hw,
	s6e63m0_19gamma_300cd_new_hw,//24
};

#endif

struct brightness_level brt_table[] = {
    { 0, 5 },// Off
    { 20, 1 },// Dimming pulse
    { MIN_BRIGHTNESS_VALUE,1 },// Min
    { 39, 2},
    { 49, 3},
    { 59, 4},
    { 69, 5},
    { 78, 6},
    { 88, 7},
    { 105, 8},
    //{ 108,  8 },
    { 118, 9},// default
    { 127, 10 },
    { 137, 11 },
    { 147, 12 },
    { 157, 13 },
    { 166, 14 },
    { 176, 15 },
    { 186, 16 },
    { 196, 17 },
    { 206, 18 },
    { 215, 19 },
    { 225, 20 },
    { 235, 21 },
    { 245, 22 },
    { MAX_BRIGHTNESS_VALUE, 23 },// Max
};

struct s6e63m0_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	unsigned int			current_brightness;
	unsigned int 			acl_enable;
	unsigned int 			cur_elvss;
	unsigned int			cur_acl;
	unsigned int			bl;
	struct samsung_spi_data main_elvss[4];

#ifdef SMART_DIMMING
	struct str_smart_dim	smart;
	struct mutex			bl_lock;
#endif

};

static struct s6e63m0_state_type s6e63m0_state = {
    .disp_initialized = FALSE,
    .display_on = FALSE,
    .disp_powered_up = FALSE,

};

#ifdef SMART_DIMMING
#define LCD_DCX     93		//Hsync
#define LCD_RDX     91		//Enable
#define LCD_CSX  46		
#define LCD_WRX 45		//Scl
//#define LCD_SCSX 68

#define LDI_MTP_LENGTH 21
#define LDI_MTP_ADDR	0xd3

static unsigned int LCD_DB[8] = {20, 21, 22, 100, 101, 102, 103, 104};   //D0 ~ D7

static int is_load_mtp_offset;

static struct samsung_spi_data prepare_mtp_read[] = {
/* LV2, LV3, MTP lock release code */
	{ .addr = 0xf0, .len = 2, .data = { 0x5a, 0x5a }},
	{ .addr = 0xf1, .len = 2, .data = { 0x5a, 0x5a }},
	{ .addr = 0xfc, .len = 2, .data = { 0x5a, 0x5a }},
/* MTP cell enable */
	{ .addr = 0xd1, .len = 1, .data = { 0x80 }},
};

static struct samsung_spi_data start_mtp_read[] = {
/* MPU  8bit read mode start */
	{ .addr = 0xfc, .len = 2, .data = { 0x0c, 0x00 }},
};	

static void s6e63m0_read_elvss_info();
static void s6e63m0_set_elvss(struct s6e63m0_state_type *lcd,  struct samsung_spi_data *SEQ_ELVSS_Data);
static void samsung_set_acl(struct s6e63m0_state_type *lcd);
static void s6e63mo_read_mtp_info();
u32 transform_gamma(u32 brightness);
static void s6e63m0_set_brightness(int level);
static void s6e63m0_update_gammma(struct s6e63m0_state_type *lcd);


#define REG_ID1	0xDA
#define REG_ID2	0xDB
#define REG_ID3	0xDC
static void lcdc_s6e63m0_set_acl_parameter(struct s6e63m0_state_type *lcd);
	
#endif
	
static int lcdc_s6e63m0_get_ldi_state(void)
{
    return ldi_enable;
}

static void lcdc_s6e63m0_set_ldi_state(int OnOff)
{
    ldi_enable = OnOff;
}


static void s6e63m0_write_byte(boolean dc, u8 data)
{
	uint32 bit;
	int bnum;

	LCD_SCL_LOW
	if(dc)
		LCD_SDI_HIGH
	else
		LCD_SDI_LOW
	udelay(1);			/* at least 20 ns */
	LCD_SCL_HIGH	/* clk high */
	udelay(1);			/* at least 20 ns */

	bnum = 8;			/* 8 data bits */
	bit = 0x80;
	while (bnum--) {
		LCD_SCL_LOW /* clk low */
		if(data & bit)
			LCD_SDI_HIGH
		else
			LCD_SDI_LOW
		udelay(1);
		LCD_SCL_HIGH /* clk high */
		udelay(1);
		bit >>= 1;
	}
	LCD_SDI_LOW

}

static void s6e63m0_read_bytes(u8 cmd, u8 *data, int num)
{
	int bnum;

	/* Chip Select - low */
	LCD_CSX_LOW
	udelay(2);

	/* command byte first */
	s6e63m0_write_byte(0, cmd);
	udelay(2);

	gpio_direction_input(spi_sdi);

	if (num > 1) {
		/* extra dummy clock */
		LCD_SCL_LOW
		udelay(1);
		LCD_SCL_HIGH
		udelay(1);
	}

	/* followed by data bytes */
	bnum = num * 8;	/* number of bits */
	*data = 0;
	while (bnum) {
		LCD_SCL_LOW /* clk low */
		udelay(1);
		*data <<= 1;
		*data |= gpio_get_value(spi_sdi) ? 1 : 0;
		LCD_SCL_HIGH /* clk high */
		udelay(1);
		--bnum;
		if ((bnum % 8) == 0)
			++data;
	}

	gpio_direction_output(spi_sdi, 0);

	/* Chip Select - high */
	udelay(2);
	LCD_CSX_HIGH
}


static void lcdc_s6e63m0_write_no_spinlock(struct setting_table *table)
{
    long i, j;
    
    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP);

    /* Write Command */
    LCD_CSX_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SCL_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SDI_LOW
    udelay(DEFAULT_USLEEP);
    
    LCD_SCL_HIGH
    udelay(DEFAULT_USLEEP);

    for (i = 7; i >= 0; i--) {
        LCD_SCL_LOW
        udelay(DEFAULT_USLEEP);
        if ((table->command >> i) & 0x1)
            LCD_SDI_HIGH
        else
            LCD_SDI_LOW
        udelay(DEFAULT_USLEEP);
        LCD_SCL_HIGH
        udelay(DEFAULT_USLEEP);
    }

    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);

    /* Write Parameter */
    if ((table->parameters) > 0) {
        for (j = 0; j < table->parameters; j++) {
            LCD_CSX_LOW 
            udelay(DEFAULT_USLEEP);
            
            LCD_SCL_LOW
            udelay(DEFAULT_USLEEP);
            LCD_SDI_HIGH
            udelay(DEFAULT_USLEEP);
            LCD_SCL_HIGH
            udelay(DEFAULT_USLEEP);

            for (i = 7; i >= 0; i--) {
                LCD_SCL_LOW
                udelay(DEFAULT_USLEEP);
                if ((table->parameter[j] >> i) & 0x1)
                    LCD_SDI_HIGH
                else
                    LCD_SDI_LOW
                udelay(DEFAULT_USLEEP);
                LCD_SCL_HIGH
                udelay(DEFAULT_USLEEP);
            }

            LCD_CSX_HIGH
            udelay(DEFAULT_USLEEP);
        }
    }
    mdelay(table->wait);
}

static void lcdc_s6e63m0_write(struct setting_table *table)
{
    long i, j;
    unsigned long irqflags;

    spin_lock_irqsave(&lcd_ctrl_irq_lock, irqflags);
    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP);

    /* Write Command */
    LCD_CSX_LOW
    udelay(DEFAULT_USLEEP);
    LCD_SCL_LOW 
    udelay(DEFAULT_USLEEP);
    LCD_SDI_LOW 
    udelay(DEFAULT_USLEEP);
    
    LCD_SCL_HIGH 
    udelay(DEFAULT_USLEEP); 

       for (i = 7; i >= 0; i--) {
        LCD_SCL_LOW
        udelay(DEFAULT_USLEEP);
        if ((table->command >> i) & 0x1)
            LCD_SDI_HIGH
        else
            LCD_SDI_LOW
        udelay(DEFAULT_USLEEP);
        LCD_SCL_HIGH
        udelay(DEFAULT_USLEEP);
    }

    LCD_CSX_HIGH
    udelay(DEFAULT_USLEEP);

    /* Write Parameter */
    if ((table->parameters) > 0) {
        for (j = 0; j < table->parameters; j++) {
            LCD_CSX_LOW 
            udelay(DEFAULT_USLEEP);
            
            LCD_SCL_LOW 
            udelay(DEFAULT_USLEEP);
            LCD_SDI_HIGH 
            udelay(DEFAULT_USLEEP);
            LCD_SCL_HIGH 
            udelay(DEFAULT_USLEEP);

            for (i = 7; i >= 0; i--) {
                LCD_SCL_LOW
                udelay(DEFAULT_USLEEP);
                if ((table->parameter[j] >> i) & 0x1)
                    LCD_SDI_HIGH
                else
                    LCD_SDI_LOW
                udelay(DEFAULT_USLEEP);
                LCD_SCL_HIGH
                udelay(DEFAULT_USLEEP);
            }

            LCD_CSX_HIGH
            udelay(DEFAULT_USLEEP);
        }
    }
    spin_unlock_irqrestore(&lcd_ctrl_irq_lock, irqflags);
    mdelay(table->wait);
}

static void lcdc_s6e63m0_spi_init(void)
{
    /* Setting the Default GPIO's */
    spi_sclk = *(lcdc_s6e63m0_pdata->gpio_num);
    spi_cs   = *(lcdc_s6e63m0_pdata->gpio_num + 1);
    spi_sdi  = *(lcdc_s6e63m0_pdata->gpio_num + 2);
    lcd_reset= *(lcdc_s6e63m0_pdata->gpio_num + 3);

    /* Set the output so that we dont disturb the slave device */
    gpio_set_value(spi_sclk, 0);
    gpio_set_value(spi_sdi, 0);

    /* Set the Chip Select De-asserted */
    gpio_set_value(spi_cs, 0);

}




static void samsung_spi_write_byte(boolean dc, u8 data)
{
	uint32 bit;
	int bnum;

	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, dc ? 1 : 0);
	udelay(1);			/* at least 20 ns */
	gpio_set_value(spi_sclk, 1);	/* clk high */
	udelay(1);			/* at least 20 ns */

	bnum = 8;			/* 8 data bits */
	bit = 0x80;
	while (bnum--) {
		gpio_set_value(spi_sclk, 0); /* clk low */
		gpio_set_value(spi_sdi, (data & bit) ? 1 : 0);
		udelay(1);
		gpio_set_value(spi_sclk, 1); /* clk high */
		udelay(1);
		bit >>= 1;
	}
	gpio_set_value(spi_sdi, 0);

}

static int samsung_serigo(struct samsung_spi_data data)
{
	int i;

	/* Chip Select - low */
	LCD_CSX_LOW;
	udelay(1);

	samsung_spi_write_byte(FALSE, data.addr);
	udelay(1);

	for (i = 0; i < data.len; ++i) {
		samsung_spi_write_byte(TRUE, data.data[i]);
		udelay(1);
	}

	/* Chip Select - high */
	LCD_CSX_HIGH;
//#ifdef DEBUG
#if 0
	pr_info("%s: cmd=0x%02x, #args=%d\n", __func__, data.addr, data.len);
#endif
	return 0;

}

static int samsung_write_cmd(u8 cmd)
{

	/* Chip Select - low */
	LCD_CSX_LOW;
	udelay(2);

	samsung_spi_write_byte(FALSE, cmd);

	/* Chip Select - high */
	udelay(2);
	LCD_CSX_HIGH;
#ifdef DEBUG
	pr_info("%s: cmd=0x%02x\n", __func__, cmd);
#endif
	return 0;

}

static int samsung_serigo_list(struct samsung_spi_data *data, int count)
{
	int i, rc;
	for (i = 0; i < count; ++i, ++data) {
		rc = samsung_serigo(*data);
		if (rc)
			return rc;
		//msleep(10);
		//mdelay(1);
		udelay(1);
	}
	return 0;
}

static void lcdc_s6e63m0_disp_powerup(void)
{
    DPRINT("start %s\n", __func__);

    if (!s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {
        /* Reset the hardware first */
        //TODO: turn on ldo
        gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);

        //LCD_RESET_N_HI
        gpio_set_value(lcd_reset, 1);
        mdelay(10);
        //LCD_RESET_N_LO
        gpio_set_value(lcd_reset, 0);
        mdelay(20);
        //LCD_RESET_N_HI
        gpio_set_value(lcd_reset, 1);
        mdelay(10);

        /* Include DAC power up implementation here */
        s6e63m0_state.disp_powered_up = TRUE;
    }
}

static void lcdc_s6e63m0_disp_powerdown(void)
{
    DPRINT("start %s\n", __func__);

    /* Reset Assert */
    gpio_tlmm_config(GPIO_CFG(129, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
    gpio_set_value(lcd_reset, 0);        
    mdelay(10);        /* ensure power is stable */
    s6e63m0_state.disp_powered_up = FALSE;
}

static void lcdc_s6e63m0_disp_on(void)
{
    int i;
    DPRINT("start %s\n", __func__);

    if (s6e63m0_state.disp_powered_up && !s6e63m0_state.display_on) {

	samsung_serigo_list(panel_sequence,
		sizeof(panel_sequence)/sizeof(*panel_sequence));

	samsung_serigo_list(display_sequence,
			sizeof(display_sequence)/sizeof(*display_sequence));

#ifdef SMART_DIMMING
		s6e63m0_set_brightness(s6e63m0_state.current_brightness);
#else
		lcdc_samsung_set_brightness(samsung_state.current_brightness);
#endif

	samsung_serigo_list(etc_sequence,
		sizeof(etc_sequence)/sizeof(*etc_sequence));

	/* 0x11: Sleep Out */
	samsung_write_cmd(0x11);
	msleep(120);

	/* 0x29: Display On */
	samsung_write_cmd(0x29);
	msleep(10);

        s6e63m0_state.display_on = TRUE;
    }
}

static int lcdc_s6e63m0_get_gamma_value_from_bl(int bl)
{
    int gamma_value = 0;
    int gamma_val_x10 = 0;

    if(bl >= MIN_BL) {
        gamma_val_x10 = 10*(MAX_GAMMA_VALUE-1)*bl/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
        gamma_value = (gamma_val_x10+5)/10;
    }else{
        gamma_value = 0;
    }

    return gamma_value;
}

static void lcdc_s6e63m0_set_brightness(int level)
{

	if(level){
	        if(on_19gamma)
	            lcdc_s6e63m0_write(p19Gamma_set_new_hw[level]);
	        else
	            lcdc_s6e63m0_write(p22Gamma_set_new_hw[level]);

	   lcdc_s6e63m0_write(gamma_update);
        }

    //lcdc_s6e63m0_write(display_on_seq);
    DPRINT("brightness: %d on_19gamma: %d\n",level,on_19gamma);
}

#ifdef SMART_DIMMING
u32 transform_gamma(u32 brightness)
{
	u32 gamma;

	if(brightness > MAX_GAMMA_VALUE)
		brightness = MAX_GAMMA_VALUE;
	else if(brightness <0)
		brightness = 0;
	
	gamma = brightness*10 + 30;

	return gamma;
}

static void s6e63m0_set_brightness(int level)
{
	s6e63m0_state.bl=level;

	s6e63m0_set_elvss(&s6e63m0_state, s6e63m0_state.main_elvss);
	samsung_set_acl(&s6e63m0_state);
       //lcdc_s6e63m0_set_acl_parameter(&s6e63m0_state);
	s6e63m0_update_gammma(&s6e63m0_state); 
	//lcdc_s6e63m0_set_brightness(s6e63m0_state.bl);
}

static void s6e63m0_update_gammma(struct s6e63m0_state_type *lcd)
{

	int gamma = transform_gamma(lcd->bl);
	int i;
	int ret = 0;

	//gamma = 300;
	
	struct samsung_spi_data gamma_regs;
	
	gamma_regs.addr = 0xFA;
	gamma_regs.len = 22;

		
	for(i=0;i<40;i++)  {
		gamma_regs.data[i] = 0;
	}
	
	gamma_regs.data[0] = 0x02;
	
	if(gamma<0)
	{
		gamma=0;		
	}

	//gamma_table °ª ¾ò¾î¿È.
	calc_gamma_table(&lcd->smart, gamma, (gamma_regs.data)+1);
	

	//for(i=0; i<40;i++)
	//	DPRINT("[%d] gamma [%d] , gamma_regs [%02x]\n", i,gamma, gamma_regs.data[i]);
	
	//write
	samsung_serigo_list(&gamma_regs, 1);

	
	//
	//gamma_update
	samsung_serigo_list(gamma_update,
		sizeof(gamma_update)/sizeof(*gamma_update));

	lcd->current_brightness = lcd->bl;
}
#endif

static ssize_t s6e63m0_show_power(struct device *dev, struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", s6e63m0_state.display_on );
}

static ssize_t s6e63m0_store_power(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char *endp;
    int power = simple_strtoul(buf, &endp, 0);    

    DPRINT("s6e63m0_store_power is called: %d", power);

    if (power == 1)
        lcdc_s6e63m0_panel_on(dev);
    else if(power == 0)
        lcdc_s6e63m0_panel_off(dev);


    return 0;
}

static ssize_t s6e63m0_show_lcd_type(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count = 0;
    
    if(lcd_type == 0)
        count = sprintf(buf, "SMD_S6E63M0\n");
        
    return count;
}

#ifdef CONFIG_USES_ACL

static ssize_t aclset_file_cmd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    DPRINT("called %s \n",__func__);
    return sprintf(buf,"%u\n", s6e63m0_state.acl_enable);
}

static ssize_t aclset_file_cmd_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
    int value, i, gamma;
    sscanf(buf, "%d", &value);
    DPRINT("in aclset_file_cmd_store, input value = %d \n", value);

    if (!lcdc_s6e63m0_get_ldi_state()){
        DPRINT("return because LDI is disabled, input value = %d \n", value);
        return size;
    }

    if ((value != 0) && (value != 1)){
        DPRINT("aclset_file_cmd_store value is same : value(%d)\n", value);
        return size;
    }

    if (s6e63m0_state.acl_enable != value){
        s6e63m0_state.acl_enable = value;
	samsung_set_acl(&s6e63m0_state);
//	lcdc_s6e63m0_set_acl_parameter(&s6e63m0_state);

	}

    return size;
}
#endif

#ifdef GAMMASET_CONTROL //for 1.9/2.2 gamma control from platform
static ssize_t gammaset_file_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	DPRINT("called %s \n",__func__);

	return sprintf(buf,"%u\n", current_gamma_lvl);
}
static ssize_t gammaset_file_cmd_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
	int value;
	
    sscanf(buf, "%d", &value);

	//printk(KERN_INFO "[gamma set] in gammaset_file_cmd_store, input value = %d \n",value);
	if (!lcdc_s6e63m0_get_ldi_state())	{
		DPRINT("[gamma set] return because LDI is disabled, input value = %d \n", value);
		return size;
	}

	if ((value != 0) && (value != 1))	{
		DPRINT("\ngammaset_file_cmd_store value(%d) on_19gamma(%d) \n", value,on_19gamma);
		return size;
	}

	if (value != on_19gamma)	{
		on_19gamma = value;
		lcdc_s6e63m0_set_brightness(current_gamma_lvl);
	}

	return size;
}
#endif

extern unsigned long acpuclk_usr_set_max(void);

static int lcdc_s6e63m0_panel_on(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);

    if (!s6e63m0_state.disp_initialized) {
    	 acpuclk_usr_set_max(); 
        /* Configure reset GPIO that drives DAC */
        lcdc_s6e63m0_pdata->panel_config_gpio(1);
        lcdc_s6e63m0_spi_init();    /* LCD needs SPI */
        lcdc_s6e63m0_disp_powerup();
#ifdef SMART_DIMMING
	if(!is_load_mtp_offset) {

			
		s6e63m0_read_elvss_info();
		
		DPRINT("%s + read_mtp_offset spi_cs_sel\n", __func__);


		s6e63mo_read_mtp_info();

		is_load_mtp_offset =  1;


		gpio_set_value_cansleep(129, 0);
		msleep(25);
		gpio_set_value_cansleep(129, 1);
		msleep(25);
	
		lcdc_s6e63m0_pdata->panel_config_gpio(1);
		lcdc_s6e63m0_spi_init();
		lcdc_s6e63m0_disp_powerup();

	}
#endif
        lcdc_s6e63m0_disp_on();
        s6e63m0_state.disp_initialized = TRUE;
    }
 
    if(!First_Disp_Power_On)
    {
    	First_Disp_Power_On = TRUE;
	s6e63m0_set_brightness(DEFAULT_LCD_ON_BACKLIGHT_LEVEL);	
    }
	

    if(delayed_backlight_value != -1) {
        lcdc_s6e63m0_set_brightness(delayed_backlight_value);
        DPRINT("delayed backlight on %d\n", delayed_backlight_value);
    }

    return 0;
}


static int lcdc_s6e63m0_panel_off(struct platform_device *pdev)
{
    int i;
    unsigned long irqflags;

    DPRINT("start %s\n", __func__);


    if (s6e63m0_state.disp_powered_up && s6e63m0_state.display_on) {


	/* 0x10: Sleep In */
	samsung_write_cmd(0x10);
	msleep(120);

        lcdc_s6e63m0_pdata->panel_config_gpio(0);
        lcdc_s6e63m0_disp_powerdown();
        s6e63m0_state.display_on = FALSE;
        s6e63m0_state.disp_initialized = FALSE;
    }
    return 0;
}


static void s6e63m0_set_elvss(struct s6e63m0_state_type *lcd, struct samsung_spi_data *SEQ_ELVSS_Data)
{
	int i = 0;
	
	switch (lcd->bl) {
	case 0 ... 7: /* 30cd ~ 100cd */
		//if(lcd->cur_elvss!=29) 
		{
			DPRINT("Dynamic ELVSS : -2.4V (30cd~100cd) \n");
			//for(i=0;i<4;i++)
			//	DPRINT("ELVSS Value : %2x \n", SEQ_ELVSS_Data[0].data[i]);
			
			samsung_serigo_list(SEQ_ELVSS_Data,1);
			lcd->cur_elvss=24;
			samsung_serigo_list(SEQ_ELVSS_ON,1);
		}
		break;
	case 8 ... 13: /* 110cd ~ 160cd */
		//if(lcd->cur_elvss!=32) 
		{		
			DPRINT("Dynamic ELVSS : -2.7V (110cd~160cd) \n");
			//for(i=0;i<4;i++)
			//	DPRINT("ELVSS Value : %2x \n", SEQ_ELVSS_Data[1].data[i]);
			
			samsung_serigo_list(SEQ_ELVSS_Data+1,1);
			lcd->cur_elvss=27;
			samsung_serigo_list(SEQ_ELVSS_ON,1);
		}
		break;
	case 14 ... 17: /* 170cd ~ 200cd */
		//if(lcd->cur_elvss!=35) 
		{
			DPRINT("Dynamic ELVSS : -2.9V (170cd~200cd) \n");
			//for(i=0;i<4;i++)
			//	DPRINT("ELVSS Value : %2x \n", SEQ_ELVSS_Data[2].data[i]);
			
			samsung_serigo_list(SEQ_ELVSS_Data+2,1);
			lcd->cur_elvss=29;
			samsung_serigo_list(SEQ_ELVSS_ON,1);
		}
		break;
	case 18 ... 27: /* 210cd ~ 300cd */
		//if(lcd->cur_elvss!=39)
		{
			DPRINT("Dynamic ELVSS : -3.5V (210cd~300cd) \n");
			//for(i=0;i<4;i++)
			//	DPRINT("ELVSS Value : %2x \n", SEQ_ELVSS_Data[3].data[i]);
			
			samsung_serigo_list(SEQ_ELVSS_Data+3,1);
			lcd->cur_elvss=35;
			samsung_serigo_list(SEQ_ELVSS_ON,1);
		}
		break;
	default:
		DPRINT("Dynamic ELVSS : Error Table \n");
		lcd->cur_elvss=0;
		samsung_serigo_list(SEQ_ELVSS_OFF,1);
		break;
	}

}


static void samsung_set_acl(struct s6e63m0_state_type *lcd)
{
	if(lcd->acl_enable)
	{
		switch (lcd->bl) 
		{
			case 0 ... 1: /* 30cd ~ 40cd */
				//if (lcd->cur_acl != 0) 
				{
					samsung_serigo_list(SEQ_ACL_OFF,1);
					DPRINT("ACL : off!!\n");
					lcd->cur_acl = 0;
				}
				break;
			case 2 ... 13: /* 50cd ~ 160cd */
				//if (lcd->cur_acl != 40) 
				{
					samsung_serigo_list(ACL_cutoff_set[0],1);
					samsung_serigo_list(SEQ_ACL_ON,1);
					DPRINT("ACL : 40!!\n");
					lcd->cur_acl = 40;
				}
				break;
			case 14 ... 19: /* 170cd ~ 220cd */
				//if (lcd->cur_acl != 48) 
				{
					samsung_serigo_list(ACL_cutoff_set[0],1);
					samsung_serigo_list(SEQ_ACL_ON,1);
					DPRINT("ACL : 40!!\n");
					lcd->cur_acl = 40;
				}
				break;
			case 20 ... 27: /* 230cd ~ 300cd */	
				//if (lcd->cur_acl != 50) 
				{
					samsung_serigo_list(ACL_cutoff_set[0],1);
					samsung_serigo_list(SEQ_ACL_ON,1);
					DPRINT("ACL : 40!!\n");
					lcd->cur_acl = 40;
				}			
				break;
			default:		/* Error */
				DPRINT("ACL : Error Table \n");
				lcd->cur_acl = 0;
				break;
		}
	}
	else
	{
		//if (lcd->cur_acl != 0) 
		{
			samsung_serigo_list(SEQ_ACL_OFF,1);
			DPRINT("ACL : off!!\n");
			lcd->cur_acl = 0;
		}
	}
	
}

static void lcdc_s6e63m0_set_backlight(struct msm_fb_data_type *mfd)
{    
    int bl_level = mfd->bl_level;
    int gamma_level = 0;
    int i;


    gamma_level = lcdc_s6e63m0_get_gamma_value_from_bl(bl_level);


    // LCD should be turned on prior to backlight
    if(s6e63m0_state.disp_initialized == FALSE && gamma_level > 0){
        delayed_backlight_value = gamma_level;
        DPRINT("delayed_backlight_value = gamma_level\n");
        return;
    } else {
        delayed_backlight_value = -1;
    }

    DPRINT("bl: %d, gamma: %d,  %dccd\n",bl_level,gamma_level,(gamma_level+3)*10);
	
#ifdef SMART_DIMMING
	s6e63m0_set_brightness(gamma_level);
#else
	lcdc_s6e63m0_set_brightness(gamma_level);
#endif


}


#ifdef SMART_DIMMING

static void s6e63m0_parallel_read(u8 cmd, u8 *data, size_t len)
{
	int i;
	
	int delay = 1;

	gpio_set_value(LCD_DCX, 0);
	udelay(delay);
	gpio_set_value(LCD_WRX, 0);
	for (i = 0; i < 8; i++) {
		gpio_direction_output(LCD_DB[i], (cmd >> i) & 1);
	}

	udelay(delay);
	gpio_set_value(LCD_WRX, 1);
	udelay(delay);
	gpio_set_value(LCD_DCX, 1);
	
	for (i = 0; i < 8; i++) {
		gpio_tlmm_config(GPIO_CFG(LCD_DB[i],  0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),GPIO_ENABLE);
	}
		
	udelay(delay);
	gpio_set_value(LCD_RDX, 0);
	udelay(delay);
	gpio_set_value(LCD_RDX, 1);
	udelay(delay);

	while (len--) {
		char d = 0;
		gpio_set_value(LCD_RDX, 0);
		udelay(delay);
		for (i = 0; i < 8; i++)
			d |= gpio_get_value(LCD_DB[i]) << i;
		*data++ = d;
		gpio_set_value(LCD_RDX, 1);
		udelay(delay);
	}
	gpio_set_value(LCD_RDX, 1);
	
}

static int configure_mtp_gpios(bool enable)
{
	int i;
	int ret = 0;
	if (enable) {
		gpio_tlmm_config(GPIO_CFG(LCD_RDX,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(LCD_DCX,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),GPIO_ENABLE);
		
		for (i = 0; i < 8; i++) 
			gpio_tlmm_config(GPIO_CFG(LCD_DB[i],  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_6MA),GPIO_ENABLE);	
	} else {
		gpio_tlmm_config(GPIO_CFG(LCD_DCX, 1, 1, GPIO_CFG_NO_PULL, GPIO_CFG_6MA) ,GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(LCD_RDX, 1, 1, GPIO_CFG_NO_PULL, GPIO_CFG_6MA) ,GPIO_ENABLE);

		for (i = 0; i < 8; i++)
			gpio_tlmm_config(GPIO_CFG(LCD_DB[i], 1, 1, GPIO_CFG_NO_PULL, GPIO_CFG_6MA) ,GPIO_ENABLE);		
	}		
	return ret;
}


static int s6e63m0_parallel_setup_gpios(bool init)
{
	int ret;
	
	if (init) {
		configure_mtp_gpios(true);

		gpio_set_value(LCD_CSX, 0);	
		gpio_set_value(LCD_WRX, 1);
		gpio_set_value(LCD_RDX, 1);
		gpio_set_value(LCD_DCX, 0);
	} else {
		configure_mtp_gpios(false);
		gpio_set_value(LCD_CSX, 0);
		
	}

	return 0;
}

static void s6e63mo_read_mtp_info()
{
	
	int i=0;
	unsigned int lcd_id = 0;

	u8 main_mtp_data[LDI_MTP_LENGTH] = {0,};
	u8 gamma_regs[LDI_MTP_LENGTH] = {0,};

	init_table_info(&(s6e63m0_state.smart), lcd_id);


//SPI I/F MODE
//	samsung_write_cmd(0x01);
//	msleep(120);

//Main LCD

	DPRINT("%s\n", __func__);

	samsung_serigo_list(prepare_mtp_read,
		sizeof(prepare_mtp_read)/sizeof(*prepare_mtp_read));


	samsung_serigo_list(start_mtp_read,
		sizeof(start_mtp_read)/sizeof(*start_mtp_read));

	s6e63m0_parallel_setup_gpios(true);
	
	s6e63m0_parallel_read(LDI_MTP_ADDR, main_mtp_data, sizeof(main_mtp_data));
	

	//for(i=0;i<LDI_MTP_LENGTH;i++) {
	//	DPRINT("main_mtp_data[%d] : %02x\n", i, main_mtp_data[i]);
	//}

	calc_voltage_table(&(s6e63m0_state.smart), main_mtp_data);

	
	s6e63m0_parallel_setup_gpios(false);

	LCD_CSX_HIGH;

	gpio_set_value(LCD_WRX, 1);
	udelay(1);


}
#endif


static void s6e63m0_set_elvss_data(u8 value, struct samsung_spi_data *elvss) 
{
	int i = 0;
	int j = 0;
	u8 alph[4] = {0xC,0x8, 0x6, 0x0};
	u8 data;
	
	for(i=0; i<4;i++) {
		elvss[i].addr = 0xB2;
		elvss[i].len = 4;
		data = value+alph[i]; 
		DPRINT("value[%d]+alph[%d] = %2x\n" , value, i, data); 
		if(data > 0x29) 
			data = 0x29;
		for(j=0; j<4;j++) {		 
			elvss[i].data[j] = data;
			DPRINT("elvss[%d], data[%d] = %2x\n" , i, j, elvss[i].data[j]); 
		}
	}
}

static void s6e63m0_read_elvss_info()
{
	u8 reg_id2 = 0;
	u8 reg_id3 = 0;
	int i = 0;

	//read ID2 value for elvss
	s6e63m0_read_bytes(REG_ID2, &reg_id2, 1);

	DPRINT("REG_ID2 : 0x%02x\n", reg_id2);
	
	if(reg_id2 == 0xC1)
	{

		s6e63m0_read_bytes(REG_ID3, &reg_id3, 1);
		DPRINT("main elvss value : %02x\n", reg_id3);
	
		if(reg_id3 == 0x0) {
			for(i=0;i<4;i++) 
				s6e63m0_state.main_elvss[i] = *(SEQ_ELVSS_set[i]);
		} else {
				s6e63m0_set_elvss_data(reg_id3, s6e63m0_state.main_elvss);
		}
	}
	else {
		for(i=0;i<4;i++) 
				s6e63m0_state.main_elvss[i] = *(SEQ_ELVSS_set[i]);
	}

}



static int __init lcdc_s6e63m0_probe(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);

    if (pdev->id == 0) {
        lcdc_s6e63m0_pdata = pdev->dev.platform_data;
        
        lcdc_s6e63m0_spi_init();
        if( !spi_sclk || !spi_cs || !spi_sdi || !lcd_reset)
        {
            DPRINT("SPI Init Error. %d,%d,%d,%d\n",spi_sclk,spi_cs,spi_sdi,lcd_reset);
            spi_cs = 46;
            spi_sclk = 45;
            spi_sdi = 47;
            lcd_reset = 129;
        }    

        /* sys fs */
        if(IS_ERR(sec_class))
            pr_err("Failed to create class(sec)!\n");
     
        lcd_dev = device_create(sec_class, NULL, 0, NULL, "sec_lcd");
        if(IS_ERR(lcd_dev))
            pr_err("Failed to create device(lcd)!\n");
     
        if(device_create_file(lcd_dev, &dev_attr_lcdtype_file_cmd) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_lcdtype_file_cmd.attr.name);
        if(device_create_file(lcd_dev, &dev_attr_lcd_power) < 0)
            pr_err("Failed to create device file(%s)!\n", dev_attr_lcd_power.attr.name); 

        lcdc_s6e63m0_set_ldi_state(1);
#ifdef CONFIG_USES_ACL
        DPRINT("making aclset sysfile start\n");
        acl_class = class_create(THIS_MODULE, "aclset");
        if (IS_ERR(acl_class))
            DPRINT("Failed to create class(acl_class)!\n");
    
        switch_aclset_dev = device_create(acl_class, NULL, 0, NULL, "switch_aclset");
        if (IS_ERR(switch_aclset_dev))
            DPRINT("Failed to create device(switch_aclset_dev)!\n");
    
        if (device_create_file(switch_aclset_dev, &dev_attr_aclset_file_cmd) < 0)
            DPRINT("Failed to create device file(%s)!\n", dev_attr_aclset_file_cmd.attr.name);
#endif
#ifdef GAMMASET_CONTROL
	gammaset_class = class_create(THIS_MODULE, "gammaset");
	if (IS_ERR(gammaset_class))
		DPRINT("Failed to create class(gammaset_class)!\n");

	switch_gammaset_dev = device_create(gammaset_class, NULL, 0, NULL, "switch_gammaset");
	if (IS_ERR(switch_gammaset_dev))
		DPRINT("Failed to create device(switch_gammaset_dev)!\n");

	if (device_create_file(switch_gammaset_dev, &dev_attr_gammaset_file_cmd) < 0)
		DPRINT("Failed to create device file(%s)!\n", dev_attr_gammaset_file_cmd.attr.name);
#endif
            return 0;
    }
    msm_fb_add_device(pdev);

    return 0;
}

static void lcdc_s6e63m0_shutdown(struct platform_device *pdev)
{
    DPRINT("start %s\n", __func__);
    lcdc_s6e63m0_set_ldi_state(0);
    current_gamma_lvl = -1;
    //lcdc_s6e63m0_panel_off(pdev);
    gpio_set_value(lcd_reset, 0);    
}

static struct platform_driver this_driver = {
    .probe = lcdc_s6e63m0_probe,
    .shutdown = lcdc_s6e63m0_shutdown,
    .driver = {
        .name   = "lcdc_s6e63m0_wvga",
        .owner  = THIS_MODULE,
    },
};

static struct msm_fb_panel_data s6e63m0_panel_data = {
    .on = lcdc_s6e63m0_panel_on,
    .off = lcdc_s6e63m0_panel_off,
    .set_backlight = lcdc_s6e63m0_set_backlight,
};

static struct platform_device this_device = {
    .name = "lcdc_panel",
    .id = 1,
    .dev = {
        .platform_data = &s6e63m0_panel_data,
    }
};

static int __init lcdc_s6e63m0_panel_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
    if (msm_fb_detect_client("lcdc_s6e63m0_wvga"))
    {
        DPRINT("%s: msm_fb_detect_client failed!\n", __func__);
        return 0;
    }
#endif
    DPRINT("start %s\n", __func__);
    
    ret = platform_driver_register(&this_driver);
    if (ret)
    {
        DPRINT("%s: platform_driver_register failed! ret=%d\n", __func__, ret);
        return ret;
    }

    pinfo = &s6e63m0_panel_data.panel_info;
    pinfo->xres = LCDC_FB_XRES;
    pinfo->yres = LCDC_FB_YRES;
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 24;
    pinfo->fb_num = 2;
#if defined(CONFIG_MACH_APACHE)
    pinfo->clk_rate = 25423*1000;
#else
    pinfo->clk_rate = 24576* 1000;
#endif
    pinfo->bl_max = 255;
    pinfo->bl_min = 1;

    pinfo->lcdc.h_back_porch = LCDC_HBP;
    pinfo->lcdc.h_front_porch = LCDC_HFP;
    pinfo->lcdc.h_pulse_width = LCDC_HPW;
    pinfo->lcdc.v_back_porch = LCDC_VBP;
    pinfo->lcdc.v_front_porch = LCDC_VFP;
    pinfo->lcdc.v_pulse_width = LCDC_VPW;
    pinfo->lcdc.border_clr = 0;     /* blk */
    pinfo->lcdc.underflow_clr = 0xff0000;       /* red */
    pinfo->lcdc.hsync_skew = 0;

    DPRINT("%s\n", __func__);

    ret = platform_device_register(&this_device);
    if (ret)
    {
        DPRINT("%s: platform_device_register failed! ret=%d\n", __func__, ret);
        platform_driver_unregister(&this_driver);
    }

    return ret;
}

module_init(lcdc_s6e63m0_panel_init);


