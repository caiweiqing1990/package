/*
 ***************************************************************************
 * Ralink Tech Inc.
 * 4F, No. 2 Technology 5th Rd.
 * Science-based Industrial Park
 * Hsin-chu, Taiwan, R.O.C.
 *
 * (c) Copyright, Ralink Technology, Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *
 ***************************************************************************
 */

#ifndef __POWER_MODE_SWITCH_H__
#define __POWER_MODE_SWITCH_H__
/*CASE SWITCH mode*/
#define gprs_on		1
#define gprs_off		2
#define SAT_System_Reset	3
#define SAT_ON		4
#define SAT_OFF		5
#define GPS_ON		6
#define GPS_OFF		7
#define GE1_TXD2_ON		8
#define GE1_TXD2_OFF	9

/*peripheral mode*/
#define VD5_ON		*GPIO71_40_DATA &=~(1<<19)//ND_D7 59
#define VD5_OFF		*GPIO71_40_DATA |=(1<<19)//ND_D7 59
#define USB_HUP_5V_disab	*GPIO71_40_DATA |=(1<<4)//JTAG_TRST44
#define USB_HUP_5V_EN	*GPIO71_40_DATA &=~(1<<4)//JTAG_TRST44 USB_HUP_5V_EN USB_HUP_5V_disab
#define PERIOHERAL_GPIO_OUT	*GPIO71_40_DIR |= (1<<4)|(1<<19)
#define PERIOHERAL_GPIO_IN	*GPIO71_40_DIR &=~((1<<4)|(1<<19))
/*GPS mode*/
#define GPS_ONOFF_L		*GPIO71_40_DATA &=~(1<<8)
#define GPS_ONOFF_H		*GPIO71_40_DATA |=(1<<8)
#define GPS_ONOFF_OUT	*GPIO71_40_DIR |= (1<<8)//NA_ND_wp 48 
#define GPS_ONOFF_IN		*GPIO71_40_DIR &=~(1<<8)//NA_ND_wp 48 

#define GPS_2_ONOFF_L		*GPIO71_40_DATA &=~(1<<7)
#define GPS_2_ONOFF_H		*GPIO71_40_DATA |=(1<<7)
#define GPS_2_ONOFF_OUT		*GPIO71_40_DIR |= (1<<7)//NA_RE_n 47 
#define GPS_2_ONOFF_IN		*GPIO71_40_DIR &=~(1<<7)//NA_RE_n 47 

/*9602 mode*/
#define Iridium9602_ON_L	*GPIO71_40_DATA |=(1<<1)//JTAG_TD1 41
#define Iridium9602_ON_H	*GPIO71_40_DATA &=~(1<<1)
#define TX_5V_ONOFF9602_L	*GPIO71_40_DATA &=~(1<<6)//NA_ND_we 46 
#define TX_5V_ONOFF9602_H	*GPIO71_40_DATA |=(1<<6)
#define TX_5V_ONOFF9602_OUT	*GPIO71_40_DIR |= (1<<6)
#define TX_5V_ONOFF9602_IN	*GPIO71_40_DIR &=~((1<<6))
#define Iridium9602_ON_OUT	*GPIO71_40_DIR |= (1<<1)
#define Iridium9602_ON_IN	*GPIO71_40_DIR &=~((1<<1))
/*3732 mode*/
#define GPRS_POWER_L		*GPIO71_40_DATA &=~(1<<0)//JTAG_DO 40
#define GPRS_POWER_H		*GPIO71_40_DATA |=(1<<0)
#define M3732_ONOFF_L 	*GPIO71_40_DATA &=~(1<<10)
#define M3732_ONOFF_H 	*GPIO71_40_DATA |=(1<<10)
#define M3732_GPIO_OUT	*GPIO71_40_DIR |= (1<<0)|(1<<10)
#define M3732_GPIO_IN	*GPIO71_40_DIR &=~((1<<0)|(1<<10))
/*2500 mode*/
#define SM2500_POWER_L	*GPIO71_40_DATA &=~(1<<9)
#define SM2500_POWER_H	*GPIO71_40_DATA |=(1<<9)
#define SM2500_ON_OFF_L	*GPIO71_40_DATA &=~(1<<11)
#define SM2500_ON_OFF_H	*GPIO71_40_DATA |=(1<<11)
#define SAT_OFF_L 		*GPIO71_40_DATA &=~(1<<2)
#define SAT_OFF_H 		*GPIO71_40_DATA |=(1<<2)
#define SAT_ON_L 		*GPIO71_40_DATA &=~(1<<3)
#define SAT_ON_H		*GPIO71_40_DATA |=(1<<3)
#define SAT_RESET_L 		*GPIO23_00_DATA &=~(1<<0)
#define SAT_RESET_H 		*GPIO23_00_DATA |=(1<<0)
#define SAT_GPIO_OUT		*GPIO71_40_DIR |= (1<<11)|(1<<9)|(1<<3)|(1<<2)// ,NA_ND_RB_N 49 ,NA_ND_ALE 51 ,EPHY_tck 43 ,EPHY_TMS 42
#define SAT_GPIO_IN		*GPIO71_40_DIR &=~((1<<11)|(1<<9)|(1<<3)|(1<<2))// ,NA_ND_RB_N 49 ,NA_ND_ALE 51 ,EPHY_tck 43 ,EPHY_TMS 42
#define GPIO_OUT		*GPIO23_00_DIR |= (1<<0)
#define GPIO_IN			*GPIO23_00_DIR |=~((1<<0))

#define GE1_TXD2_OUT	*GPIO39_24_DIR |= (0x1<<2)
#define GE1_TXD2_H		*GPIO39_24_DATA |= (0x1<<2)
#define GE1_TXD2_L		*GPIO39_24_DATA &= ~(0x1<<2)

#define POWER_KEY_IN	*GPIO71_40_DIR &= ~(0x1<<20)	//gpio#60 GE2_TXD0
#define POWER_KEY_OUT	*GPIO71_40_DIR |= (0x1<<20)
#define POWER_KEY_H		*GPIO71_40_DATA |= (0x1<<20)
#define POWER_KEY_L		*GPIO71_40_DATA &= ~(0x1<<20)

#define RSTIN_N_IN		*GPIO71_40_DIR &=~(0x1<<24)		//gpio#64 GE2_TXEN
#define RSTIN_N_OUT		*GPIO71_40_DIR |= (0x1<<24)
#define RSTIN_N_H		*GPIO71_40_DATA |= (0x1<<24)
#define RSTIN_N_L		*GPIO71_40_DATA &=~(0x1<<24)

//以下输入输出都是在主机立场说的，MSM01A作为slave
//send data
#define A2B_WAKEUP_IN	*GPIO71_40_DIR &=~(0x1<<19)		//ND_D7 gpio#59
#define A2B_WAKEUP_OUT	*GPIO71_40_DIR |=(0x1<<19)		//					作为输出 下降沿唤醒模块
#define A2B_WAKEUP_H	*GPIO71_40_DATA |=(0x1<<19)
#define A2B_WAKEUP_L	*GPIO71_40_DATA &=~(0x1<<19)

#define B2A_SLEEP_IN	*GPIO71_40_DIR &=~(0x1<<16)		//ND_D4 gpio#56 	作为输入 检测到高电平，表示模块已经睡眠
#define B2A_SLEEP_OUT	*GPIO71_40_DIR |= (0x1<<16)
#define B2A_SLEEP_H		*GPIO71_40_DATA |=(0x1<<16)
#define B2A_SLEEP_L		*GPIO71_40_DATA &=~(0x1<<16)
#define B2A_SLEEP_STAT  ((*GPIO71_40_DATA & (0x1<<16)) > 0 ? 1:0)

//recv data
#define B2A_WAKEUP_IN	*GPIO71_40_DIR &=~(0x1<<18)		//ND_D6 gpio#58 	作为输入，用于模块唤醒主机
#define B2A_WAKEUP_OUT	*GPIO71_40_DIR |= (0x1<<18)
#define B2A_WAKEUP_H	*GPIO71_40_DATA |=(0x1<<18)
#define B2A_WAKEUP_L	*GPIO71_40_DATA &=~(0x1<<18)
#define B2A_WAKEUP_STAT	((*GPIO71_40_DATA & (0x1<<18)) > 0 ? 1:0)

#define A2B_SLEEP_IN	*GPIO71_40_DIR &=~(0x1<<17)		//ND_D5 gpio#57 
#define A2B_SLEEP_OUT	*GPIO71_40_DIR |= (0x1<<17)		//					作为输出,输出低电平通知模块主机处于正常工作转态
#define A2B_SLEEP_H		*GPIO71_40_DATA |=(0x1<<17)
#define A2B_SLEEP_L		*GPIO71_40_DATA &=~(0x1<<17)

//gstar
#define ENABLE_GSTAR_OUT	*GPIO39_24_DIR |= (1<<5)	//GE1_TXCLK GPIO#29
#define ENABLE_GSTAR		*GPIO39_24_DATA &= ~(1<<5)
#define DISABLE_GSTAR		*GPIO39_24_DATA |= (1<<5)

#endif
