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
/*peripheral mode*/
#define VD5_ON		*GPIO71_40_DATA &=~(1<<19)//ND_D7 59
#define VD5_OFF		*GPIO71_40_DATA |=(1<<19)//ND_D7 59
#define USB_HUP_5V_EN	*GPIO71_40_DATA |=(1<<4)//JTAG_TRST44
#define USB_HUP_5V_disab	*GPIO71_40_DATA &=~(1<<4)//JTAG_TRST44
#define PERIOHERAL_GPIO_OUT	*GPIO71_40_DIR |= (1<<4)|(1<<19)
#define PERIOHERAL_GPIO_IN	*GPIO71_40_DIR &=~((1<<4)|(1<<19))

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
#define GPRS_POWER_L		*GPIO71_40_DATA &=~(1<<0)//JTAG_TDO 40
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
#define GPIO_IN		*GPIO23_00_DIR |=~((1<<0))

/*SC2 mode*/
#define 5V_SC2_USB_ON_OFF_OUT	*GPIO71_40_DIR |= (1<<6)	//ND_WE_N 46 
#define 5V_SC2_USB_ON_OFF_IN	*GPIO71_40_DIR &=~(1<<6)
#define 5V_SC2_USB_ON_OFF_H		*GPIO71_40_DATA |=(1<<6)
#define 5V_SC2_USB_ON_OFF_L		*GPIO71_40_DATA &=~(1<<6)

#define SC2_UP_OUT				*GPIO71_40_DIR |= (1<<8)	//ND_WP	48
#define SC2_UP_IN				*GPIO71_40_DIR &=~(1<<8)
#define SC2_UP_H				*GPIO71_40_DATA |=(1<<8)	
#define SC2_UP_L				*GPIO71_40_DATA &=~(1<<8)	

#define SC2_4.2V_ON_OFF_OUT		*GPIO71_40_DIR |= (1<<0)	//JTAG_TDO 40		
#define SC2_4.2V_ON_OFF_IN		*GPIO71_40_DIR &=~(1<<0)
#define SC2_4.2V_ON_OFF_H		*GPIO71_40_DATA |=(1<<0)
#define SC2_4.2V_ON_OFF_L		*GPIO71_40_DATA &=~(1<<0)

/*USB_HUP*/
#define USB_HUP_5V_EN_OUT		*GPIO71_40_DIR |= (1<<4)	//JTAG_TRST	44
#define USB_HUP_5V_EN_IN		*GPIO71_40_DIR &=~ (1<<4)
#define USB_HUP_5V_EN_H			*GPIO71_40_DATA |= (1<<4)
#define USB_HUP_5V_EN_L			*GPIO71_40_DATA &=~ (1<<4)

/*GPS mode*/
#define GPS_ONOFF_L				*GPIO39_24_DATA &=~(1<<8)	//GE1_RXD2 32 
#define GPS_ONOFF_H				*GPIO39_24_DATA |=(1<<8)
#define GPS_ONOFF_OUT			*GPIO39_24_DIR |= (1<<8)
#define GPS_ONOFF_IN			*GPIO39_24_DIR &=~(1<<8)
#endif
