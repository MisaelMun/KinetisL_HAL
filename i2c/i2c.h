/*
 * File:        i2c.h
 * Purpose:     I2C buffer structure information
 *
 * Notes:
 */

#ifndef __I2C_H__
#define __I2C_H__

#include "derivative.h" /* include peripheral declarations */
#include "Types.h"
#include "libs.h"
#include "sysinit.h"

/********************************************************************/
#define I2C_A1(x)                                  I2C_A1_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_F(x)                                   I2C_F_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_C1(x)                                  I2C_C1_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_S(x)                                   I2C_S_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_D(x)                                   I2C_D_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_C2(x)                                  I2C_C2_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_FLT(x)                                 I2C_FLT_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_RA(x)                                  I2C_RA_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_SMB(x)                                 I2C_SMB_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_A2(x)                                  I2C_A2_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_SLTH(x)                                I2C_SLTH_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))
#define I2C_SLTL(x)                                I2C_SLTL_REG((I2C_MemMapPtr)((uint32_t)I2C0_BASE_PTR + x*0x10))

extern CPU_tdSinit CPU_tData;
#define i2c__CLK	CPU_tData.core_clk_khz /*Include here the clk sys variable*/
#define BUS_CLOCK 	(i2c__CLK*500)

#define _i2c_delay_1bit()	   	/*Some system delay */
#define _i2c_delay_init()		/* Some system delay initialisation */



/* I2C parameters */
#define I2C_BUFFER_SIZE		(64)
#define I2C_TXRX			(2)
#define I2C_TX				(0)
#define I2C_RX				(1)
#define I2C_10BIT_TX    	(3)
#define I2C_10BIT_RX    	(4)
#define I2C_10BIT_TXRX  	(5)  

#define I2C_10BIT_ADDR_MASK     (0xF0)  

/* Function prototypes */
void i2c_init(uint8 channel, uint8 addr, uint32 bps);
void i2c_10bit_init(uint8 channel, uint16 addr, uint32 bps);
void smbus_init(uint8 channel, uint8 addr, uint8 sec_addr, uint32 bps);
void i2c_master (uint8 channel, uint8 mode, uint16 slave_address);

/* Structure for storing I2C transfer data */
typedef struct {
    int tx_index;			/* TX index */
    int rx_index;			/* RX index */
    int data_present;			/* Data present flag */
    uint16 length;			/* Length of the buffer in bytes */
    uint8 buf[I2C_BUFFER_SIZE];		/* Data buffer */
} I2C_BUFFER;

extern I2C_BUFFER i2c_tx_buffer;
extern I2C_BUFFER i2c_rx_buffer;
extern uint8 rev_flg;


#ifdef I2C__Internal

static int _i2c_set_bps(uint8 channel, uint32 bps);
static int _i2c_init_pin_mux(uint8 channel);
static void i2c_handler(uint8 channel);
static void i2csmbus_handler(uint8 channel);


#endif 




/********************************************************************/

#endif /* __I2C_H__ */
