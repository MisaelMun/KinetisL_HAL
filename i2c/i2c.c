/*
 * File:		I2C.c
 * Purpose:		I2C transfer functions and interrupt handler
 *
 * Notes:		
 *  
 */
#define I2C__Internal

#include "i2c_cfg.h"
#include "i2c.h"
#define printf put


/* Globals to be shared throughout the project */
I2C_BUFFER i2c_tx_buffer;
I2C_BUFFER i2c_rx_buffer;

/* Globals for this file only */
uint8 master_mode,iic_mode = 0;
uint8 master_tx_done;
uint8 rev_flg = 0;

const uint16 i2c_prescaler_val[] = 
{    
    20,40,80,
    22,44,88,
    24,48,96,
    26,52,108,
    28,56,112,
    30,60,120,
    34,68,136,
    40,80,160,
    28,56,112,
    32,64,128,
    36,72,144,
    40,80,160,
    44,88,176,
    48,96,192,
    56,112,224,
    68,136,272,
    48,96,192,
    56,112,224,
    64,128,256,
    72,144,288,
    80,160,320,
    88,176,352,
    104,208,416,
    128,256,512,
    80,160,320,
    96,192,384,
    112,224,448,
    128,256,512,
    144,288,576,
    160,320,640,
    192,384,768,
    240,480,960,
    160,320,640,
    192,384,768,
    224,448,896,
    256,512,1024,
    288,576,1152,
    320,640,1280,
    384,768,1536,
    480,960,1920,
    320,640,1280,
    384,768,1536,
    448,896,1792,
    512,1024,2048,
    576,1152,2304,
    640,1280,2560,
    768,1536,3072,
    960,1920,3840,
    640,1280,2560,
    768,1536,3072,
    896,1792,3584,
    1024,2048,4096,
    1152,2304,4608,
    1280,2560,5120,
    1536,3072,6144,
    1920,3840,7680,
    1280,2560,5120,
    1536,3072,6144,
    1792,3584,7168,
    2048,4096,8192,
    2304,4608,9216,
    2560,5120,10240,
    3072,6144,12288,
    3840,7680,15360,
};




/*
Function i2c_set_bps
Description : set baud of i2c module
*/
static int _i2c_set_bps(uint8 channel, uint32 bps)
{
  uint8 x;
  uint8 best_ndx=(uint8)-1u;
  uint8 best_mul=(uint8)-1u;
  uint16 e=(uint16)-1u;
  uint32 d=(BUS_CLOCK)/bps;
  
  for(x=0; x<sizeof(i2c_prescaler_val)/sizeof(i2c_prescaler_val[0]); x++)
  {
    uint16 e1;
    if (d>i2c_prescaler_val[x])
    {
      continue;
    }
    
    e1=(unsigned short)(i2c_prescaler_val[x]-d);
    if ((e1<e) || (e1==0))
    {
      e=e1;
      best_ndx=x/3;
      best_mul=x%3;
    }
  }

  if (best_ndx == (uint8)-1u)
  {
    return(-1);
  } 
  I2C_F(channel) = (best_mul<<I2C_F_MULT_SHIFT) + best_ndx;
  return(0);
}

static int _i2c_init_pin_mux(uint8 channel)
{
	/* Configure GPIO pins to I2C function */
	switch(channel)
	{
		case 0:
			I2C__SCL0_PCR = PORT_PCR_MUX(I2C__SCL0_MUX) | PORT_PCR_DSE_MASK; 
			I2C__SDA0_PCR = PORT_PCR_MUX(I2C__SDA0_MUX) | PORT_PCR_DSE_MASK; 		
			break;
		case 1:
			I2C__SCL1_PCR = PORT_PCR_MUX(I2C__SCL1_MUX) | PORT_PCR_DSE_MASK; 
			I2C__SDA1_PCR = PORT_PCR_MUX(I2C__SDA1_MUX) | PORT_PCR_DSE_MASK; 	
			break;
		default:
			return -1;	
	}
	return 0;	
}

void i2c_init(uint8 channel, uint8 addr, uint32 bps)
{	
	if(_i2c_init_pin_mux(channel)==-1)
	{
		printf("***Fail in i2c pin mux config***\r\n");
		return;	
	}
	if(-1==_i2c_set_bps(channel, bps))
	{
		printf("***Fail in i2c set bps***\r\n");
		return;	
	}

	I2C_C2(channel) = 0x0;  //general call disabled, 7-bit address
        
#if POLLING_MODE
	I2C_C1(channel) = I2C_C1_IICEN_MASK | 0;
#else	
	/* start the module and enable interrupt */
	I2C_C1(channel) = I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK | 0;
#endif	
	/* set slave address */
	I2C_A1(channel) = addr << 1;
	
	/* initial RAM variable */
	iic_mode = 0;
}

void i2c_10bit_init(uint8 channel, uint16 addr, uint32 bps)
{
	if(_i2c_init_pin_mux(channel)==-1)
	{
		return;	
	}
	if(-1==_i2c_set_bps(channel, bps))
	{
		return;	
	}
    
	I2C_C2(channel) = 0x0|I2C_C2_ADEXT_MASK|(uint8)((addr&0x0380)>>7);  //general call disabled, 10-bit address
        
	/* start the module and enable interrupt */
	I2C_C1(channel) = I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK | 0;
	
	/* set slave address */
	I2C_A1(channel) = (uint8)(addr << 1);
}

void smbus_init(uint8 channel, uint8 addr, uint8 sec_addr, uint32 bps)
{	
	if(_i2c_init_pin_mux(channel)==-1)
	{
		return;	
	}
	if(-1==_i2c_set_bps(channel, bps))
	{
		return;	
	}
    
	I2C_C2(channel) = 0x0;  //general call disabled, 7-bit address
    
	I2C_SMB(channel) = 0
//					  |I2C_SMB_ALERTEN_MASK
//					  |I2C_SMB_SIICAEN_MASK
					  |I2C_SMB_FACK_MASK;    
	/* start the module and enable interrupt */
	I2C_C1(channel) = I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK | 0;
	
	I2C_A1(channel) = addr << 1;
	
	/* set smbus slave address */
	I2C_A2(channel) = sec_addr << 1;

	/* initial RAM variable */
	iic_mode = 0;
	
	/* Init MTIM for delay */
	_i2c_delay_init();
}


/********************************************************************

**********/
/*	Purpose:  	General function for performing I2C master transfers.  Capable
 *				of performing Master TX, Master RX, and Master TX/RX w/ Repeated
 *				Start.
 *				
 *	Arguments:	mode - Valid modes include I2C_TX, I2C_RX, and I2C_TXRX 
 *					(all modes defined in i2c.h)
 *				slave_address - The slave address of the I2C module that needs
 *					to be dealt with.  
 */
void i2c_master (uint8 channel, uint8 mode, uint16 slave_address)
{
    
#if POLLING_MODE		
	int i;
	uint8 dummy_read;
	
	/* Clear IBCR.IBDIS, disable interrupts */
	I2C_C1(channel) = 0x80;	
	
	/* Reset index for TX and RX buffers */
	i2c_tx_buffer.tx_index = 0;
	i2c_rx_buffer.rx_index = 0;
	
	if (mode == I2C_TXRX)
	{
		/* Make sure bus is idle */
		while (I2C_S(channel) & I2C_S_BUSY_MASK);
		/* Put module in master TX mode (generates START) */
		I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
		/* Put target address into IBDR */
		I2C_D(channel) = ( 0 | (slave_address<<1) | I2C_TX);
		/* Wait for I2SR[IBB] (bus busy) to be set */
		while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
		
		/* Wait for address transfer to complete */
		while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
		I2C_S(channel) |= I2C_S_IICIF_MASK;
		
		/* Send the contents of the TX buffer */
		while (i2c_tx_buffer.length > 0)
		{
			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
			i2c_tx_buffer.length--;
			/* Wait for transfer to complete */
			while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
			I2C_S(channel) |= I2C_S_IICIF_MASK;
		}
	
		/* Wait for a bit */
		i=10000;
		while(i--);
		
		/* Set IBCR.RSTA and put in master RX mode */
		I2C_C1(channel) |= (0 | I2C_C1_RSTA_MASK);
		/* Put target address into IBDR and set the R/!W bit */
		I2C_D(channel) = (0 | slave_address | I2C_RX);
		
		/* Wait for address transfer to complete */
		while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
		I2C_S(channel) |= I2C_S_IICIF_MASK;
		
		/* Clear TX/RX bit in order to set receive mode */
		I2C_C1(channel) &= ~I2C_C1_TX_MASK; 
		
		/* Dummy read of IBDR to signal the module is ready for the next byte */
		dummy_read = I2C_D(channel);
		
		/* Receive data from slave */
		while (i2c_rx_buffer.length > 0) 
		{
			/* Wait for transfer to complete */
			while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
			I2C_S(channel) |= I2C_S_IICIF_MASK;
			/* Check for second-to-last and last byte transmission.  After second-to-last
			   byte is received, it is required to disable the ACK bit in order to signal
			   to the slave that the last byte has been received.  The actual NAck does not
			   take place until after the last byte has been received. */
			if (i2c_rx_buffer.length == 2) 
			{
				/* Disable Acknowledge, generate STOP after next byte transfer */
				I2C_C1(channel) |= I2C_C1_TXAK_MASK;
			}
			if(i2c_rx_buffer.length == 1) 
			{
				/* Generate STOP */
				I2C_C1(channel) &= ~I2C_C1_MST_MASK;
			}
			
			/* Store received data in RX buffer */
			i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
			i2c_rx_buffer.length--;
		}
	
		/* Restore module to it's idle (but active) state */
		I2C_C1(channel) = 0x80;
		
		return;
	
	}
	else if (mode == I2C_TX)
	{
		/* Make sure bus is idle */
		while (I2C_S(channel) & I2C_S_BUSY_MASK);
		/* Put module in master TX mode (generates START) */
		I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
		/* Put target address into IBDR */
		I2C_D(channel) = ( 0 | (slave_address<<1) | I2C_TX);
		/* Wait for I2SR[IBB] (bus busy) to be set */
		while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
		
		/* Wait for address transfer to complete */
		while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
		I2C_S(channel) |= I2C_S_IICIF_MASK;
		
		/* Send the contents of the TX buffer */
		while (i2c_tx_buffer.length > 0)
		{
			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
			i2c_tx_buffer.length--;
			/* Wait for transfer to complete */
			while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
			I2C_S(channel) |= I2C_S_IICIF_MASK;
		}
		
		/* Restore module to it's idle (but active) state */
		I2C_C1(channel) = 0x80;
		
		return;
	}
	else if (mode == I2C_RX)
	{
		/* Make sure bus is idle */
		while (I2C_S(channel) & I2C_S_BUSY_MASK);
		/* Put module in master TX mode (generates START) */
		I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
		/* Put target address into IBDR */
		I2C_D(channel) = ( 0 | (slave_address<<1) | I2C_RX);
		/* Wait for I2SR[IBB] (bus busy) to be set */
		while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
		
		/* Wait for address transfer to complete */
		while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
		I2C_S(channel) |= I2C_S_IICIF_MASK;
		
		/* Clear TX/RX bit in order to set receive mode */
		I2C_C1(channel) &= ~I2C_C1_TX_MASK; 
		
		/* Dummy read of IBDR to signal the module is ready for the next byte */
		dummy_read = I2C_D(channel);
		
		/* Receive data from slave */
		while (i2c_rx_buffer.length > 0) 
		{
			/* Wait for transfer to complete */
			while (!(I2C_S(channel) & I2C_S_IICIF_MASK));
			I2C_S(channel) |= I2C_S_IICIF_MASK;
			/* Check for second-to-last and last byte transmission.  After second-to-last
			   byte is received, it is required to disable the ACK bit in order to signal
			   to the slave that the last byte has been received.  The actual NAck does not
			   take place until after the last byte has been received. */
			if (i2c_rx_buffer.length == 2) 
			{
				/* Disable Acknowledge, generate STOP after next byte transfer */
				I2C_C1(channel) |= I2C_C1_TXAK_MASK;
			}
			if(i2c_rx_buffer.length == 1) 
			{
				/* Generate STOP */
				I2C_C1(channel) &= ~I2C_C1_MST_MASK;
			}
			
			/* Store received data in RX buffer */
			i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
			i2c_rx_buffer.length--;
		}
		
		/* Restore module to it's idle (but active) state */
		I2C_C1(channel) = 0x80;
		
		return;		
	}
	else
	{
  }  
    
#else	/* Interrrupt-Driven */
    
	master_mode = mode;
	if(master_mode>I2C_TXRX)
	    master_mode -= I2C_10BIT_TX;//10 bit address mode  
	master_tx_done = FALSE;
	
	/* Reset index for TX and RX buffers */
	i2c_tx_buffer.tx_index = 0;
	i2c_rx_buffer.rx_index = 0;
    
    /* Transmit followed by receive using RSTA */
    if (mode == I2C_TXRX)
    {
    	/* Make sure bus is idle */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	/* Put module in master TX mode (generates START) */
    	I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    	/* Put target address into IBDR */
    	I2C_D(channel) = ( 0 | (slave_address << 1) | I2C_TX);
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));

    	/* Wait for TX to finish before starting RX */
    	while (!master_tx_done);
    	
    	master_mode = I2C_RX;
    	/* Set IBCR.RSTA and put in master RX mode */
    	I2C_C1(channel) |= (0 | I2C_C1_RSTA_MASK);
    	/* Put target address into IBDR and set the R/!W bit */
    	I2C_D(channel) = (0 | (slave_address << 1) | I2C_RX);
    	
    	/* Wait for bus to become free before continuing */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	
    	/* Restore module to it's idle (but active) state */
    	I2C_C1(channel) = 0xC0;
    	
    	return;
    }
    /* Single TX or RX */
    else if ( (mode == I2C_TX) || (mode == I2C_RX) )
    {
    	/* Make sure bus is idle */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	/* Put module in master TX mode (generates START) */
    	I2C_C1(channel) |= (0 | I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    	/* Put target address into IBDR */
    	I2C_D(channel) = ( 0 | (slave_address << 1) | mode);
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
    	
    	/* Wait for bus to become free before continuing */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);

    	/* Restore module to it's idle (but active) state */
    	I2C_C1(channel) = 0xC0;
    	
    	return;
    }
#if 0
    /* 10bit address Transmit followed by receive using RSTA */
    else if (mode == I2C_10BIT_TXRX)
    {
    	
    	/* Make sure bus is idle */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	/* Put module in master TX mode (generates START) */
    	I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    	/* put 0b11110+ad10+ad9 into IBDR */
    	I2C_D(channel) = ( I2C_10BIT_ADDR_MASK | ((uint8)((slave_address&0x0300)>>7)) | I2C_TX);
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));    	
    	/* Put target address into IBDR */
    	I2C_D(channel) = ( 0 | (slave_address&0x0FF));
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));

    	/* Wait for TX to finish before starting RX */
    	while (!master_tx_done);
    	
    	master_mode = I2C_RX;
    	/* Set IBCR.RSTA and put in master RX mode */
    	I2C_C1(channel) |= (0 | I2C_C1_RSTA_MASK);
    	/* Put target address into IBDR and set the R/!W bit */
    	I2C_D(channel) = (0 | (slave_address << 1) | I2C_RX);
    	
    	/* Wait for bus to become free before continuing */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	
    	/* Restore module to it's idle (but active) state */
    	I2C_C1(channel) = 0xC0;
    	
    	return;
    }
#endif
    /* 10bit address Single TX */
    else if (mode == I2C_10BIT_TX)
    {
    	//int i;
    	/* Make sure bus is idle */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	/* Put module in master TX mode (generates START) */
    	I2C_C1(channel) |= (0 | I2C_C1_MST_MASK | I2C_C1_TX_MASK);

    	    
     	/* put 0b11110+ad10+ad9 into IBDR */
    	I2C_D(channel) = ( I2C_10BIT_ADDR_MASK | ((uint8)((slave_address&0x0300)>>7)) | I2C_TX);   	
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK)); 
    	/* Wait for bus to become free before continuing */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);

    	/* Restore module to it's idle (but active) state */
    	I2C_C1(channel) = 0xC0;
    	
    	return;
    }
    // 10bit address sigle RX
    else if(mode == I2C_10BIT_RX)
    {
    	/* need to transfer ext address byte */
    	/* 1st byte to be transfer should be slave address */
    	master_mode = I2C_TXRX;
    	/* Make sure bus is idle */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	/* Put module in master TX mode (generates START) */
    	I2C_C1(channel) |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    	/* put 0b11110+ad10+ad9 into IBDR */
    	I2C_D(channel) = ( I2C_10BIT_ADDR_MASK | ((uint8)((slave_address&0x0300)>>7)) | I2C_TX);
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
    	/* Wait for bus to become free before continuing */
    	while (!master_tx_done);
    
        master_mode = I2C_RX;
    	/* Set IBCR.RSTA and put in master RX mode */
    	I2C_C1(channel) |= (0 | I2C_C1_RSTA_MASK);
    	/* Put target address into IBDR and set the R/!W bit */
    	I2C_D(channel) = (0 | I2C_10BIT_ADDR_MASK | ((uint8)((slave_address&0x0300)>>7))| I2C_RX);
    	/* Wait for I2SR[IBB] (bus busy) to be set */
    	while (!(I2C_S(channel) & I2C_S_BUSY_MASK));
    	/* Wait for bus to become free before continuing */
    	while (I2C_S(channel) & I2C_S_BUSY_MASK);
    	
    	/* Restore module to it's idle (but active) state */
    	I2C_C1(channel) = 0xC0;
    	
    	return;
    }
    else{
	  /*do nothing*/
    }
#endif
}

void I2C0_IRQHandler(void)
{
	i2csmbus_handler(0);
}

void I2C1_IRQHandler(void)
{
	i2csmbus_handler(1);
}


/******************************************************************************/
/*	Purpose:  	General I2C handler, created using the flowchart example 
 *				included in the I2C chapter of the UM.  
 */
static void i2c_handler(uint8 channel)
{
    /* Temp variable for dummy reads */
    uint8 dummy_read;		
	
    /* Clear the I2C Interrupt Flag. */
    I2C_S(channel) |= I2C_S_IICIF_MASK;
    
    /* Check if this device is in Master or Slave Mode. */
    if (I2C_C1(channel) & I2C_C1_MST_MASK)
    {
      	/* Master Mode - Check if this device is in Transmit or Receive Mode. */
      	if (I2C_C1(channel) & I2C_C1_TX_MASK)
      	{
      	    /* Master Transmit Mode - Check if last byte was tranmitted. */
      	    if ((i2c_tx_buffer.length == 0) && (master_mode != I2C_RX))
      	    {
          		/* Last byte was transmitted - 
          		   Generate Stop signal by changing to Slave Mode. */

          		/* If TXRX mode (Repeated Start), signal end of TX */
          		if (master_mode == I2C_TXRX)
          		    master_tx_done = TRUE;
          		/* Issue STOP */
          		else
          		    I2C_C1(channel) &= ~I2C_C1_MST_MASK;
          		              
      	    }
      	    else
      	    {
          		/* More bytes to be transmitted - Check if ACK received. */
          		if (I2C_S(channel) & I2C_S_RXAK_MASK)
          		{
          		    /* ACK not received - Generate STOP */
          		    I2C_C1(channel) &= ~I2C_C1_MST_MASK;
          		    
          		}
          		else
          		{
          		    /* Check if end of address cycle */
          		    if (master_mode == I2C_RX)
          		    {
						I2C_C1(channel) &= ~I2C_C1_TX_MASK;
					
						/* there is only 1 byte to be received */
						if (i2c_rx_buffer.length == 1)
						{
							I2C_C1(channel) |= I2C_C1_TXAK_MASK;
						}
						/*ToDo: Change dummy_read for a void reading */
						dummy_read = I2C_D(channel);
					
					   #if DEBUG
						  printf("Master TX mode: End of RX address cycle, switch to RX mode.\n");
					   #endif
          		    }
          		    /* ACK received, send data */
          		    else
          		    {
          		    	/* Not end of address cycle - Write next byte to MBDR */
              			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
              			i2c_tx_buffer.length--;

						#if DEBUG
							printf("Master TX mode: Send Byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
						#endif
          		    }
          		}
      	    } 
      	}
      	else
      	{
      	    /* Master Receive Mode - Check if this is last byte to be read. */	    
      	    if (i2c_rx_buffer.length == 1)
      	    {
          		#if DEBUG
          			printf("I2CR TXAK = 0x%02X\n",I2C_C1(channel));
          		#endif
          		/* Last byte to be read - 
          		   Generate Stop signal by changing to Slave Mode. */
          		I2C_C1(channel) &= ~I2C_C1_MST_MASK;

          		#if DEBUG
          		     printf("Master RX mode: All data received, send STOP.\n");
                  #endif
      	    }
      	    else
      	    {
          		/* Not last byte to be read - Check if second to last byte. */
          		if (i2c_rx_buffer.length == 2)
          		{
          		    #if DEBUG
          		    	    printf("I2CR S2L  = 0x%02X\n",I2C_C1(channel));
          		    #endif
          		    /* Second to last byte to be read - Set Transmit Acknowledge Enable
          		       bit so no ACK is sent after the next byte is received, which
          		       indicates "end of data" to the slave. */
          		    I2C_C1(channel) |= I2C_C1_TXAK_MASK;
          		    
          		    #if DEBUG
          		         printf("Master RX mode: Second-to-last byte received, set TXAK.\n");
                      #endif
          		}
      	    }

      	    /* Store received data in RX buffer */
      	    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
      	    i2c_rx_buffer.length--;

      	    #if DEBUG
      	         printf("Master RX mode: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
             #endif
      	}
    }
    else
    {
      	/* Slave Mode - Check if Arbitration Lost. */
      	if (I2C_S(channel) & I2C_S_ARBL_MASK)
      	{
      	    #if DEBUG
      	         printf("Arbitration Lost.\n");
             #endif

      	    /* Clear IAL bit */
      	    I2C_S(channel) &= ~I2C_S_ARBL_MASK;
      	    
      	    /* Arbitration Lost - Check if this device is being addressed as slave.
      	       (If not, nothing more needs to be done.) */
      	    if (I2C_S(channel) & I2C_S_IAAS_MASK)
      	    {
          		/* Addressed as slave - 
          		   Check if master was reading from slave or writing to slave. */
          		if (I2C_S(channel) & I2C_S_SRW_MASK)
          		{
          		    /* Set tx_index to 0 */
          		    i2c_tx_buffer.tx_index = 0;
          		    
          		    /* Master was reading from slave - Set Transmit Mode. */
          		    I2C_C1(channel) |= I2C_C1_TX_MASK;
          		    
          		    /* Write data to MBDR. */
          		    I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];

          		    #if DEBUG
          		         printf("Arbitration Lost: Addressed as slave - TX mode.\n");
                      #endif
          		}
          		else
          		{
          		    /* Set rx_index to 0 */
          		    i2c_rx_buffer.rx_index = 0;
          		    
          		    /* Master was writing to slave - Set Receive Mode. */
          		    I2C_C1(channel) &= ~I2C_C1_TX_MASK;
          		    
          		    /* Dummy read from MBDR, to clear the ICF bit. */
          		    dummy_read = I2C_D(channel);
          		    
          		    #if DEBUG
          		         printf("Arbitration Lost: Addressed as slave - RX mode.\n");
                      #endif
          		}
      	    }	    
      	}
      	else
      	{
      	    /* Arbitration Not Lost - Check if data byte is this devices's Slave Address byte. */
      	    if (I2C_S(channel) & I2C_S_IAAS_MASK)
      	    {
            		/* Data byte is Slave Address byte - Check Slave Read/Write bit. */
            		if (I2C_S(channel) & I2C_S_SRW_MASK)
            		{
            		    /* Set tx_index to 0 */
            		    i2c_tx_buffer.tx_index = 0;
            		    
            		    /* Master was reading from slave - Set Transmit Mode. */
            		    I2C_C1(channel) |= I2C_C1_TX_MASK;

            		    /* Write data to MBDR. */
            		    I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];

            		    #if DEBUG
            		         printf("Slave TX: First byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
                      #endif
            		}
            		else
            		{
            		    /* Master has specified Slave Receive Mode.
            		       Set Receive Mode.  (Writing to MBCR clears IAAS.) */
            		    
            		    /* Set rx_index to 0 */
            		    i2c_rx_buffer.rx_index = 0;
						#if DEBUG
							 printf("Slave RX: I2C_S = 0x%02x.\r\n", I2C_S(channel));
						#endif 
            		    I2C_C1(channel) &= ~I2C_C1_TX_MASK;
            		    
            		    /* Read address data from MBDR and store it. */
            		    dummy_read = I2C_D(channel);

            		    #if DEBUG
            		         printf("Slave RX: Receive address.\r\n");
                        #endif
            		}
      	    }
      	    else
      	    {
          		/* Data byte received is not Slave Address byte - 
          		   Check if this device is in Transmit or Receive Mode. */
          		if (I2C_C1(channel) & I2C_C1_TX_MASK)
          		{
          		    
          		    /* Last byte received? */
          		    if (I2C_S(channel) & I2C_S_RXAK_MASK)
          		    {
              			I2C_C1(channel) &= ~I2C_C1_TX_MASK;
              			dummy_read = I2C_D(channel);

              			#if DEBUG
              			     printf("Slave TX: Last byte has been sent.\n");
                          #endif
          		    }
          		    else
          		    {
              			/* Write data to MBDR. */
              			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
              			i2c_tx_buffer.length--;

              			#if DEBUG
              			     printf("Slave TX: Send byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
                          #endif
          		    }
          		}
          		else
          		{
          		    /* Receive Mode - Read data from MBDR and store it. */
          		    if( (I2C_C2(channel)&I2C_C2_ADEXT_MASK) && ((I2C_D(channel)&0xF8)==I2C_10BIT_ADDR_MASK))
          		    {
          		        
           		        #if DEBUG
          		        printf("Slave RX: extend address recieved\r\n");
                        #endif
          		    }
          		    else
          		    {
           				if(i2c_rx_buffer.rx_index + 2 == i2c_rx_buffer.length)
            			{// receiving second-to-last byte?
           					// NACK for last byte
                  		    I2C_C1(channel) |= I2C_C1_TXAK_MASK;
            			}
           				
              		    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
              		    i2c_rx_buffer.data_present = TRUE;
              		    if(i2c_rx_buffer.rx_index == i2c_rx_buffer.length)
              		    {// for next transfer
              		    	I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
              		    }
              		    #if DEBUG
              		         printf("Slave RX: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
                        #endif
          		    }
          		}
      	    }
      	}
    }
}

static void i2csmbus_handler(uint8 channel)
{
    /* low time out? */
    if((I2C_SMB(channel) & I2C_SMB_SLTF_MASK) == 0)
    {
        if(I2C_SMB(channel) & I2C_SMB_FACK_MASK)
        {
            /* Temp variable for dummy reads */
            uint8 dummy_read; 
            /* Check if this device is in Master or Slave Mode. */
            if (I2C_C1(channel) & I2C_C1_MST_MASK)
            {
              	/* Master Mode - Check if this device is in Transmit or Receive Mode. */
              	if (I2C_C1(channel) & I2C_C1_TX_MASK)
              	{
              	    /* Master Transmit Mode - Check if last byte was tranmitted. */
              	    if ((i2c_tx_buffer.length == 0) && (master_mode != I2C_RX))
              	    {
              		/* Last byte was transmitted - 
              		   Generate Stop signal by changing to Slave Mode. */

                  		/* If TXRX mode (Repeated Start), signal end of TX */
                  		if (master_mode == I2C_TXRX)
                  		    master_tx_done = TRUE;
                  		/* Issue STOP */
                  		else
                  		    I2C_C1(channel) &= ~I2C_C1_MST_MASK;
              		
                         #if DEBUG
              		        printf("Master TX mode: Last Byte\n");
                         #endif
              	    }
              	    else
              	    {
                  		/* More bytes to be transmitted - Check if ACK received. */
                  		if (I2C_S(channel) & I2C_S_RXAK_MASK)
                  		{
                  		    /* ACK not received - Generate STOP */
                  		    I2C_C1(channel) &= ~I2C_C1_MST_MASK;
                  		    
                              #if DEBUG
                  		         printf("Master TX mode: NAK\n");
                              #endif
                  		}
                  		else
                  		{
                  		    /* Check if end of address cycle */
                  		    if (master_mode == I2C_RX)
                  		    {
                      			/* Clear the I2C Interrupt Flag. */
                                I2C_S(channel) |= I2C_S_IICIF_MASK; 
                                
                                /* switch to RX mode */
                      			I2C_C1(channel) &= ~I2C_C1_TX_MASK;

        						/* there is only 1 byte to be received */
        						if (i2c_rx_buffer.length == 1)
        						{
        							I2C_C1(channel) |= I2C_C1_TXAK_MASK;
        						}
                      			
                      			dummy_read = I2C_D(channel);
                          	    
								#if DEBUG
									printf("Master TX mode: End of RX address cycle, switch to RX mode.\n");
								#endif
                  		    }
                  		    /* ACK received, send data */
                  		    else
                  		    {
                      			/* Clear the I2C Interrupt Flag. */
                                I2C_S(channel) |= I2C_S_IICIF_MASK; 
                                
                    			/* Not end of address cycle - Write next byte to MBDR */
                      			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
                      			i2c_tx_buffer.length--;

                                if(i2c_tx_buffer.length == 1)
                                {// last byte left
                        		    I2C_SMB(channel) &= ~I2C_SMB_FACK_MASK; 
                                }
                                #if DEBUG
									printf("Master TX mode: Send Byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
                                #endif
                  		    }
                  		}
              	    } 
              	}
              	else
              	{
              	    /* Master Receive Mode - Check if this is last byte to be read. */	    
              	    if (i2c_rx_buffer.length == 1)
              	    {             		
                  		/* Clear the I2C Interrupt Flag. */
                        I2C_S(channel) |= I2C_S_IICIF_MASK; 
              	    	
              	    	#if DEBUG
                  			printf("I2CR TXAK = 0x%02X\n",I2C_C1(channel));
                  		#endif
                  		/* Last byte to be read - 
                  		   Generate Stop signal by changing to Slave Mode. */
                  		I2C_C1(channel) &= ~I2C_C1_MST_MASK;
                  		
                  		/* delay 1bit scl */
                  	    _i2c_delay_1bit();
                  	    /* Store received data in RX buffer */
                  	    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
                  	    i2c_rx_buffer.length--;

                  	    #if DEBUG
                  	         printf("Master RX mode: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
                  		     printf("Master RX mode: All data received, send STOP.\n");
                        #endif
                  		
                  		/* disable interrupt */
                  		I2C_C1(channel) &= ~I2C_C1_IICIE_MASK;
                  		return;
              	    }
              	    else
              	    {
              	    	/* Not last byte to be read - Check if second to last byte. */
                  		if (i2c_rx_buffer.length == 2)
                  		{
                  		    /* Second to last byte to be read - Set Transmit Acknowledge Enable
                  		       bit so no ACK is sent after the next byte is received, which
                  		       indicates "end of data" to the slave. */            			
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                      	    /* Store received data in RX buffer */
                      	    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
                      	    i2c_rx_buffer.length--;
                      	    
                      	    /* ACK current byte */
                      	    I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;

                      		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK; 
                            
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			dummy_read = I2C_D(channel);

                			I2C_C1(channel) |= I2C_C1_TXAK_MASK;
                  		    I2C_SMB(channel) &= ~I2C_SMB_FACK_MASK;
                  		    
                  		    #if DEBUG
                  		         printf("Master RX mode: Second-to-last byte received, set TXAK--0x%02x.\r\n", I2C_C1(channel));
                  		         printf("Master RX mode: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
                            #endif
                  		         
                  		    return;
                  		}
                  		else
                  		{/* set TXAK to proper value: delay about 1-2bit scl cycle waiting data
                  		    register updated. */
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);
                			_i2c_delay_1bit();                  		    
                  			
                  			I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
                  		}
                  		/* Clear the I2C Interrupt Flag. */
                        I2C_S(channel) |= I2C_S_IICIF_MASK; 
                        
               	    }

              	    _i2c_delay_1bit();
              	    /* Store received data in RX buffer */
              	    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
              	    i2c_rx_buffer.length--;

              	    #if DEBUG
              	         printf("Master RX mode: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
                    #endif
              	}
            }
            else
            {
              	/* Slave Mode - Check if Arbitration Lost. */
              	if (I2C_S(channel) & I2C_S_ARBL_MASK)
              	{
              	    #if DEBUG
              	         printf("Arbitration Lost.\n");
                     #endif

              	    /* Clear IAL bit */
              	    I2C_S(channel) &= ~I2C_S_ARBL_MASK;
              	    
              	    /* Arbitration Lost - Check if this device is being addressed as slave.
              	       (If not, nothing more needs to be done.) */
              	    if (I2C_S(channel) & I2C_S_IAAS_MASK)
              	    {
                  		/* Addressed as slave - 
                  		   Check if master was reading from slave or writing to slave. */
                  		if (I2C_S(channel) & I2C_S_SRW_MASK)
                  		{
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);
                			_i2c_delay_1bit();
                			
                			/* set TXAK to proper value */
                			I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
                      		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK;  
                            /* delay 1 scl */
                            _i2c_delay_1bit();
                            
                  		    /* Set tx_index to 0 */
                  		    i2c_tx_buffer.tx_index = 0;
                  		    
                  		    /* Master was reading from slave - Set Transmit Mode. */
                  		    I2C_C1(channel) |= I2C_C1_TX_MASK;
                  		    
                  		    /* Write data to MBDR. */
                  		    I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];

                  		    #if DEBUG
                  		         printf("Arbitration Lost: Addressed as slave - TX mode.\n");
                              #endif
                  		}
                  		else
                  		{
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);
                			_i2c_delay_1bit();
                			
                			/* set TXAK to proper value */
                			I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
                      		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK;  
                            /* delay 1 scl */
                            _i2c_delay_1bit();
                            
                  		    /* Set rx_index to 0 */
                  		    i2c_rx_buffer.rx_index = 0;
                  		    
                  		    /* Master was writing to slave - Set Receive Mode. */
                  		    I2C_C1(channel) &= ~I2C_C1_TX_MASK;
                  		    
                  		    /* Dummy read from MBDR, to clear the ICF bit. */
                  		    dummy_read = I2C_D(channel);
                  		    
                  		    /* delay 1 scl */
                  		    _i2c_delay_1bit();
                  		    
                  		    #if DEBUG
                  		         printf("Arbitration Lost: Addressed as slave - RX mode.\n");
                              #endif
                  		}
              	    }	    
              	}
              	else
              	{
              	    /* Arbitration Not Lost - Check if data byte is this devices's Slave Address byte. */
              	    if (I2C_S(channel) & I2C_S_IAAS_MASK)
              	    {
                		/* Data byte is Slave Address byte - Check Slave Read/Write bit. */
                		if (I2C_S(channel) & I2C_S_SRW_MASK)
                		{
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);
                			_i2c_delay_1bit();
                			
                			/* set TXAK to proper value */
                			I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
                      		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK;  
                            /* delay 1 scl */
                            _i2c_delay_1bit();
                            
                			/* Set tx_index to 0 */
                		    i2c_tx_buffer.tx_index = 0;
                		    
                		    /* Master was reading from slave - Set Transmit Mode. */
                		    I2C_C1(channel) |= I2C_C1_TX_MASK;

                		    /* Write data to MBDR. */
                		    I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];

                		    #if DEBUG
                		         printf("Slave TX: First byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
                          #endif
                		}
                		else
                		{
                		    /* Master has specified Slave Receive Mode.
                		       Set Receive Mode.  (Writing to MBCR clears IAAS.) */
                		    I2C_C1(channel) &= ~I2C_C1_TX_MASK;
                		    
                			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);
                			_i2c_delay_1bit();
                			
                			/* set TXAK to proper value */
                			I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;
                      		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK;  
                            /* delay 1 scl */
                            _i2c_delay_1bit();
                            
                		    /* Set rx_index to 0 */
                		    i2c_rx_buffer.rx_index = 0;
                		    
                		    /* Read address data from MBDR and store it. */
                		    dummy_read = I2C_D(channel);
//                		    _i2c_delay_1bit();

                		    #if DEBUG
                		         printf("Slave RX: Receive address.\n");
							#endif
                		}
              	    }
              	    else
              	    {
                  		/* Data byte received is not Slave Address byte - 
                  		   Check if this device is in Transmit or Receive Mode. */
                  		if (I2C_C1(channel) & I2C_C1_TX_MASK)
                  		{
                  		    
                  		    /* Last byte received? */
                  		    if (I2C_S(channel) & I2C_S_RXAK_MASK)
                  		    {
                      			I2C_C1(channel) &= ~I2C_C1_TX_MASK;
                      			dummy_read = I2C_D(channel);

                      			#if DEBUG
                      			     printf("Slave TX: Last byte has been sent.\n");
                                #endif
                  		    }
                  		    else
                  		    {
                                /* Clear the I2C Interrupt Flag. */
                                I2C_S(channel) |= I2C_S_IICIF_MASK; 
                                   
                      			/* Write data to MBDR. */
                      			I2C_D(channel) = i2c_tx_buffer.buf[i2c_tx_buffer.tx_index++];
                      			i2c_tx_buffer.length--;

                      			#if DEBUG
                      			     printf("Slave TX: Send byte - 0x%02X\n",i2c_tx_buffer.buf[i2c_tx_buffer.tx_index-1]);
                                  #endif
                  		    }
                  		}
                  		else
                  		{
                  		    /* set TXAK to proper value, delay about 1-2bit scl cycle waiting 
                  		       data register updated */
                  			/* delay 1 scl */
                			_i2c_delay_1bit();
                			/* read data... */
                			dummy_read = I2C_D(channel);  
               				_i2c_delay_1bit();
 			
               				/* ACK for current byte */
               				I2C_C1(channel) &= ~I2C_C1_TXAK_MASK;	
               				
               				if(i2c_rx_buffer.rx_index + 2 == i2c_rx_buffer.length)
                			{// receiving second-to-last byte?
                      			/* must delay 1 scl here */
                    			_i2c_delay_1bit();               					
                    			// disable FACK
                    			if(I2C_SMB(channel)&I2C_SMB_SIICAEN_MASK)
                    			{// default address for arp protocol
                    				I2C_SMB(channel) &= ~I2C_SMB_FACK_MASK;
                    			}
                    			else
                    			{
                    				I2C_SMB(channel) &= ~(I2C_SMB_FACK_MASK|I2C_SMB_SIICAEN_MASK);	
                    			}               					
                    			// NACK for last byte
                      		    I2C_C1(channel) |= I2C_C1_TXAK_MASK;
								#if DEBUG
									printf("SMB RX: C1=0x%02x, SMB=0x%02x\r\n", 
											I2C_C1(channel),
											I2C_SMB(channel));
								#endif
                			}
             			
                     		/* Clear the I2C Interrupt Flag. */
                            I2C_S(channel) |= I2C_S_IICIF_MASK;  
                            /* delay 1 scl */
                            _i2c_delay_1bit();                		       
                  		       
                  		    /* Receive Mode - Read data from MBDR and store it. */
                  		    i2c_rx_buffer.buf[i2c_rx_buffer.rx_index++] = I2C_D(channel);
                  		    i2c_rx_buffer.data_present = TRUE;

                  		    #if DEBUG
                  		         printf("Slave RX: Receive byte - 0x%02X\n",i2c_rx_buffer.buf[i2c_rx_buffer.rx_index-1]);
                            #endif
                  		}
              	    }
              	}
            }
        }
        else
        {
            /* Typicall I2C interrupt routine */
            i2c_handler(channel); 
        }
    }
    else
    {
        /* Clear the I2C Interrupt Flag. */
        I2C_S(channel) |= I2C_S_IICIF_MASK; 
        /* clear timeout flag */
        I2C_SMB(channel) |= I2C_SMB_SLTF_MASK;
        #if DEBUG
        printf("Low Timeout occurs\r\n");
        #endif
    }
}
