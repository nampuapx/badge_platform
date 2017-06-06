/**
 *
 * \file
 *
 * \brief This module contains NMC1000 bus wrapper APIs implementation.
 *
 * Copyright (c) 2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

#include <stdio.h>
#include "bsp/include/nm_bsp.h"
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
//#include "asf.h"
#include "conf_winc.h"
#include <stm32f2xx_hal.h>


///////////////////////////////////////////////////////
#include "cmsis_os.h"
	osThreadId spi_thread_id;


extern SPI_HandleTypeDef hspi3;

///////////////////////////////////////////

#define NM_BUS_MAX_TRX_SZ	256
SPI_HandleTypeDef hspi1;
tstrNmBusCapabilities egstrNmBusCapabilities =
{
	NM_BUS_MAX_TRX_SZ
};

#ifdef CONF_WINC_USE_I2C

struct i2c_master_module i2c_master_instance;
#define SLAVE_ADDRESS 0x60

/** Number of times to try to send packet if failed. */
#define I2C_TIMEOUT 100

static sint8 nm_i2c_write(uint8 *b, uint16 sz)
{
	sint8 result = M2M_SUCCESS;
	uint16_t timeout = 0;

	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = b,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}

	return result;
}

static sint8 nm_i2c_read(uint8 *rb, uint16 sz)
{
	uint16_t timeout = 0;
	sint8 result = M2M_SUCCESS;
	struct i2c_master_packet packet = {
		.address     = SLAVE_ADDRESS,
		.data_length = sz,
		.data        = rb,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) != STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == I2C_TIMEOUT) {
			break;
		}
	}

	return result;
}

static sint8 nm_i2c_write_special(uint8 *wb1, uint16 sz1, uint8 *wb2, uint16 sz2)
{
	static uint8 tmp[NM_BUS_MAX_TRX_SZ];
	m2m_memcpy(tmp, wb1, sz1);
	m2m_memcpy(&tmp[sz1], wb2, sz2);
	return nm_i2c_write(tmp, sz1+sz2);
}
#endif

#ifdef CONF_WINC_USE_SPI

//struct spi_module master;
//struct spi_slave_inst slave_inst;


//void WINC_SPI_callback(uint32_t event)
//{
//    switch (event)
//    {
//    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
//        /* Success: Wakeup Thread */
//        osSignalSet(spi_thread_id, 0x0100);
//        break;
//    case ARM_SPI_EVENT_DATA_LOST:
//        /*  Occurs in slave mode when data is requested/sent by master
//            but send/receive/transfer operation has not been started
//            and indicates that data is lost. Occurs also in master mode
//            when driver cannot transfer data fast enough. */
//        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
//        break;
//    case ARM_SPI_EVENT_MODE_FAULT:
//        /*  Occurs in master mode when Slave Select is deactivated and
//            indicates Master Mode Fault. */
//        __breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
//        break;
//    }
//}




static sint8 spi_rw(uint8* pu8Mosi, uint8* pu8Miso, uint16 u16Sz)
{
	uint8 u8Dummy = 0;
	uint8 u8SkipMosi = 0, u8SkipMiso = 0;
	uint8_t txd_data = 0;
	uint8_t rxd_data = 0;

	osEvent evt;
	
  osPriority priority; 
	spi_thread_id = osThreadGetId (); 
	priority = osThreadGetPriority (spi_thread_id);
	
	
	
	if (!pu8Mosi) {


				//HAL_StatusTypeDef HAL_SPI_Transmit(hspi3, uint8_t *pData, uint16_t Size, uint32_t Timeout)
				HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout)

				Driver_SPI3.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
				osThreadSetPriority(spi_thread_id,osPriorityHigh);
				
				Driver_SPI3.Receive(pu8Miso, u16Sz);
				
				evt = osSignalWait(0x0100, 1000);
        if (evt.status == osEventTimeout) {
            __breakpoint(0); /* Timeout error: Call debugger */
        }
				LED_off(LED_BLUE);

				Driver_SPI3.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);

	}
	else if(!pu8Miso) {
		//pu8Miso = &u8Dummy;
		//u8SkipMiso = 1;

				Driver_SPI3.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_ACTIVE);
				osThreadSetPriority(spi_thread_id,osPriorityHigh);
				Driver_SPI3.Send(pu8Mosi, u16Sz);
				
				LED_on(LED_BLUE);
				//while(WINC1500_SPI.GetStatus().busy);
				evt = osSignalWait(0x0100, 1000);
        if (evt.status == osEventTimeout) {
            __breakpoint(0); /* Timeout error: Call debugger */
        }
				LED_off(LED_BLUE);				

				Driver_SPI3.Control(ARM_SPI_CONTROL_SS, ARM_SPI_SS_INACTIVE);
	}
	else {
		return M2M_ERR_BUS_FAIL;
	}
	osThreadSetPriority(spi_thread_id,priority);
	return M2M_SUCCESS;
	
//////////////Old//////////////////////////////////	
	HAL_GPIO_WritePin(GPIOA,CONF_WINC_PIN_CS,GPIO_PIN_RESET);
//	spi_select_slave(&master, &slave_inst, true);

	while (u16Sz) {
		txd_data = *pu8Mosi;
		//printf("\nsend %d",txd_data);

		//HAL_SPI_TransmitReceive(&hspi1,&txd_data,&rxd_data,1,1000);
    Driver_SPI3.Transfer(&txd_data, &rxd_data, 1);
    while(Driver_SPI3.GetStatus().busy);


		//HAL_SPI_Transmit(&hspi1,&txd_data,1,1000);
		//HAL_SPI_Receive(&hspi1,&rxd_data,1,1000);
//		while (!spi_is_ready_to_write(&master))
//			;
//		while(spi_write(&master, txd_data) != STATUS_OK)
//			;

//		/* Read SPI master data register. */
//		while (!spi_is_ready_to_read(&master))
//			;
//		while (spi_read(&master, &rxd_data) != STATUS_OK)
//			;
		*pu8Miso = rxd_data;
//printf("\nrecv %d",rxd_data);
		u16Sz--;
		if (!u8SkipMiso)
			pu8Miso++;
		if (!u8SkipMosi)
			pu8Mosi++;
	}

//	while (!spi_is_write_complete(&master))
//		;

//	spi_select_slave(&master, &slave_inst, false);
HAL_GPIO_WritePin(GPIOA,CONF_WINC_PIN_CS,GPIO_PIN_SET);
	return M2M_SUCCESS;
}
#endif

/*
*	@fn		nm_bus_init
*	@brief	Initialize the bus wrapper
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*/
sint8 nm_bus_init(void *pvinit)
{
	sint8 result = M2M_SUCCESS;

#ifdef CONF_WINC_USE_I2C
	/* Initialize config structure and software module. */
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);

	/* Change buffer timeout to something longer. */
	config_i2c_master.buffer_timeout = 1000;

	/* Initialize and enable device with config. */
	i2c_master_init(&i2c_master_instance, SERCOM2, &config_i2c_master);

	i2c_master_enable(&i2c_master_instance);

#elif defined CONF_WINC_USE_SPI
//	hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
//  hspi1.Init.CRCPolynomial = 10;
	/* Structure for SPI configuration. */
//	struct spi_config config;
//	struct spi_slave_inst_config slave_config;

//	/* Select SPI slave CS pin. */
//	/* This step will set the CS high */
//	spi_slave_inst_get_config_defaults(&slave_config);
//	slave_config.ss_pin = CONF_WINC_SPI_CS_PIN;
//	spi_attach_slave(&slave_inst, &slave_config);

//	/* Configure the SPI master. */
//	spi_get_config_defaults(&config);
//	config.mux_setting = CONF_WINC_SPI_SERCOM_MUX;
//	config.pinmux_pad0 = CONF_WINC_SPI_PINMUX_PAD0;
//	config.pinmux_pad1 = CONF_WINC_SPI_PINMUX_PAD1;
//	config.pinmux_pad2 = CONF_WINC_SPI_PINMUX_PAD2;
//	config.pinmux_pad3 = CONF_WINC_SPI_PINMUX_PAD3;
//	config.master_slave_select_enable = false;

//	config.mode_specific.master.baudrate = CONF_WINC_SPI_CLOCK;
//	if (spi_init(&master, CONF_WINC_SPI_MODULE, &config) != STATUS_OK) {
//		return M2M_ERR_BUS_FAIL;
//	}

//	/* Enable the SPI master. */
//	spi_enable(&master);

	nm_bsp_reset();
	nm_bsp_sleep(1);
#endif
	return result;
}

/*
*	@fn		nm_bus_ioctl
*	@brief	send/receive from the bus
*	@param[IN]	u8Cmd
*					IOCTL command for the operation
*	@param[IN]	pvParameter
*					Arbitrary parameter depenging on IOCTL
*	@return	M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@note	For SPI only, it's important to be able to send/receive at the same time
*/
sint8 nm_bus_ioctl(uint8 u8Cmd, void* pvParameter)
{
	sint8 s8Ret = 0;
	switch(u8Cmd)
	{
#ifdef CONF_WINC_USE_I2C
		case NM_BUS_IOCTL_R: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_read(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W: {
			tstrNmI2cDefault *pstrParam = (tstrNmI2cDefault *)pvParameter;
			s8Ret = nm_i2c_write(pstrParam->pu8Buf, pstrParam->u16Sz);
		}
		break;
		case NM_BUS_IOCTL_W_SPECIAL: {
			tstrNmI2cSpecial *pstrParam = (tstrNmI2cSpecial *)pvParameter;
			s8Ret = nm_i2c_write_special(pstrParam->pu8Buf1, pstrParam->u16Sz1, pstrParam->pu8Buf2, pstrParam->u16Sz2);
		}
		break;
#elif defined CONF_WINC_USE_SPI
		case NM_BUS_IOCTL_RW: {
			tstrNmSpiRw *pstrParam = (tstrNmSpiRw *)pvParameter;
			s8Ret = spi_rw(pstrParam->pu8InBuf, pstrParam->pu8OutBuf, pstrParam->u16Sz);
		}
		break;
#endif
		default:
			s8Ret = -1;
			M2M_ERR("invalide ioclt cmd\n");
			break;
	}

	return s8Ret;
}

/*
*	@fn		nm_bus_deinit
*	@brief	De-initialize the bus wrapper
*/
sint8 nm_bus_deinit(void)
{
	sint8 result = M2M_SUCCESS;
//	struct port_config pin_conf;
//		
//	port_get_config_defaults(&pin_conf);
//	/* Configure control pins as input no pull up. */
//	pin_conf.direction  = PORT_PIN_DIR_INPUT;
//	pin_conf.input_pull = PORT_PIN_PULL_NONE;

//#ifdef CONF_WINC_USE_I2C
//	i2c_master_disable(&i2c_master_instance);
//#endif /* CONF_WINC_USE_I2C */
//#ifdef CONF_WINC_USE_SPI
//	spi_disable(&master);
//#endif /* CONF_WINC_USE_SPI */
	return result;
}

/*
*	@fn			nm_bus_reinit
*	@brief		re-initialize the bus wrapper
*	@param [in]	void *config
*					re-init configuration data
*	@return		M2M_SUCCESS in case of success and M2M_ERR_BUS_FAIL in case of failure
*	@author		Dina El Sissy
*	@date		19 Sept 2012
*	@version	1.0
*/
sint8 nm_bus_reinit(void* config)
{
	return M2M_SUCCESS;
}

