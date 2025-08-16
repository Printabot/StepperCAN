/*
 * @file    tmc2209.c
 * @brief   Implements functions for basic configuration of TMC2209 drivers
 * @author  github.com/printabot
 * @date    2025-07-13
 *
 * @warning This implementation uses bit-field structures which may have
 *          portability issues across different compilers and architectures.
 * @note    A portable bitmask-based implementation is being developed
 *          to replace this version.
 *
 * @details Current limitations:
 * - Bit-field ordering/padding is compiler-dependent
 * - Not guaranteed to work on all architectures
 * - Endianness considerations not handled
 *
 * Temporary workaround:
 * - #pragma pack directives are used to minimize padding issues
 * - Currently validated only for STM32 Cortex-M0 (GCC)
 *
 * Planned improvements:
 * - Migration to portable bitmask/shift implementation
 * - Endianness-aware access functions
 * - Compiler-agnostic register access
 *
 * License: CC BY-NC-SA 4.0. See LICENSE.
 * Disclaimer: Provided as is, no warranty of any kind.
 */

#include <string.h>
#include <stdbool.h>
#include "../Inc/tmc2209.h"
#include "../../Inc/port.h"
#include "../../Inc/uart.h"

static void set_rms_current (TMC2209_t *driver);

static const TMC2209_t tmc2209_default_values = {
    .config.f_clk = TMC2209_F_CLK,
    .config.mode = TMC2209_MODE,
    .config.r_sense = TMC2209_R_SENSE,
    .config.current = TMC2209_CURRENT,
    .config.hold_current_pct = TMC2209_HOLD_CURRENT_PCT,
    .config.microsteps = TMC2209_MICROSTEPS,
    .gconf.addr.reg = GCONF_ADDR,
    .gconf.reg.en_spreadcycle = TMC2209_SPREADCYCLE,
    .ihold_irun.addr.reg = IHOLD_IRUN_ADDR,
    .ihold_irun.reg.iholddelay = TMC2209_IHOLDDELAY,
    .chopconf.addr.reg = CHOPCONF_ADDR,
    .chopconf.reg.tbl = TMC2209_TBL,
    .chopconf.reg.toff = TMC2209_TOFF,
    .chopconf.reg.hstrt = TMC2209_HSTRT-1,
    .chopconf.reg.hend = TMC2209_HEND+3,
    .chopconf.reg.intpol = TMC2209_INTPOL,
};

/**
  * @brief Calculate irun and ihold values using fixed point
  *
  * @param TMC2209_t	Pointer to driver struct
  *
  * @retval None
  */
static void set_rms_current (TMC2209_t *driver)
{
    /* tmp en µV (mA * mΩ = µV), incluye el factor 32 */
    uint32_t rs_mohm = (uint32_t)(driver->config.r_sense) + 20U;   /* +20 mΩ de offset */
    uint32_t i32_ma  = 32U * (uint32_t)driver->config.current;     /* hasta ~64k si current<=2000 */
    uint32_t tmp_uV  = rs_mohm * i32_ma;                           /* máx ~11e6 -> cabe en 32 bits */

    /* Multiplica por sqrt(2) ≈ 181/128 con redondeo
       (tmp_uV * 181 + 64) >> 7  == (tmp_uV * 181)/128 redondeado */
    uint32_t tmp_uV_sqrt2 = (tmp_uV * 181U + 64U) >> 7;             /* sigue en µV */

    /* pasa a mV con redondeo */
    uint32_t maxv_mV = (tmp_uV_sqrt2 + 500U) / 1000U;

    /* Calcula CS (current_scaling) con vsense por defecto
        current_scaling = (maxv_mV / vsense_mV) - 1
        Si es negativo, satura a 0 */
    int32_t cs = (int32_t)(maxv_mV / (uint32_t)325);
    cs -= 1;
    if (cs < 0) cs = 0;

    /* Si CS < 16, activa vsense=1 (Vfs pequeño) y recalcula con vsense[1] */
    if (cs < 16) {
        driver->chopconf.reg.vsense = 1;
        cs = (int32_t)(maxv_mV / (uint32_t)180) - 1;
        if (cs < 0) cs = 0;
    } else {
        driver->chopconf.reg.vsense = 0;
    }

    /* Satura a 5 bits (0..31) y aplica hold% */
    if (cs > 31) cs = 31;
    uint8_t irun = (uint8_t)cs;

    driver->ihold_irun.reg.irun  = irun;
    driver->ihold_irun.reg.ihold = (uint8_t)(( (uint32_t)irun * (uint32_t)driver->config.hold_current_pct ) / 100U);
}

/**
  * @brief Configure the TMC driver according board_config.h
  *
  * @param TMC2209_t	Pointer to driver struct
  *
  * @retval None
  */
void TMC2209_config (TMC2209_t *driver, uint8_t microsteps, uint16_t current, uint8_t hold_current, bool dedge)
{
	driver->config.motor.address = 0;

	driver->gconf.reg.I_scale_analog = 0; 		/* Use internal reference derived from 5VOUT */
	driver->gconf.reg.internal_Rsense = 0; 		/* Operation with external sense resistors */
	driver->gconf.reg.pdn_disable = 1; 			/* PDN_UART input function disabled. Set this bit,  when using the UART interface! */
	driver->gconf.reg.mstep_reg_select = 1;		/* Microstep resolution selected by MRES register */
	driver->gconf.reg.en_spreadcycle = 0;		/* StealthChop PWM mode enabled */
	TMC2209_write_register(driver, (TMC2209_datagram_t*)&driver->gconf);

	driver->chopconf.reg.dedge = 1;				/* Enable step impulse at each step edge to reduce step frequency requirement.*/
	TMC2209_write_register(driver, (TMC2209_datagram_t*)&driver->chopconf);

	/* Set current */
	driver->config.current = (uint16_t)400;
	driver->config.hold_current_pct = (uint8_t)20;
	set_rms_current(driver);
	TMC2209_write_register(driver, (TMC2209_datagram_t *)&driver->chopconf);
	TMC2209_write_register(driver, (TMC2209_datagram_t *)&driver->ihold_irun);

	/* Set microsteps */
	driver->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps_32);
	driver->config.microsteps = (tmc2209_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
	TMC2209_write_register(driver, (TMC2209_datagram_t *)&driver->chopconf);

}

void set_default_values(TMC2209_t *driver){
	memcpy(driver, &tmc2209_default_values, sizeof(TMC2209_t));		/* Set default values */
}


void TMC2209_write_register (TMC2209_t *driver, TMC2209_datagram_t *reg)
{
	TMC_uart_tx_datagram_t datagram;

	datagram.msg.sync = 0x05;
    datagram.msg.slave = driver->config.motor.address;
    datagram.msg.addr.value = reg->addr.value;
    datagram.msg.addr.write = 1;
    datagram.msg.payload.value = reg->payload.value;

    byte_swap(datagram.msg.payload.data);
    tmc_crc8(datagram.data, sizeof(TMC_uart_tx_datagram_t));

    delay_ms(10);
    tmc_uart_write(&driver->config.motor, &datagram);
    clear_WDT();
}



uint8_t tmc_microsteps_to_mres (uint16_t microsteps)
{
    uint8_t value = 0;

    if(microsteps == 0){
    	microsteps = 1;
    }

    while((microsteps & 0x01) == 0) {
        value++;
        microsteps >>= 1;
    }
    if(value > 8){
    	value = 8;
    }

    return 8 - value;
}

void tmc_crc8 (uint8_t *datagram, uint8_t length)
{
    int32_t i;
	int32_t j;
    uint8_t *crc = datagram + (length - 1u);
    uint8_t cbyte;
    *crc = 0;

    for (i = 0; i < (length - 1u); i++) {
    	cbyte = datagram[i];
        for (j = 0; j < 8u; j++) {
            if ((*crc >> 7u) ^ (cbyte & 0x01u)){
                *crc = (*crc << 1u) ^ 0x07u;
            }
            else{
                *crc = (*crc << 1u);
            }
            cbyte = cbyte >> 1u;
        }
    }
}

void byte_swap (uint8_t data[4])
{
    uint8_t tmp;

    tmp = data[0];
    data[0] = data[3];
    data[3] = tmp;
    tmp = data[1];
    data[1] = data[2];
    data[2] = tmp;
}



