/*
 * @file    tmc2209.h
 * @brief   Header for tmc2209.c
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

#ifndef _TMC2209_H_
#define _TMC2209_H_

#include <stdint.h>
#include <stdbool.h>

#pragma pack(push, 1)

typedef struct {
    uint8_t id;         /* motor id */
    uint8_t axis;       /* axis index */
    uint8_t address;    /* UART address */
} trinamic_motor_t;

enum tmc2209_reg_addr {
    GCONF_ADDR        = 0x00,
    IHOLD_IRUN_ADDR   = 0x10,
    CHOPCONF_ADDR     = 0x6C,
};

typedef enum {
    microsteps_1 = 1,
    microsteps_2 = 2,
    microsteps_4 = 4,
    microsteps_8 = 8,
    microsteps_16 = 16,
    microsteps_32 = 32,
    microsteps_64 = 64,
    microsteps_128 = 128,
    microsteps_256 = 256
} tmc2209_microsteps_t;

typedef enum {
   StealthChop = 0,
   CoolStep,
   StallGuard,
} trinamic_mode_t;

/* Default values */
#define TMC2209_F_CLK               12000000UL
#define TMC2209_MICROSTEPS          microsteps_4
#define TMC2209_CURRENT             500
#define TMC2209_R_SENSE             110
#define TMC2209_HOLD_CURRENT_PCT    50

/* 0 = StealthChop, 1 = CoolStep, 3 = StallGuard */
#define TMC2209_MODE                StealthChop

/* IHOLD_IRUN */
#define TMC2209_IHOLDDELAY  		10

/* GCONF */
#define TMC2209_SPREADCYCLE   		0

/* CHOPCONF */
#define TMC2209_INTPOL              1
#define TMC2209_TOFF                3
#define TMC2209_TBL                 1
#define TMC2209_HSTRT               1
#define TMC2209_HEND               -1
#define TMC2209_HMAX               16

/* Register struct */
/* --------------------------------------------------------- */

typedef union {
    uint8_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2209_addr_t;
/* --------------------------------------------------------- */

typedef union {
    uint32_t value;
    struct {
        uint32_t
        I_scale_analog   :1,
        internal_Rsense  :1,
        en_spreadcycle   :1,
        shaft            :1,
        index_otpw       :1,
        index_step       :1,
        pdn_disable      :1,
        mstep_reg_select :1,
        multistep_filt   :1,
        test_mode        :1,
        reserved         :22;
    };
} TMC2209_gconf_register_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_gconf_register_t reg;
} TMC2209_gconf_datagram_t;
/* --------------------------------------------------------- */

typedef union {
    uint32_t value;
    struct {
        uint32_t
        reserved0 :8,
        conf      :4,
        reserved1 :20;
    };
} TMC2209_slaveconf_reg_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_slaveconf_reg_t reg;
} TMC2209_slaveconf_datagram_t;
/* --------------------------------------------------------- */

typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        reserved1  :3,
        irun       :5,
        reserved2  :3,
        iholddelay :4,
        reserved3  :12;
    };
} TMC2209_ihold_irun_reg_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ihold_irun_reg_t reg;
} TMC2209_ihold_irun_datagram_t;
/* --------------------------------------------------------- */

typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,
        hstrt     :3,
        hend      :4,
        reserved0 :4,
        tbl       :2,
        vsense    :1,
        reserved1 :6,
        mres      :4,
        intpol    :1,
        dedge     :1,
        diss2g    :1,
        diss2vs   :1;
    };
} TMC2209_chopconf_reg_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_chopconf_reg_t reg;
} TMC2209_chopconf_datagram_t;
/* --------------------------------------------------------- */

/* Payload */
typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2209_gconf_register_t 	gconf;
    TMC2209_slaveconf_reg_t 	slaveconf;
    TMC2209_ihold_irun_reg_t 	ihold_irun;
    TMC2209_chopconf_reg_t 		chopconf;
} TMC2209_payload;

/* Datagram */
typedef struct {
     TMC2209_addr_t 	addr;
     TMC2209_payload 	payload;
} TMC2209_datagram_t;

/* Configuration */
typedef struct {
    uint32_t 			f_clk;
    uint16_t 			microsteps;
    uint16_t 			r_sense;
    uint16_t 			current;
    uint8_t 			hold_current_pct;
    trinamic_mode_t 	mode;
    trinamic_motor_t 	motor;
} trinamic_config_t;

typedef struct {
	trinamic_config_t 				config;
	TMC2209_gconf_datagram_t 		gconf;
    TMC2209_ihold_irun_datagram_t 	ihold_irun;
    TMC2209_chopconf_datagram_t 	chopconf;
} TMC2209_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} tmc_generic_addr_t;

typedef union {
    uint32_t value;
    uint8_t data[4];
} tmc_generic_payload_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        tmc_generic_addr_t addr;
        tmc_generic_payload_t payload;
        uint8_t crc;
    } msg;
} TMC_uart_tx_datagram_t;

#pragma pack(pop)

void tmc_crc8 (uint8_t *datagram, uint8_t length);
void byte_swap (uint8_t data[4]);
void TMC2209_config (TMC2209_t *driver, uint8_t microsteps, uint16_t current, uint8_t hold_current, bool dedge);
void TMC2209_write_register (TMC2209_t *driver, TMC2209_datagram_t *reg);
void set_default_values(TMC2209_t *driver);
uint8_t tmc_microsteps_to_mres (uint16_t microsteps);

#endif
