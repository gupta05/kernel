/*
 * Provides constants for the pm8xxx multy-purpose pin (MPP) binding.
 */

#ifndef _DT_BINDINGS_PINCTRL_QCOM_PM8XXX_MPP_H
#define _DT_BINDINGS_PINCTRL_QCOM_PM8XXX_MPP_H

/*
 * Analog Output - Set the analog output reference.
 * To be used with "qcom,aout = <>"
 */
#define PM8XXX_MPP_AOUT_1V25                   0
#define PM8XXX_MPP_AOUT_1V25_2                 1
#define PM8XXX_MPP_AOUT_0V625                  2
#define PM8XXX_MPP_AOUT_0V3125                 3
#define PM8XXX_MPP_AOUT_MPP                    4
#define PM8XXX_MPP_AOUT_ABUS1                  5
#define PM8XXX_MPP_AOUT_ABUS2                  6
#define PM8XXX_MPP_AOUT_ABUS3                  7
#define PM8XXX_MPP_AOUT_ABUS4                  8

/*
 * Analog Input - Set the source for analog input.
 * To be used with "qcom,ain = <>"
 */
#define PM8XXX_MPP_AIN_CH5                     0
#define PM8XXX_MPP_AIN_CH6                     1
#define PM8XXX_MPP_AIN_CH7                     2
#define PM8XXX_MPP_AIN_CH8                     3
#define PM8XXX_MPP_AIN_CH9                     4
#define PM8XXX_MPP_AIN_ABUS1                   5
#define PM8XXX_MPP_AIN_ABUS2                   6
#define PM8XXX_MPP_AIN_ABUS3                   7
#define PM8XXX_MPP_AIN_ABUS4                   8

#endif
