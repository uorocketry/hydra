/* Auto-generated config file peripheral_clk_config.h */
#ifndef PERIPHERAL_CLK_CONFIG_H
#define PERIPHERAL_CLK_CONFIG_H

// <<< Use Configuration Wizard in Context Menu >>>

/**
 * \def CONF_CPU_FREQUENCY
 * \brief CPU's Clock frequency
 */
#ifndef CONF_CPU_FREQUENCY
#define CONF_CPU_FREQUENCY 12000000
#endif

// <y> CAN0 Clock Source
// <id> can_gclk_selection

// <GCLK_PCHCTRL_GEN_GCLK0_Val"> Generic clock generator 0

// <GCLK_PCHCTRL_GEN_GCLK1_Val"> Generic clock generator 1

// <GCLK_PCHCTRL_GEN_GCLK2_Val"> Generic clock generator 2

// <GCLK_PCHCTRL_GEN_GCLK3_Val"> Generic clock generator 3

// <GCLK_PCHCTRL_GEN_GCLK4_Val"> Generic clock generator 4

// <GCLK_PCHCTRL_GEN_GCLK5_Val"> Generic clock generator 5

// <GCLK_PCHCTRL_GEN_GCLK6_Val"> Generic clock generator 6

// <GCLK_PCHCTRL_GEN_GCLK7_Val"> Generic clock generator 7

// <GCLK_PCHCTRL_GEN_GCLK8_Val"> Generic clock generator 8

// <GCLK_PCHCTRL_GEN_GCLK9_Val"> Generic clock generator 9

// <GCLK_PCHCTRL_GEN_GCLK10_Val"> Generic clock generator 10

// <GCLK_PCHCTRL_GEN_GCLK11_Val"> Generic clock generator 11

// <i> Select the clock source for CAN0.
#ifndef CONF_GCLK_CAN0_SRC
#define CONF_GCLK_CAN0_SRC GCLK_PCHCTRL_GEN_GCLK0_Val
#endif

/**
 * \def CONF_GCLK_CAN0_FREQUENCY
 * \brief CAN0's Clock frequency
 */
#ifndef CONF_GCLK_CAN0_FREQUENCY
#define CONF_GCLK_CAN0_FREQUENCY 12000000
#endif

// <<< end of configuration section >>>

#endif // PERIPHERAL_CLK_CONFIG_H
