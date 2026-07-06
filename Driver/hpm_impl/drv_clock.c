#include "intf_clock.h"
#include "hpm_clock_drv.h"
#include "hpm_pllctlv2_drv.h"
#include "hpm_sysctl_drv.h"
#include "hpm_pcfg_drv.h"

void intf_clock_init(void)
{
    uint32_t cpu0_freq = clock_get_frequency(clock_cpu0);

    if (cpu0_freq == PLLCTL_SOC_PLL_REFCLK_FREQ) {
        pllctlv2_xtal_set_rampup_time(HPM_PLLCTLV2, 32UL * 1000UL * 9U);
        sysctl_clock_set_preset(HPM_SYSCTL, 2);
    }

    pllctlv2_select_reference_clock(HPM_PLLCTLV2, pllctlv2_pll0, 0);
    pllctlv2_select_reference_clock(HPM_PLLCTLV2, pllctlv2_pll1, 0);

    clock_add_to_group(clock_cpu0, 0);
    clock_add_to_group(clock_ahb, 0);
    clock_add_to_group(clock_lmm0, 0);
    clock_add_to_group(clock_mchtmr0, 0);
    clock_add_to_group(clock_rom, 0);
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_gpio, 0);
    clock_add_to_group(clock_hdma, 0);
    clock_add_to_group(clock_xpi0, 0);
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    clock_add_to_group(clock_can0, 0);
    clock_set_source_divider(clock_can0, clk_src_pll1_clk0, 10);

    clock_connect_group_to_cpu(0, 0);

    /* Bump DCDC to the SDK-recommended voltage for stable 480MHz CPU operation. */
    pcfg_dcdc_set_voltage(HPM_PCFG, 1275);

    /* Configure CPU to 480MHz, AXI/AHB to 160MHz from PLL0CLK0. */
    sysctl_config_cpu0_domain_clock(HPM_SYSCTL, clock_source_pll0_clk0, 2, 3);

    /* Configure PLL0 to 960MHz. */
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll0, pllctlv2_clk0, pllctlv2_div_1p0);
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll0, pllctlv2_clk1, pllctlv2_div_1p6);
    pllctlv2_set_postdiv(HPM_PLLCTLV2, pllctlv2_pll0, pllctlv2_clk2, pllctlv2_div_2p4);
    pllctlv2_init_pll_with_freq(HPM_PLLCTLV2, pllctlv2_pll0, 960000000);

    clock_update_core_clock();

    clock_set_source_divider(clock_mchtmr0, clk_src_osc24m, 1);
}

uint32_t intf_clock_get_cpu_freq(void)
{
    return clock_get_frequency(clock_cpu0);
}

uint32_t intf_clock_get_ahb_freq(void)
{
    return clock_get_frequency(clock_ahb);
}

uint32_t intf_clock_get_cycle(void)
{
    uint32_t value;
    __asm__ volatile("csrr %0, mcycle" : "=r"(value));
    return value;
}

void intf_clock_delay_ms(uint32_t ms)
{
    clock_cpu_delay_ms(ms);
}

void intf_clock_delay_us(uint32_t us)
{
    clock_cpu_delay_us(us);
}
