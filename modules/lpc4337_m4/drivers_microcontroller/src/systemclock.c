	/* Copyright 2019,
 * Sebastian Mateos
 * smateos@ingenieria.uner.edu.ar
 * Facultad de Ingeniería
 * Universidad Nacional de Entre Ríos
 * Argentina
 *
 * All rights reserved.
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
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Bare Metal driver for the clock of EDU-CIAA board.
 **
 **/

/*
 * Initials     Name
 * ---------------------------
 * SM		Sebastian Mateos
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20190228 v0.1 SM initial version
 */

/*==================[inclusions]=============================================*/
#include "systemclock.h"
#include "chip.h"
#include "bool.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
/* Structure for initial base clock states */
struct CLK_BASE_STATES {
	CHIP_CGU_BASE_CLK_T clk;	/* Base clock */
	CHIP_CGU_CLKIN_T clkin;	/* Base clock source, see UM for allowable souorces per base clock */
	bool autoblock_enab;/* Set to true to enable autoblocking on frequency change */
	bool powerdn;		/* Set to true if the base clock is initially powered down */
};

/* Initial base clock states are mostly on */
STATIC const struct CLK_BASE_STATES InitClkStates[] = {
	{CLK_BASE_PHY_TX, CLKIN_ENET_TX, true, false},
#if defined(USE_RMII)
	{CLK_BASE_PHY_RX, CLKIN_ENET_TX, true, false},
#else
	{CLK_BASE_PHY_RX, CLKIN_ENET_RX, true, false},
#endif

	/* Clocks derived from dividers */
	{CLK_BASE_LCD, CLKIN_IDIVC, true, false},
	{CLK_BASE_USB1, CLKIN_IDIVD, true, true}
};


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/
//const uint32_t ExtRateIn = 0;  /**< Frecuencia de clock externa inyectada al microcontrolador. No utilizado en las CIAA. */
//const uint32_t OscRateIn = 12000000; /**< Frecuencia del oscilador externo incorporado en la CIAA-NXP. */

/*==================[internal functions definition]==========================*/
static void ciaaSetupCoreClock(CHIP_CGU_CLKIN_T clkin, uint32_t core_freq, bool setbase)
{
	int i;
	volatile uint32_t delay = 500;
	uint32_t direct = 0;
	PLL_PARAM_T ppll;

	if (clkin == CLKIN_CRYSTAL) {
		/* Switch main system clocking to crystal */
		Chip_Clock_EnableCrystal();
	}
	Chip_Clock_SetBaseClock(CLK_BASE_MX, clkin, true, false);
	Chip_Clock_DisableMainPLL(); /* Disable PLL */

	/* Calculate the PLL Parameters */
	ppll.srcin = clkin;
	Chip_Clock_CalcMainPLLValue(core_freq, &ppll);

	if (core_freq > 110000000UL) {
		if (!(ppll.ctrl & (1 << 7)) || ppll.psel) {
			PLL_PARAM_T lpll;
			/* Calculate the PLL Parameters */
			lpll.srcin = clkin;
			Chip_Clock_CalcMainPLLValue(110000000UL, &lpll);
			Chip_Clock_SetupMainPLL(&lpll);
			/* Wait for the PLL to lock */
			while(!Chip_Clock_MainPLLLocked()) {}
			Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_MAINPLL, true, false);
			while(delay --){}
			delay = 500;
		} else {
			direct = 1;
			ppll.ctrl &= ~(1 << 7);
		}
	}

	/* Setup and start the PLL */
	Chip_Clock_SetupMainPLL(&ppll);

	/* Wait for the PLL to lock */
	while(!Chip_Clock_MainPLLLocked()) {}

	/* Set core clock base as PLL1 */
	Chip_Clock_SetBaseClock(CLK_BASE_MX, CLKIN_MAINPLL, true, false);

	while(delay --){} /* Wait for approx 50 uSec */
	if (direct) {
		delay = 5000;
		ppll.ctrl |= 1 << 7;
		Chip_Clock_SetupMainPLL(&ppll); /* Set DIRECT to operate at full frequency */
		while(delay --){} /* Wait for approx 50 uSec */
	}

	if (setbase) {
		/* Setup system base clocks and initial states. This won't enable and
		   disable individual clocks, but sets up the base clock sources for
		   each individual peripheral clock. */
		for (i = 0; i < (sizeof(InitClkStates) / sizeof(InitClkStates[0])); i++) {
			Chip_Clock_SetBaseClock(InitClkStates[i].clk, InitClkStates[i].clkin,
									InitClkStates[i].autoblock_enab, InitClkStates[i].powerdn);
		}
	}
}
/*==================[external functions definition]==========================*/

void SystemClockInit(void)
{
 	SystemCoreClockUpdate();
 	Chip_SetupXtalClocking();
 	//ciaaSetupCoreClock(CLKIN_CRYSTAL, MAX_CLOCK_FREQ, true);
}

/*==================[end of file]============================================*/
