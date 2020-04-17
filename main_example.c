
#include <stdio.h>
#include <fps220_d20h.h>

/* Private variable */
int32_t real_p_s32, real_t_s32;
//float real_p, real_t;
extern volatile uint32_t TMR0_Ticks = 0; //one tick per millisecond(ms)
extern volatile uint32_t fps220_update_rdy = 0;

/**
 * @brief      A timer generate an interrupt every millisecond
 */
void TMR0_IRQHandler(void)
{
	if (TIMER_GetIntFlag(TIMER0) == 1)
	{
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER0);

		TMR0_Ticks++;
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	/* fps220 initiation */
	fps220_init();

	while (1)
	{
		/* Updating fps220 data */
		fps220_update_data();
		if (fps220_update_rdy) {
			/* If you need the pressure value read is in uint of Pa, use this function. */
			// real_p = fps220_read_pressure();
			/* If you need the temperature value read is in unit of degree Celsius, use this function. */
			// real_t = fps220_read_temperature();

			/* This function read pressure and temperature values. Pressure uint:0.01 Pa, Temperature unit:0.01 degree Celsius */
			fps220_read_data(&real_p_s32, &real_t_s32);
			fps220_update_rdy = 0;
		}
	}
}
