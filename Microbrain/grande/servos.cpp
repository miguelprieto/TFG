#include "defines.h"
#include <libpic30++.h>
#include <stdio.h>

#include "bus_interface.h"
#include "ecan_lib.h"
#include "pwm.h"
#include "wait.h"
#include "servos.h"

t_servo_status_grande servo_status;

void update_servo_status_grande(const uint8_t *data, unsigned int len, void *user_ptr)
{
    servo_status =  *(t_servo_status_grande*)data;

}

bool automation_busy(void)
{
	// OCCUPATO = 1 , LIBERO = 0
    ecan_update_object(SERVO_STATUS_GRANDE_OBJECT);
    return servo_status.busy_flags;
}

void servos_off(void)
{
    // PARATIE
    set_servo_position(PAR_SX, 0);
    set_servo_position(PAR_DX, 0);

    // BAFFI
    set_servo_position(BAF_SX, 0);
    set_servo_position(BAF_DX, 0);

    // UPDOWN
    set_servo_position(UPD_SX, 0);
    set_servo_position(UPD_DX, 0);

    // ABBASSA
    set_servo_position(ABB_BASC, 0);
}


void set_baffi_start (void)
{
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(300);

	set_servo_position (BAF_SX, (BAF_SX_UPD));
	set_servo_position (BAF_DX, (BAF_DX_UPD));
	__delay_ms(300);
	
	set_servo_position (UPD_SX, (UPD_SX_DOWN));
	set_servo_position (UPD_DX, (UPD_DX_DOWN));
	__delay_ms(300);

	set_servo_position (UPD_SX, (UPD_SX_UP));
	set_servo_position (UPD_DX, (UPD_DX_UP));
	__delay_ms(300);
}

void set_abbassa_start()
{
	set_servo_position(ABB_BASC, ABB_ALTA);
}

void set_spazzola_start()
{
	if(color==YELLOW)
		set_servo_position(SPA, SPA_YEL);
	else
		set_servo_position(SPA, SPA_BLU);
		
}

void servo_reset(void)
{
	set_baffi_start();
	__delay_ms(300)
	set_abbassa_start();
	__delay_ms(300)
	set_spazzola_start();
	__delay_ms(1500);
}
