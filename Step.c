#include <math.h>

float left_velo = 0.0f;
float right_velo = 0.0f;

float current_velo = 0.2f;
float target_velo = 2.0f;

float real_target_velo = 2.0f;
float slow_target_velo = 1.5f;

float accel = 3.0f;
float decel = 3.0f;

float comp_velo = 2.0f;
float d_comp_const = 14000.f;
float comp_const = 16000.f;

float dcurrent_velo = 0.2f;

int left_map_count = 0;
int right_map_count = 0;

int delay_const = 20;

int decel_safety_step = 0;
int pre_mark_index;
float initial_velo;
float peak_velo;
int mark_error = 0;
int end_flag = 0;
int second_map_count = 0;

int i = 0;


void Step_Motor_Left(void) {
	// speed -> timer
	if (left_velo < 0.13086973288014245122458253507825f) left_velo = 0.14f;
	
	TIM1 -> ARR = 8576.547944300135541003016436353f / left_velo;

	l_phase_index = (l_phase_index + 1) & 0x07; // next phase
	l_port = l_phase[l_phase_index];
	Poke_Data_Bits(l_map, 4); // port out
	
	left_map_count++;
}


void Step_Motor_Right(void) {
	if (right_velo < 0.13086973288014245122458253507825f) right_velo = 0.14f;
	
	TIM8 ->ARR =8576.547944300135541003016436353f / right_velo;
	
	r_phase_index = (r_phase_index + 1) & 0x07;
	r_port = r_phase[r_phase_index];
	Poke_Data_Bits(r_map, 4);
	
	right_map_count++;
}


void Follow_Motor_Velocity(void) {
	Lcd_Clear();
	Step_Motor_Start();
	
	for (;;) {
		
		Inswitch2();
		left_velo = current_velo;
		right_velo = current_velo;
		
		Lcd_Printf("/00%2f/10%2f", current_velo, accel);
		
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) accel += 0.01f;
		if (sw == SW_SHORT_2 || sw == SW_LONG_2) accel -= 0.01f;		
	}
}


void Control_Motor_Velocity(void) {
	Lcd_Clear();
	Step_Motor_Start();
	
	for(;;) {
		Inswitch2();
		// Step_Motor_Left();
		// Step_Motor_Right();
		
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) {
			left_velo += 0.1f;
			right_velo += 0.1f;
			// target_velo += 0.1f;
		}
			
		if (sw == SW_SHORT_2 || sw == SW_LONG_2) {
			left_velo -= 0.1f;
			right_velo -= 0.1f;
			// target_velo -= 0.1f;
		}
		
		Lcd_Printf("/00%2f/10%2f", left_velo, right_velo);
	}
}


void Turn_Motor_Velocity(void) {
	Lcd_Clear();
	Reset_Value(); // 2

	Start_Sensor();
	Step_Motor_Start();
	
	for (;;) {
		Set_Mark();
		
		dcurrent_velo = (float)(current_velo * (1 - (current_position / comp_const)));
		left_velo = dcurrent_velo * (1 + (position / 12500.f));		
		right_velo = dcurrent_velo * (1 - (position / 12500.f));
		
		if (stop == 1) {
			Delay_ms(delay_const);
			decel = (current_velo * current_velo) / 0.4f;
			target_velo = 0.0f;
			break;
		}
	}
	
	while (current_velo >= 0.14f) {
		if ((position < -2000) || (position > 2000)) {
			left_velo = current_velo * (1 + (position / 16000.f));
			right_velo = current_velo * (1 - (position / 16000.f));
		} else {
			left_velo = current_velo * (1 + (position / 12500.f));
			right_velo = current_velo * (1 - (position / 12500.f));
		}
	}
	Delay_ms(10);
	Step_Motor_Stop();
}


void Secondary(void)
{
	pre_mark_index = all_mark - 1;
	
	if (all_mark_index > 0 && pre_mark_index >= 0) {
		if ((correct_mark[pre_mark_index] == saved_mark[pre_mark_index]) && (!mark_error) && (!end_flag)) {
			if (saved_line[pre_mark_index] == STRAIGHT) {
				second_map_count = (left_map_count + right_map_count) / 2; //second_map_count=avg_step
				if (map_count[pre_mark_index] > 400) { // 400 = safety_step
					if (pre_mark_index<all_mark_index-2) {
						decel_safety_step=((current_velo + initial_velo) * (current_velo - initial_velo) * 400) / (2 * decel) / (2 * 3.14159265359f * 0.02534f);
						if (second_map_count > 400) {
							if (second_map_count > map_count[pre_mark_index] - 400 - decel_safety_step) target_velo = initial_velo;
							else target_velo=peak_velo;
						}		
					} else {
						decel_safety_step = (current_velo * current_velo * 400) / (2 * decel) / (2 * 3.14159265359f * 0.02534f);
						if (second_map_count > 400) {
							if (second_map_count > (map_count[pre_mark_index] + (0.25f * 400 / (2 * 3.14159265359f * 0.02534f))) - 400 - decel_safety_step) {
								end_flag = 1;
								target_velo = 0;
							} else target_velo=peak_velo;
						}
					}
				}
			}	
		} else {
			mark_error = 1;
			target_velo = initial_velo;		
		}
	}
}


void Second_Run(void)
{
	Lcd_Clear();
	
	Secondary_Reset_Value();

	Start_Sensor();
	Step_Motor_Start();
	
	for (;;) {
		Set_Mark();
		Secondary();
		
		dcurrent_velo=(float)(current_velo*(1-(current_position/comp_const)));
		left_velo=dcurrent_velo*(1+(position/12500.f));		
		right_velo=dcurrent_velo*(1-(position/12500.f));
		
		// if (position < -2000 || position > 2000) {
		// 	left_velo = current_velo * (1 + (position / 16000.f));			
		// 	right_velo = current_velo * (1 - (position / 16000.f));
		// } else {
		// 	left_velo = current_velo * (1 + (position / 12500.f));
		// 	right_velo = current_velo * (1 - (position / 12500.f));
		// }
		
		if (stop == 1) {
			Delay_ms(delay_const);
			decel = (current_velo * current_velo) / 0.4f;
			target_velo = 0.0f;
			break;
		}
	}
	
	while (current_velo >= 0.14f) {
		Position();
		
		if ((position < -2000) || (position > 2000)) {
			left_velo=current_velo * (1 + (position / 16000.f));
			right_velo=current_velo * (1 - (position / 16000.f));
		} else {
			left_velo = current_velo * (1 + (position / 12500.f));
			right_velo = current_velo * (1 - (position / 12500.f));
		}
	}
	Delay_ms(10);
	// Lcd_Printf("/10%2d%2d%2d%2d", left_mark, right_mark, end_mark, cross_mark);
	Step_Motor_Stop();
}


void Change_Comp_Const(void) {
	comp_const = 16000.f;

	for (;;) {
		Inswitch2();
			
		if(sw == SW_SHORT_1 || sw == SW_LONG_1) comp_const += 500.f;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) comp_const -= 500.f;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		
		Lcd_Printf("/00COMPC/10%f", comp_const);
	}
}



void Change_Target_Velocity(void) {
	target_velo = 2.0f;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) target_velo += 0.1f;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) target_velo -= 0.1f;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		
		Lcd_Printf("/00VELOCTRL/10%1.2f", target_velo);
	}
}


void Change_Slow_Velocity(void) {
	slow_target_velo = 1.5f;
		
	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) slow_target_velo += 0.1f;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) slow_target_velo -= 0.1f;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		
		Lcd_Printf("/00SLOWCTRL/10%1.2f", slow_target_velo);
	}
}


void Fit_In(void) {
	delay_const=20;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) delay_const += 1;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) delay_const -= 1;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		
		Lcd_Printf("/00FITIN/10%2d", delay_const);
	}
}


void Change_Accel(void) {
	accel=3.0;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) accel += 1;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) accel -= 1;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		
		Lcd_Printf("/00ACCEL/10%1.2f", accel);
	}
}


void Change_Decel(void)
{
	decel=3.0;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) decel += 1;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) decel -= 1;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;

		Lcd_Printf("/00DECEL/10%1.2f", decel);
	}
}


void Change_Initial_Velocity(void)
{
	initial_velo=1.5f;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) initial_velo += 0.1f;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) initial_velo -= 0.1f;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;

		Lcd_Printf("/00INITVELO/10%1.2f", initial_velo);	
	}
}


void Change_Peak_Velocity(void)
{
	peak_velo=1.5f;

	for (;;) {
		Inswitch2();
			
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) peak_velo += 0.1f;
		if(sw == SW_SHORT_2 || sw == SW_LONG_2) peak_velo -= 0.1f;
		if(sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;

		Lcd_Printf("/00PEAKVELO/10%1.2f", peak_velo);
	}
}


void Change_Motor_Velocity(void) { // PendSV_Handler Timer.c
	if(current_velo > target_velo) {
		current_velo -= decel * 0.0004f;
		if(target_velo >= current_velo) current_velo = target_velo;
	} else if(current_velo < target_velo) {
		current_velo += accel * 0.0004f;
		if (target_velo <= current_velo) current_velo = target_velo;
	}
}
