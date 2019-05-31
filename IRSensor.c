#define ABS(x) ((x)>=0) ? (x) : (-x)
#define LEFT 1
#define RIGHT 2
#define STRAIGHT 3

volatile unsigned int uhPrescalerValue = 0;
volatile unsigned int sensor_order[8] = {0x0000,0x0040,0x0080,0x00c0,0x0100,0x0140,0x0180,0x01c0};
volatile unsigned char sensor_index = 0;
volatile unsigned int sensor_ad_data[16];
volatile unsigned int sensor_state;

int black_max[8];
int white_max[8];
int normalized_data[8];

int normalized_sum;
int displacement[8] = {0, -10000, -6000, -2000, 2000, 6000, 10000, 0};
int position;
int ex_position;
int current_position = 0;
int compare_position = 0;

int limit_i = 10;
int limit_o = 1;

int stop;

int threshold[8];
int thres_percent;

int mark_state = 0;
int accumulate_state = 0;
int accumulate_cross_state = 0;

int left_mark = 0;
int right_mark = 0;
int end_mark = 0;
int cross_mark = 0;
int all_mark;

int all_mark_index;

int saved_mark[300];
int saved_line[300];
int map_count[300];
int count_int = 0;

int correct_mark[300];

int left_map_count_array[300];
int right_map_count_array[300];

int mark_int = 0;


void Sensor_Interrupt(void) {
    GPIOG -> ODR = (GPIOG -> ODR & 0xFE3F) | sensor_order[sensor_index]; //sensor select
    GPIOG -> ODR |= GPIO_Pin_11; // sensor on
    Delay_us(5);
    Adc_Soc1(ADC1, ADC_Channel_3);
    GPIOG -> ODR &= (~GPIO_Pin_11); // sensor off
    sensor_ad_data[sensor_index] = adc_data_a >> 4;

	// bit mover ex. 00001000 << 1 = 00010000, 00000001 << 5 00100000	
	if (sensor_ad_data[sensor_index] > threshold[sensor_index]) sensor_state |= 1 << (7 - sensor_index);
	else if (sensor_ad_data[sensor_index] < threshold[sensor_index]) sensor_state &= ~(1 << (7 - sensor_index));

	// hexadecimal ex. 0x01 = binary 0000 0001, 0x02 = binary 0000 0010
    sensor_index = (sensor_index + 1) & 0x07;
}


void Start_Sensor(void) {
	Timer_Enable_Ir();	
}


void Stop_Sensor(void) {
	Timer_Disable_Ir();
}


void test_sensor(void) {
    sensor_index = 0;
	for (;;) {
		GPIOG -> ODR = (GPIOG -> ODR & 0xFE3F) | sensor_order[sensor_index];
		GPIOG -> ODR |= GPIO_Pin_11;
		Delay_us(5);
		Adc_Soc1(ADC1, ADC_Channel_3);
		GPIOG -> ODR &= (~GPIO_Pin_11);
		
		Lcd_Printf("/00EMI : %d /10data:%3x", sensor_index, adc_data_a);
		Inswitch2();

		if (sw == SW_SHORT_1 || sw == SW_LONG_1) sensor_index = (sensor_index + 1) & 0x07;
		else if (sw == SW_SHORT_2 || sw == SW_LONG_2) sensor_index = (sensor_index - 1) & 0x07;
		else if (sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
	}
}


void Calibration(void) {
	int cal_int;
	black_max[cal_int] = 0;
	white_max[cal_int] = 0;
	
	thres_percent = 15;
	
	Start_Sensor();
	Lcd_Clear();
	
	while(!Inswitch2())	Lcd_Printf("/00blackmax");
	
	while(!Inswitch2()) {
		for (cal_int = 0; cal_int < 8; cal_int++) {
			if (black_max[cal_int] < sensor_ad_data[cal_int]) black_max[cal_int] = sensor_ad_data[cal_int];
		}
		Lcd_Printf("/00%2x%2x%2x%2x/10%2x%2x%2x%2x", black_max[0], black_max[1], black_max[2], black_max[3], black_max[4], black_max[5], black_max[6], black_max[7]);
	}
	
	Lcd_Clear();
	
	while (!Inswitch2()) Lcd_Printf("/00whitemax");
	
	while (!Inswitch2()) {
		for (cal_int = 0; cal_int < 8; cal_int++) {
			if (white_max[cal_int] < sensor_ad_data[cal_int]) white_max[cal_int] = sensor_ad_data[cal_int];
		}
		
		Lcd_Printf("/00%2x%2x%2x%2x/10%2x%2x%2x%2x", white_max[0], white_max[1], white_max[2], white_max[3], white_max[4], white_max[5], white_max[6], white_max[7]);
	}
	
	Lcd_Clear();
	
	for (;;) {
		Inswitch2();
		
		if (sw==SW_SHORT_1||sw==SW_LONG_1) thres_percent+=1;
		if (sw==SW_SHORT_2||sw==SW_LONG_2) thres_percent-=1;
		if (sw==SW_SHORT_BOTH||sw==SW_LONG_BOTH) break;
	
		Lcd_Printf("/00THRESH/10%d", thres_percent);
	}
	
	for (cal_int = 0; cal_int < 8; cal_int++) threshold[cal_int] = (float)(((white_max[cal_int] - black_max[cal_int]) * thres_percent / 100) + black_max[cal_int]);
	
	Lcd_Clear();
}


void Normalization(void) {
	int nor_int;
	unsigned int sensor_ad_data_temp[8];
	
	for (nor_int = 0; nor_int < 8; nor_int++) {
		sensor_ad_data_temp[nor_int] = sensor_ad_data[nor_int];
		if (sensor_ad_data_temp[nor_int] < black_max[nor_int]) normalized_data[nor_int] = 0;
		else if (sensor_ad_data_temp[nor_int] > white_max[nor_int]) normalized_data[nor_int] = 255;
		else normalized_data[nor_int] = 255 * (float)((sensor_ad_data_temp[nor_int] - black_max[nor_int])) / (white_max[nor_int] - black_max[nor_int]);
	}
}


void Position(void) {
	float position_1 = 0;
	int pos_int;

	normalized_sum = 0;
	Normalization();
	
	for (pos_int = 1; pos_int < 7; pos_int++) {
		position_1 += (float)(normalized_data[pos_int] * displacement[pos_int]);
		normalized_sum += (normalized_data[pos_int]);
	}
	
	if (normalized_sum==0) normalized_sum=1;
	
	position = (int)((float)position_1 / normalized_sum);
	
	if (position < 0) ex_position = -position;
	else ex_position = position;
}


void Test_Position(void) {
	for (;;) { 
		Position();
		Lcd_Printf("/00%d",current_position);
	}
}


void Set_Mark(void) {
	if ((sensor_state & 0xff) == 0) stop = 1;
	
	if (mark_state == 0) { // observe_state
		if ((sensor_state & 0x24) == 0x24) mark_state = 3; // 0010 0100		
		else if ((sensor_state & 0x81) != 0) mark_state = 1;
	} else if (mark_state == 1) { // accumulate_state
		accumulate_state |= sensor_state;
		if ((accumulate_state & 0x24) == 0x24) mark_state = 3;
		else if ((sensor_state & 0x81) == 0) mark_state = 2; // decide_state = 2
	} else if (mark_state == 2) { // decide_state
		if ((accumulate_state & 0x24) == 0x24) {
			mark_state = 3;
			return;
		}
		
		if ((accumulate_state & 0x81) == 0x80) {
			left_mark++;
			all_mark++;
			
			left_map_count_array[count_int] = left_map_count;
			right_map_count_array[count_int] = right_map_count;
			
			saved_mark[mark_int] = LEFT; // 1 == left
			// map_count[count_int] = (left_map_count + right_map_count) / 2;
			
			left_map_count = 0;
			right_map_count = 0;
			mark_int++;
			count_int++;
		} else if ((accumulate_state & 0x81) == 0x01) {
			right_mark++;
			all_mark++;
			
			left_map_count_array[count_int] = left_map_count;
			right_map_count_array[count_int] = right_map_count;
			
			saved_mark[mark_int] = RIGHT; // 2 == right
			// map_count[count_int] = (left_map_count + right_map_count) / 2;
			
			left_map_count = 0;
			right_map_count = 0;
			mark_int++;
			count_int++;
		} else if ((accumulate_state & 0x81) == 0x81) {	
			end_mark++;
			all_mark++;
			
			left_map_count_array[count_int] = left_map_count;
			right_map_count_array[count_int] = right_map_count;
			
			saved_mark[mark_int] = STRAIGHT; // 3 == straight
			// map_count[count_int] = (left_map_count + right_map_count) / 2;
			
			left_map_count=0;
			right_map_count=0;
			mark_int++;
			count_int++;
			
			if (end_mark==2) stop=1;
		}
		
		mark_state = 0;
		accumulate_state = 0;
	} else if (mark_state == 3) { // cross
		accumulate_state |= sensor_state;
		if ((accumulate_state & 0x81) == 0x81) mark_state = 4;
	} else if (mark_state == 4) { // cross end
		if ((sensor_state & 0x81) == 0) {
			cross_mark++;
			accumulate_state = 0;
			mark_state = 0;
		}
	}
}


void Saved_Line(void) {
	int i;
	all_mark_index = all_mark;
	
	for (i = 0; i < all_mark_index + 1; i++) {
		correct_mark[i] = saved_mark[i];
		map_count[i] = (left_map_count_array[i+1] + right_map_count_array[i+1]) / 2;
	}
	
	saved_line[0] = STRAIGHT;
	
	for (count_int = 1; count_int < 301; count_int++) {
		if (saved_line[count_int-1] == STRAIGHT) {
			if (saved_mark[count_int] == STRAIGHT) saved_line[count_int] = STRAIGHT;
			else if (saved_mark[count_int] == LEFT) saved_line[count_int] = LEFT;
			else if (saved_mark[count_int] == RIGHT) saved_line[count_int] = RIGHT;
		} else if (saved_mark[count_int-1] == LEFT) {
			if (saved_mark[count_int] == STRAIGHT) saved_line[count_int] = STRAIGHT;
			else if (saved_mark[count_int] == LEFT) saved_line[count_int] = STRAIGHT;
			else if (saved_mark[count_int] == RIGHT) saved_line[count_int] = RIGHT;
		} else if (saved_mark[count_int-1] == RIGHT) {
			if (saved_mark[count_int] == STRAIGHT) saved_line[count_int] = STRAIGHT;
			else if(saved_mark[count_int] == LEFT) saved_line[count_int] = LEFT;
			else if(saved_mark[count_int] == RIGHT) saved_line[count_int] = STRAIGHT;
		}
	}
}


void Secondary_Mark(void)
{
	if((sensor_state & 0xff) == 0) stop = 1;
	if (mark_state == 0) { // observe_state
		if ((sensor_state & 0x24) == 0x24) mark_state = 3; // 0010 0100
		else if ((sensor_state & 0x81) != 0) mark_state = 1;			
	} else if (mark_state == 1) { // accumulate_state
		accumulate_state |= sensor_state;
		if ((accumulate_state & 0x24) == 0x24) mark_state = 3;
		else if ((sensor_state & 0x81) == 0) mark_state = 2; // decide_state = 2
	} else if (mark_state == 2) { // decide_state
		if ((accumulate_state & 0x24) == 0x24) {
			mark_state = 3;
			return;
		}
		if((accumulate_state & 0x81) == 0x80) left_mark++;
		else if((accumulate_state & 0x81) == 0x01) right_mark++;
		else if ((accumulate_state & 0x81) == 0x81) {	
			end_mark++;
			if (end_mark==2) stop = 1;		
		}
		mark_state = 0;
		accumulate_state = 0;
	} else if(mark_state == 3) { // cross
		accumulate_state |= sensor_state;
		if ((accumulate_state & 0x81) == 0x81) mark_state = 4;
	} else if (mark_state == 4) { // cross end
		if ((sensor_state & 0x81) == 0) {
			cross_mark++;
			accumulate_state = 0;
			mark_state = 0;
		}	
	}	
}


void Secondary_Run(void) {
	mark_state = 0;
	accumulate_state = 0;
	accumulate_cross_state = 0;

	left_mark = 0;
	right_mark = 0;
	end_mark = 0;
	cross_mark = 0;	
	stop = 0;
	
	Saved_Line();
	
	return;
}


void Reset_Value(void) {
	left_mark = 0;
	right_mark = 0;
	end_mark = 0;
	cross_mark = 0;
	count_int = 0;
	pre_mark_index = 0;
}


void Secondary_Reset_Value(void) {
	stop = 0;
	left_mark = 0;
	right_mark = 0;
	end_mark = 0;
	all_mark=0;
	cross_mark = 0;
	pre_mark_index = 0;
	mark_int = 0;
}


void Limiter(void) {
	if (current_position > ex_position) {
		current_position -= limit_i;
		if (ex_position >= current_position) current_position = ex_position;
	} else if(current_position < ex_position) {
		current_position += limit_i;
		if (ex_position <= current_position) current_position = ex_position;
	} else current_position = ex_position;
}


void Change_Limiter(void) {
	limit_i = 10;

	for (;;) {
		Inswitch2();
		if (sw == SW_SHORT_1 || sw == SW_LONG_1) limit_i += 1;
		if (sw == SW_SHORT_2 || sw == SW_LONG_2) limit_i -= 1;
		if (sw == SW_SHORT_BOTH || sw == SW_LONG_BOTH) break;
		Lcd_Printf("/00LIMITER/10%d", limit_i);
	}
}
