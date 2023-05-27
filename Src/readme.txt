作者邮箱：oskar@mindsilicon.com




// 处理无线数据
void ParseRadioMsg(radioMsg_t *msg)
{
	int16_t pwm1 = 0, pwm2 = 0;
	static float duty[8] = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
	uint16_t keys = 0;
	
	// 先计算电机的PWM值, data[4]是右侧摇杆垂直方向, 用来控制前后运动
	pwm1 = -(msg->data[4] - 0x7F) * 8;
	pwm2 = pwm1;
	// data[3]是右侧摇杆水平方向, 用来控制左右转向时电机的差速
	pwm1 += (msg->data[3] - 0x7F) * 5;
	pwm2 -= (msg->data[3] - 0x7F) * 5;
	motorStatus.motor1_pwm = pwm1;
	motorStatus.motor2_pwm = pwm2;
	
	// data的第6和第7字节, 这两个字节共16位, 对应手柄的16个通道, 
	// 从最高位第16位到第1位, 依次是：L2, L1, LU, LL, LD, LR, SE,
	// ST, RL, RD, RR, RU, R1, R2, R-KEY, L-KEY
	// 其中R-KEY和L-KEY分别是左右摇杆向下按下对应的按键
	keys = (msg->data[5] << 8) | msg->data[6];
	
	// 2号舵机控制, 用L1和R1键控制正转和反转, 
	if((keys & (1 << 14)) && (!(keys & (1 << 3))))				// L1按下且R1松开时
	{
		duty[1] -= 0.005;
		PWM(&duty[1]);
		servoPWM.servo2 = Duty_to_PWM(duty[1]);
	}
	else if((!(keys & (1 << 14))) && (keys & (1 << 3)))		// L1松开且R1按下时
	{
		duty[1] += 0.005;
		PWM(&duty[1]);
		servoPWM.servo2 = Duty_to_PWM(duty[1]);
	}
	
	// 3号舵机控制, 用L2和R2键控制正转和反转, 
	if((keys & (1 << 15)) && (!(keys & (1 << 2))))				// L2按下且R2松开时
	{
		duty[2] -= 0.005;
		PWM(&duty[2]);
		if(duty[2] < 0.1)		// 防止云台垂直舵机转动角度过大，而导致堵转
		{
			duty[2] = 0.1;
		}
		servoPWM.servo3 = Duty_to_PWM(duty[2]);
	}
	else if((!(keys & (1 << 15))) && (keys & (1 << 2)))		// L2松开且R2按下时
	{
		duty[2] += 0.005;
		PWM(&duty[2]);
		if(duty[2] > 0.9)		// 防止云台垂直舵机转动角度过大，而导致堵转
		{
			duty[2] = 0.9;
		}
		servoPWM.servo3 = Duty_to_PWM(duty[2]);
	}
}





// 显示系统参数 
void OLED_ShowParm(void)
{
	char volt1[20],volt2[20],motor1[20],motor2[20];
	
	sprintf(volt2,"Main  Voltage:%5.2f",ADC_ConvertedValueLocal[1]);
	OLED_ShowStr(0,0,(uint8_t*)(&volt2),1);
	
	sprintf(volt1, "Servo Voltage:%5.2f",ADC_ConvertedValueLocal[0]);
	OLED_ShowStr(0,1,(uint8_t*)(&volt1),1);
	
	sprintf(motor1,"motor1_pwm:%5d",motorStatus.motor1_pwm);
	OLED_ShowStr(0,4,(uint8_t*)(&motor1),1);
	
	sprintf(motor2,"motor2_pwm:%5d",motorStatus.motor2_pwm);
	OLED_ShowStr(0,5,(uint8_t*)(&motor2),1);
	
	vTaskDelay(50);	
}











