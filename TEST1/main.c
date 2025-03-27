#include "stm32f10x.h"

#define SDA_0 GPIO_ResetBits(GPIOB,GPIO_Pin_0)//PB0 xuong muc 0
#define SDA_1 GPIO_SetBits (GPIOB,GPIO_Pin_0)//PB0 len muc 1
#define SCL_0 GPIO_ResetBits(GPIOB,GPIO_Pin_1)//PB1 xuong muc 0
#define SCL_1 GPIO_SetBits (GPIOB,GPIO_Pin_1)//PB1 len muc 1
#define SDA_VAL GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)// ham nay tra ve 1 gia tri 1 hoac 0 nghia la cao or thap


void Delay1MS(void);
void Delay_MS(uint32_t u32DelayInMs);
void Delay_US(uint32_t Delay);
void TIM2_Init(void);

void TIM2_Init(void) {
    TIM_TimeBaseInitTypeDef timerInit;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    timerInit.TIM_CounterMode = TIM_CounterMode_Up;
    timerInit.TIM_Period = 100 - 1;
    timerInit.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInit(TIM2, &timerInit);
    TIM_Cmd(TIM2, ENABLE);
}

void Delay1MS(void) 
{
	TIM_SetCounter(TIM2,0);
	while (TIM_GetCounter(TIM2)<1000) {
	}
}
void Delay_MS(uint32_t u32DelayInMs)
{
	while (u32DelayInMs){
	    Delay1MS();
		  --u32DelayInMs;
	}
}

void Delay_US(uint32_t Delay)
{
	TIM_SetCounter(TIM2,0);
	while (TIM_GetCounter(TIM2)<Delay) {
	}
}

void i2c_init(void);
void i2c_start(void);
void i2c_stop(void);
uint8_t i2c_write(uint8_t u8Data);// tra ve gia tri neu ben nhan xac nhan, neu ma xung thu 9 dang o muc 0 thi trar ve 1 va nguoc lai
uint8_t i2c_read(uint8_t u8Ack);

void i2c_init(void)
{
  GPIO_InitTypeDef gpioInit;// kieu struct
	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);//bat clock len
	gpioInit.GPIO_Mode = GPIO_Mode_Out_OD ;//che do Output Open-Drain
  gpioInit.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;// Chon chan 0 va 1 lam SDA va SCk
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 
	GPIO_Init(GPIOB,&gpioInit);//ham trong vi du
	SDA_1;
	SCL_1;
}

void i2c_start(void)//TU 1 XUONG 0 KHI SCL O MUC CAO
{
	SCL_1;
	Delay_US(3);
	SDA_1;
	Delay_US(3);
	SDA_0;
	Delay_US(3);
	SCL_0;
	Delay_US(3);
	
}

void i2c_stop(void)//TU 0 XUONG 1 KHI SCL O MUC CAO
{
	SDA_0;
	Delay_US(3);
	SCL_1;
	Delay_US(3);
	SDA_1;
	Delay_US(3);
	
}

uint8_t i2c_write(uint8_t u8Data)// tra ve gia tri neu ben nhan xac nhan, neu ma xung thu 9 dang o muc 0 thi trar ve 1 va nguoc lai
{
  uint8_t i;
	uint8_t u8Ret;
	

	for (i=0; i<8; ++i) {
	  if(u8Data & 0x80){
		   SDA_1;// Neu bit cao nhat la 1, dat SDA len muc cao
		}else {
			SDA_0;// Neu bit cao nhat la 0, dat SDA xuong muc thap
		}
		Delay_US(3);// Delay de on dinh tin hieu
		SCL_1;// Dua xung clock len muc cao
    Delay_US(5);	// Giu clock o muc cao de thiet bi nhan lay du lieu
    SCL_0;// Dua clock xuong muc thap
    Delay_US(2);	
    u8Data <<=1;	// Dich trai du lieu de gui bit tiep theo	
	}
	SDA_1;// Nha SDA de nhan phan hoi tu slave 
	Delay_US(3);
	SCL_1;// Dua SCL len cao de doc tin hieu ACK tu slave
	Delay_US(3);
	if (SDA_VAL){// Kiem tra trang thai cua SDA 
		u8Ret = 0 ;// Neu SDA van cao, tuc la khong co ACK (NACK) 
	}else {
		u8Ret =1 ;// Neu SDA xuong thap, tuc la co ACK 
	}
	Delay_US(2);	
  SCL_0;// Dua clock xuong thap de ket thuc ACK 
  Delay_US(5);
	
	return u8Ret;
}
/* truyen du lieu khi chan SCL o muc cao va thay doi du lieu khi SCL o muc thap
   xung thu 9 SDA muc thap xac nhan, muc cao khong xac nhan*/

uint8_t i2c_read(uint8_t u8Ack)
{
	uint8_t i;
	uint8_t u8Buffer;
	
	SDA_1;// Chuyen SDA ve muc cao de dam bao no o che do input
	Delay_US(3);
	
	for (i=0; i<8; ++i) {
		u8Buffer <<=1;// Dich trai de chuan bi nhan bit tiep theo
		SCL_1;// Dua xung clock len cao de nhan du lieu
		Delay_US(3);
		if (SDA_VAL){
			u8Buffer |= 0x01; // Neu SDA = 1, ghi bit 1 vao u8Buffer
		}
		Delay_US(2);
		SCL_0;// Dua clock xuong thap
		Delay_US(5);
		
	}
	// Gui tin hieu ACK hoac NACK  
	if (u8Ack){
		SDA_0;// Neu u8Ack = 1, gui ACK (SDA xuong thap)
	}else {
		SDA_1;// Neu u8Ack = 0, gui NACK (SDA len cao)
	}
	Delay_US(3);
	
	SCL_1;// Dua xung clock len cao de xac nhan ACK/NACK
	Delay_US(5);
	SCL_0;// Dua clock xuong thap ket thuc ACK/NACK
	Delay_US(5);
	
	
	return u8Buffer;// Tra ve byte da doc
}

 int main(void)
 {
  TIM_TimeBaseInitTypeDef timerInit;// kieu struct
	GPIO_InitTypeDef gpioInit;// kieu struct
	TIM_OCInitTypeDef pwmInit;// kieu struct
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC,ENABLE);//bat clock len
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE);//bat clock len
	 
	gpioInit.GPIO_Mode = GPIO_Mode_Out_PP ;//che do Output Push-Pull
  gpioInit.GPIO_Pin = GPIO_Pin_13;// Chon chan PC13
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 
	GPIO_Init(GPIOC,&gpioInit);//ham trong vi du
	 
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP  ;//che do Output Push-Pull
  gpioInit.GPIO_Pin = GPIO_Pin_0;// Chon chan PC0
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 /*pa0 gan voi pwm1*/
	 
	GPIO_Init(GPIOC,&gpioInit);//ham trong vi du
	 
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP  ;//che do Output Push-Pull
  gpioInit.GPIO_Pin = GPIO_Pin_1;// Chon chan PC1
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 /*pa0 gan voi pwm2*/
	 
	GPIO_Init(GPIOC,&gpioInit);//ham trong vi du

	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP  ;//che do Output Push-Pull
  gpioInit.GPIO_Pin = GPIO_Pin_2;// Chon chan PC2
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 /*pa0 gan voi pwm3*/
	 
	GPIO_Init(GPIOC,&gpioInit);//ham trong vi du
	
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP  ;//che do Output Push-Pull
  gpioInit.GPIO_Pin = GPIO_Pin_3;// Chon chan PC3
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	 /*pa0 gan voi pwm4*/
	 
	GPIO_Init(GPIOC,&gpioInit);//ham trong vi du
	 
	timerInit.TIM_CounterMode = TIM_CounterMode_Up;//che do timer dem len
	timerInit.TIM_Period = 100-1;// GIA TRI THANH GHI NAP LAI(BO DEM) thanh ghi ARR
	timerInit.TIM_Prescaler = 72 -1 ;// BO CHIA TAN (12)
	 
	TIM_TimeBaseInit(TIM2,&timerInit);// HAM TRONG VI DU
	
	TIM_Cmd(TIM2,ENABLE);// BAT TIMER LEN CHAY
	/*ch1 */
	/*duty 10%*/
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = 10;
	pwmInit.TIM_OutputState = TIM_OutputNState_Enable;
	TIM_OC1Init (TIM2,&pwmInit);
	
	/*ch2 */
	/*duty 25%*/
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = 25;
	pwmInit.TIM_OutputState = TIM_OutputNState_Enable;
	TIM_OC2Init (TIM2,&pwmInit);
	
	/*ch3 */
	/*duty 40%*/
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = 40;
	pwmInit.TIM_OutputState = TIM_OutputNState_Enable;
	TIM_OC3Init (TIM2,&pwmInit);
	
	/*ch4 */
	/*duty 80%*/
	pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
	pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
	pwmInit.TIM_Pulse = 80;
	pwmInit.TIM_OutputState = TIM_OutputNState_Enable;
	TIM_OC4Init (TIM2,&pwmInit);
	/*truoc khi xuat xung ra cac chan can cau hinh cac chan ket noi voi timer, can cau hinh cac chan nay che do alternate, mac dinh cac chan nay cau hinh che do gpio*/
	
	 while(1){
		 GPIO_SetBits (GPIOC,GPIO_Pin_13);
		 Delay_MS(100);
		 GPIO_ResetBits(GPIOC,GPIO_Pin_13);
		 Delay_MS(100);
	 }
 }
