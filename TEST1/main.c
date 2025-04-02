#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>

#define SDA_0 GPIO_ResetBits(GPIOB,GPIO_Pin_0)//PB0 xuong muc 0
#define SDA_1 GPIO_SetBits (GPIOB,GPIO_Pin_0)//PB0 len muc 1
#define SCL_0 GPIO_ResetBits(GPIOB,GPIO_Pin_1)//PB1 xuong muc 0
#define SCL_1 GPIO_SetBits (GPIOB,GPIO_Pin_1)//PB1 len muc 1
#define SDA_VAL GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)// ham nay tra ve 1 gia tri 1 hoac 0 nghia la cao or thap

void delay(void);
void Delay1MS(void);
void Delay_MS(uint32_t u32DelayInMs);
void Delay_US(uint32_t Delay);
void TIM2_Init(void);
void UART_Init(void);
void PWM1_Init(uint16_t pulse);
void PWM2_Init(uint16_t pulse);
void PWM3_Init(uint16_t pulse);
void PWM4_Init(uint16_t pulse);
void UART_SendString(char *str);
void ADC_InitConfig(void);
void UART_SendChar(char c);
void DHT11_Init(void);
void DHT11_Check(void);
char* DHT11_Data(void);

char* DHT11_Data(void){
	
	uint8_t Data[5];
	uint8_t i;uint8_t check;	
	 static char buffer[50];

	
	//nhan byte so 1//
	for(i=0;i < 8; i++){
		//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<65){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=65)||(u16Tim<=45)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		//cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<80){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=80)||(u16Tim<=10)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 Data[0]<<=1;//làm cho bit dau bang 0
		 if(u16Tim>45){
			 //nhan duoc bit 1//
			 Data[0] |=1;// giu nguyen nhung bit cao, set bit thap len 1
		 } else {
			 //nhan duoc bit 0//
			 Data[0] &=~1;// 00000001->11111110, xoa bit thap ve 0 giu lai nhung bit cao, 
		 }
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	//nhan byte so 2//
	for(i=0;i < 8; i++){
		//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<65){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=65)||(u16Tim<=45)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		//cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<80){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=80)||(u16Tim<=10)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 Data[1]<<=1;//làm cho bit dau bang 0
		 if(u16Tim>45){
			 //nhan duoc bit 1//
			 Data[1] |=1;// giu nguyen nhung bit cao, set bit thap len 1
		 } else {
			 //nhan duoc bit 0//
			 Data[1] &=~1;// 00000001->11111110, xoa bit thap ve 0 giu lai nhung bit cao, 
		 }
	}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//nhan byte so 3//
	for(i=0;i < 8; i++){
		//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<65){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=65)||(u16Tim<=45)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		//cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<80){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=80)||(u16Tim<=10)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 Data[2]<<=1;//làm cho bit dau bang 0
		 if(u16Tim>45){
			 //nhan duoc bit 1//
			 Data[2] |=1;// giu nguyen nhung bit cao, set bit thap len 1
		 } else {
			 //nhan duoc bit 0//
			 Data[2] &=~1;// 00000001->11111110, xoa bit thap ve 0 giu lai nhung bit cao, 
		 }
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//nhan byte so 4//
	for(i=0;i < 8; i++){
		//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<65){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=65)||(u16Tim<=45)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		//cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<80){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=80)||(u16Tim<=10)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 Data[3]<<=1;//làm cho bit dau bang 0
		 if(u16Tim>45){
			 //nhan duoc bit 1//
			 Data[3] |=1;// giu nguyen nhung bit cao, set bit thap len 1
		 } else {
			 //nhan duoc bit 0//
			 Data[3] &=~1;// 00000001->11111110, xoa bit thap ve 0 giu lai nhung bit cao, 
		 }
	}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//nhan byte so 5//
	for(i=0;i < 8; i++){
		//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<65){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=65)||(u16Tim<=45)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		//cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<80){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=80)||(u16Tim<=10)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 Data[4]<<=1;//làm cho bit dau bang 0
		 if(u16Tim>45){
			 //nhan duoc bit 1//
			 Data[4] |=1;// giu nguyen nhung bit cao, set bit thap len 1
		 } else {
			 //nhan duoc bit 0//
			 Data[4] &=~1;// 00000001->11111110, xoa bit thap ve 0 giu lai nhung bit cao, 
		 }
	}
  
  check = Data[0] + Data[1] + Data[2] + Data[3];
 if (check != Data[4]){
	 strcpy(buffer, "Loi du lieu DHT11\r\n");
 } return buffer;

    // Chuy?n d? li?u thành chu?i và g?i qua UART
    
    sprintf(buffer, "Nhiet do: %d.%d, Do am: %d.%d%%\r\n",
            Data[2], Data[3], Data[0], Data[1]);
    return buffer;
}


void DHT11_Check(void){
	   GPIO_ResetBits(GPIOB,GPIO_Pin_12); //keo xuong muc thap de bat dau truyen du lieu
		 Delay_MS(20);//>18ms
		 GPIO_SetBits (GPIOB,GPIO_Pin_12);//keo lai muc cao
	
	//cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<10){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 uint16_t u16Tim= TIM_GetCounter(TIM2);
		 if(u16Tim >=10){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 //cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<45){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=45)||(u16Tim<=5)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		 //cho chan PB12 len cao//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<90){
			 if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=90)||(u16Tim<=70)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		 //cho chan PB12 xuong thap//
		 TIM_SetCounter(TIM2, 0);
		 while (TIM_GetCounter(TIM2)<95){
			 if (!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) ){
           break;
			 }
		 }
		 u16Tim= TIM_GetCounter(TIM2);
		 if((u16Tim >=95)||(u16Tim<=75)){
			 while(1){//neu khong thoa man thi cho ngung khong chay nua
			 }
		 }
		 
		 
		 
		 
	
}


void DHT11_Init(void){
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//chan PB12 DE NHAN DU LIEU DHT11
	    GPIO_InitTypeDef gpioInit;
      gpioInit.GPIO_Mode = GPIO_Mode_Out_OD;
      gpioInit.GPIO_Pin = GPIO_Pin_12;
      gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	 
      GPIO_Init(GPIOB, &gpioInit);
	    GPIO_SetBits (GPIOB,GPIO_Pin_12); //giu o muc cao
	
	
}

void UART_SendChar(char c) {
    USART_SendData(USART3, c);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void delay(void){
	uint32_t i;
	for (i=0; i<0xfffff;++i){
	}
}
uint16_t ADC_Read(void);

/* Gui mot chuoi qua UART */
void UART_SendString(char *str) {
    while (*str) {
        USART_SendData(USART3, *str++);
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
    }
}

void GPIO_PWM_Config(void) {
    GPIO_InitTypeDef gpioInit;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    
    gpioInit.GPIO_Pin = GPIO_Pin_0; // PA0 - PWM1
    GPIO_Init(GPIOA, &gpioInit);
    
    gpioInit.GPIO_Pin = GPIO_Pin_1; // PA1 - PWM2
    GPIO_Init(GPIOA, &gpioInit);
    
    gpioInit.GPIO_Pin = GPIO_Pin_2; // PA2 - PWM3
    GPIO_Init(GPIOA, &gpioInit);
    
    gpioInit.GPIO_Pin = GPIO_Pin_3; // PA3 - PWM4
    GPIO_Init(GPIOA, &gpioInit);
}

void PWM1_Init(uint16_t pulse) {
    TIM_OCInitTypeDef pwmInit;
    pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
    pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
    pwmInit.TIM_OutputState = TIM_OutputState_Enable;
    pwmInit.TIM_Pulse = pulse;
    TIM_OC1Init(TIM2, &pwmInit);
}

void PWM2_Init(uint16_t pulse) {
    TIM_OCInitTypeDef pwmInit;
    pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
    pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
    pwmInit.TIM_OutputState = TIM_OutputState_Enable;
    pwmInit.TIM_Pulse = pulse;
    TIM_OC2Init(TIM2, &pwmInit);
}

void PWM3_Init(uint16_t pulse) {
    TIM_OCInitTypeDef pwmInit;
    pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
    pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
    pwmInit.TIM_OutputState = TIM_OutputState_Enable;
    pwmInit.TIM_Pulse = pulse;
    TIM_OC3Init(TIM2, &pwmInit);
}

void PWM4_Init(uint16_t pulse) {
    TIM_OCInitTypeDef pwmInit;
    pwmInit.TIM_OCMode = TIM_OCMode_PWM1;
    pwmInit.TIM_OCPolarity = TIM_OCPolarity_High;
    pwmInit.TIM_OutputState = TIM_OutputState_Enable;
    pwmInit.TIM_Pulse = pulse;
    TIM_OC4Init(TIM2, &pwmInit);
}

void TIM2_Init(void) {
    TIM_TimeBaseInitTypeDef timerInit;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    timerInit.TIM_CounterMode = TIM_CounterMode_Up;
    timerInit.TIM_Period = 0xFFFF;
    timerInit.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseInit(TIM2, &timerInit);
    TIM_Cmd(TIM2, ENABLE);
}

void UART_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE);//bat clock len
	
	GPIO_InitTypeDef gpioInit;// kieu struct
	/*PB10 tx*/
	gpioInit.GPIO_Mode = GPIO_Mode_AF_PP  ;//che do Output Push-Pull
	gpioInit.GPIO_Pin = GPIO_Pin_10;// Chon chan PC10
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	GPIO_Init(GPIOB,&gpioInit);//ham trong vi du
	/*PB11 rx*/
	gpioInit.GPIO_Mode = GPIO_Mode_IN_FLOATING   ;//che do dau ra
	gpioInit.GPIO_Pin = GPIO_Pin_11;// Chon chan PC11
	gpioInit.GPIO_Speed = GPIO_Speed_50MHz;// toc do 
	GPIO_Init(GPIOB,&gpioInit);//ham trong vi du
	
	USART_InitTypeDef usartInit;// kieu struct
  usartInit.USART_BaudRate = 9600;
  usartInit.USART_WordLength = USART_WordLength_8b;//chonj truyen 8 bit
  usartInit.USART_StopBits = USART_StopBits_1;//1 bit stop
  usartInit.USART_Parity = USART_Parity_No;//bit kiem trar chan le
  usartInit.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;//che do vua truyen vua nhan
  usartInit.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//che do bat tay, chon khong bat tay

  USART_Init(USART3, &usartInit);
  USART_Cmd(USART3, ENABLE);  // Bat USART3
	
	
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
	uint8_t u8Buffer=0;
	
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

// S? d?ng PA0 cho ADC (ADC_Channel_0)
void ADC_InitConfig(void) {
    // B?t clock cho ADC1 và GPIOA
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;       // Dùng PA0 cho ADC
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;     // Ch? d? Analog Input
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    ADC_InitTypeDef ADC_InitStructure;
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;    
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;       // Ch? d?c 1 kênh
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  // Chuy?n d?i liên t?c
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    
    // C?u hình kênh ADC: PA0 tuong ?ng v?i ADC_Channel_0, th?i gian m?u 55.5 cycles
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_55Cycles5);
    
    ADC_Cmd(ADC1, ENABLE);
    
    // Hi?u chu?n ADC
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

uint16_t ADC_Read(void) {
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}


 int main(void)
 {

//char dulieu[50];
    GPIO_InitTypeDef gpioInit;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    gpioInit.GPIO_Mode = GPIO_Mode_Out_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_13;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
	 
    GPIO_Init(GPIOC, &gpioInit);
	  
    //GPIO_PWM_Config();  
    //PWM1_Init(10);  // Duty cycle 10%
    //PWM2_Init(25);  // Duty cycle 25%
    //PWM3_Init(40);  // Duty cycle 40%
    //PWM4_Init(80);  // Duty cycle 80%
	 TIM2_Init();
	 UART_Init();
	 DHT11_Init();
	 
	 
	 while(1){
//		 GPIO_SetBits (GPIOC,GPIO_Pin_13);
//		 UART_SendString("Hello, UART!\r\n");
//		 USART_SendData(USART3, 0X19);
//		 while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET ){//USART_FLAG_TXE là co (flag) cho biet Data Register Empty (DR trong), tuc là du lieu dã duoc gui xong.
//		 }//neu co =0 thi tiep tuc cho, co bang 1 thoat vong cho
//		 Delay_MS(8000);
//		 GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//		 USART_SendData(USART3, 0X19);
//	   while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET ){
//		 }
//		 UART_SendString("Hello, UART!\r\n");
//		 Delay_MS(8000);
		 DHT11_Check();
		 char *dulieu = DHT11_Data();
		 UART_SendString(dulieu);
		 Delay_MS(2000); 
	   //strcpy(dulieu, DHT11_Data());
	  // UART_SendString(dulieu);
	 }
	 
	 
	 
	 
//	 while(1){
//		 if(USART_GetFlagStatus(USART3,USART_FLAG_RXNE)==SET){
//		   dulieu = USART_ReceiveData(USART3);
//			 USART_SendData(USART3, dulieu);
//		   while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET ){
//		 }
//		 }
//	 }

		 



 }
	 
