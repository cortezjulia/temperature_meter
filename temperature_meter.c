#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"

#define GPIO_PA0_U0RX 0x00000001
#define GPIO_PA1_U0TX 0x00000401

#define GPIO_PA6_M1PWM2 0x00001805
//#define GPIO_PE4_M1PWM1 0x00041005

#define PF2 (*((volatile long *) 0x40025010))

int K,C;
int C_ATUAL=1000;
uint32_t ADC0Value[4], volt0,volt1, volt2, volt3,dec_volt,temp,MEDIA,volt_medio;
uint32_t temp_atual=25;
char s_temp[7];
float DC_BASE=0.0000;
void print_float( float var);
void config_timer(void);
void WTimer5A_IntHandler(void) //ISR 
{
  PF2=0x04;
  
  TimerIntClear(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);
  
}
void CONFIGURACAO_PWM(void);
void config_PWM(float DC);

float T=0.0;
void Config_UART0 (void) 
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA0_U0RX);
  GPIOPinConfigure(GPIO_PA1_U0TX);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
  UARTStdioConfig(0, 115200, 16000000); 
}
int main (void)
{
  
  Config_UART0();
  //SysCtlClockSet(SYSCTL_SYSDIV_1|SYSCTL_USE_OSC|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
  
  SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
  UARTprintf("\x1B[2J\x1B[0;0H");
  CONFIGURACAO_PWM();
  UARTprintf("-=[ VOLTÍMETRO COM ADC ]=-\n\n");
  config_timer();
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
  ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0);
  ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH3);
  ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH3);
  ADCSequenceStepConfigure(ADC0_BASE, 2, 2, ADC_CTL_CH3);
  ADCSequenceStepConfigure(ADC0_BASE, 2 ,3, ADC_CTL_CH3|ADC_CTL_IE|ADC_CTL_END);
  ADCHardwareOversampleConfigure(ADC0_BASE, 64); //faça um teste sem esta instrução
  ADCSequenceEnable(ADC0_BASE, 2);
  
  while(1)
  {
    //ADCProcessorTrigger(ADC0_BASE, 2);
    
    
    while(ADCIntStatus(ADC0_BASE, 2, false)==0);
    
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE,2, ADC0Value);
    volt0=ADC0Value[0]*33/40950;
    volt1=ADC0Value[1]*33/40950;
    volt2=ADC0Value[2]*33/40950;
    volt3=ADC0Value[3]*33/40950;
    
    MEDIA=(ADC0Value[0]+ADC0Value[1]+ADC0Value[2]+ADC0Value[3])/4;
    volt_medio=(MEDIA*3300/4096);
    dec_volt=(MEDIA*3300/4095)- volt_medio*1000;
    
    PF2=0x00;
    SysCtlDelay(3000000);
    K = (volt_medio/10)+43;
    C=(K - 273);
    
    /***opção para mais precisão e faixa fixa de variação***/   
    /*    switch (C)
    {
  case 25: DC_BASE=119.00; UARTprintf("\rDC atual: 50%");config_PWM((DC_BASE));print_float(DC_BASE);
    
    break;
  case 26: DC_BASE=121.3; UARTprintf("\rDC atual: 51%%");config_PWM((DC_BASE));print_float(DC_BASE);
    break;
  case 27: DC_BASE=123.76; UARTprintf("\rDC atual: 52%%");config_PWM((DC_BASE));print_float(DC_BASE);
    break;
  case 28: DC_BASE=126.14; UARTprintf("\rDC atual: 53%%");config_PWM((DC_BASE));print_float(DC_BASE);
    break;
  case 29: DC_BASE=128.61; UARTprintf("\rDC atual: 54%%");config_PWM((DC_BASE));print_float(DC_BASE);
    break;
  case 30: DC_BASE=131.13; UARTprintf("\rDC atual: 55%%");config_PWM((DC_BASE));print_float(DC_BASE);
    break;
  }*/
    
    if(C!=C_ATUAL)
    {
      
      UARTprintf("\x1B[2J\x1B[0;0H");
      UARTprintf("\n%d CELSIUS -------- %d KELVIN",C,K);
      if(C==25)
      {
        UARTprintf("\n\n\nTemperatura atual igual a 25 ºC...");
        DC_BASE=119.00;
        
        //UARTprintf("\n\rDC atual: %d", (((DC_BASE*1.0196)*50)/119));
      }
      else if(C<25)
      {
        UARTprintf("\n\n\nTemperatura atual está abaixo de 25 ºC...");
        DC_BASE=119-((54.7-C)*(238/100));
        
        //UARTprintf("\n\rDC atual: %d", (((DC_BASE*1.0196)*50)/119));
      }
      else if(C>25)
      {
        UARTprintf("\n\n\nTemperatura atual está acima de 25 ºC...");
        DC_BASE=119+((C-54.7)*(238/100));
        
        //UARTprintf("\n\rDC atual: %d", (((DC_BASE*1.0196)*50)/119));
      }
      C_ATUAL=C;
      config_PWM((DC_BASE));
      UARTprintf("\nValor de DC carregado: ");
      print_float(DC_BASE);
      UARTprintf("   ~   ");
      //UARTprintf("\n\rDC: %d, temp = %d",DC_BASE,C);
      print_float(((DC_BASE*100.0000)/238.0000));UARTprintf("%%");
    }
    SysCtlDelay(3000000);
    
  }
}

void CONFIGURACAO_PWM(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_4); //clock do PWM
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //PA6
  GPIOPinConfigure(GPIO_PA6_M1PWM2); //PWM1
  
  GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
  PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
  
  PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, 238); //Período ? máx. (2^16)
  
  
  PWMGenEnable(PWM1_BASE, PWM_GEN_1); //habilita o gerador 0.
  PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true); //habilita a saída 0 do PWM0 (PA6).
}
void config_PWM(float DC)

{
  PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, DC); //tH da saída 0
}

void config_timer(void)
{
  ////Configuração da GPIO F ////
  ////Configuração da GPIO F ////
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_6);
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WTIMER5);
  TimerConfigure(WTIMER5_BASE, TIMER_CFG_A_PERIODIC);
  TimerLoadSet(WTIMER5_BASE, TIMER_A, 30000000-1); //valor em us
  TimerControlTrigger(WTIMER5_BASE,TIMER_A,true);
  TimerIntEnable(WTIMER5_BASE, TIMER_TIMA_TIMEOUT);
  IntEnable(INT_WTIMER5A_TM4C123);
  IntRegister(INT_WTIMER5A_TM4C123, WTimer5A_IntHandler);
  TimerEnable(WTIMER5_BASE, TIMER_A); //inicia
  
  PF2=0x04;
}
void print_float( float var)
{
  int x1=0,x2=0,x3=0;
  x1=(int)var;
  if(x1!= 0)
  {
    x2=(var*10)-((int)var)*10;
    x2=fabs(x2);
    x3=(var*100)-(int)var*100;
    x3=fabs(x3);
    UARTprintf("%d.%d%d",x1,x2,x3);
    
  }
  else
  {
    x1=0;
    x2=(var*10);
    x2=fabs(x2);
    x3=(var*100)-(int)var*100;
    x3=fabs(x3);
    if(var<0)
    {
      UARTprintf("-%d.%d%d",x1,x2,x3);
    }
    else
    {
      UARTprintf("%d.%d%d",x1,x2,x3);
    }
    
  }
  
  
  
}



