#include "MKL46Z4.h"
#include "lcd.h"
#include "fsl_rtc.h"

volatile int button = 0;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); // 0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++)
    ;
}

void PORTC_PORTD_IRQHandler(void)
{
  /* Interrupt on SW1 detected */
  if (PORTC->PCR[3] & PORT_PCR_ISF_MASK)
  {
    // Para evitar que se execute constantemente a interrupción.
    PORTC->PCR[3] |= PORT_PCR_ISF_MASK;
    button = 1; // ·
  }
  else /* Interrupt on SW3 detected */
  {
    // Para evitar que se execute constantemente a interrupción.
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
    button = 2; // -
  }
}

void buttons_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA);  // IRQ on falling edge
  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge

  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}

void led_green_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOD->PSOR = (1 << 5);
}

void led_green_toggle()
{
  GPIOD->PTOR = (1 << 5);
}

void led_red_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOE->PDDR |= (1 << 29);
  GPIOE->PSOR = (1 << 29);
}

void led_red_toggle(void)
{
  GPIOE->PTOR = (1 << 29);
}

void leds_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;
  PORTD->PCR[5] = PORT_PCR_MUX(1);
  PORTE->PCR[29] = PORT_PCR_MUX(1);
  GPIOD->PDDR |= (1 << 5);
  GPIOE->PDDR |= (1 << 29);
  // both LEDS off after init
  GPIOD->PSOR = (1 << 5);
  GPIOE->PSOR = (1 << 29);
}

int main(void)
{
  irclk_ini();
  lcd_ini();
  led_green_ini();
  led_red_ini();
  buttons_ini();
  
  rtc_config_t *config = malloc(sizeof(rtc_config_t));
  config->wakeupSelect = false;
  config->updateMode = true;
  config->supervisorAccess = true;

  //RTC_Init((RTC_Type *)RTC->CR, config);
  // RTC_StartTimer(RTC_BASE);
  // RTC_Reset(RTC_BASE);



  return 0;
}
