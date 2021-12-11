#include "MKL46Z4.h"
#include "lcd.h"

typedef enum _rtc_status_flags
{
  kRTC_TimeInvalidFlag = (1U << 0U),  /*!< Time invalid flag */
  kRTC_TimeOverflowFlag = (1U << 1U), /*!< Time overflow flag */
  kRTC_AlarmFlag = (1U << 2U),        /*!< Alarm flag*/
} rtc_status_flags_t;

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
    button = 1; // · (S)
  }
  else /* Interrupt on SW3 detected */
  {
    // Para evitar que se execute constantemente a interrupción.
    PORTC->PCR[12] |= PORT_PCR_ISF_MASK;
    button = 2; // - (O)
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

void RTC_ini()
{
  SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(0); // 1 Hz
  SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(1); // RTC_CLKIN

  uint32_t updateMode = 1, supervisorAccess = 1, wakeupSelect = 0, compensationInterval = 0, compensationTime = 0;
  uint32_t reg;
  reg = RTC->CR;
  /* Setup the update mode and supervisor access mode */
  reg |= RTC_CR_UM(updateMode) | RTC_CR_SUP(supervisorAccess);
  reg |= RTC_CR_WPS(wakeupSelect);

  RTC->CR = reg;

  /* Configure the RTC time compensation register */
  RTC->TCR = (RTC_TCR_CIR(compensationInterval) | RTC_TCR_TCR(compensationTime));
  /* Enable time seconds interrupts */
  RTC->IER |= RTC_IER_TSIE(1);
}

void RTC_start()
{
  RTC->SR |= RTC_SR_TCE_MASK; // Time counter is enabled.
}

void RTC_stop()
{
  RTC->SR &= ~RTC_SR_TCE_MASK; // Time counter is disabled.
}

void RTC_reset()
{
  RTC->CR |= RTC_CR_SWR_MASK;
  RTC->CR &= ~RTC_CR_SWR_MASK;

  /* Set TSR register to 0x1 to avoid the timer invalid (TIF) bit being set in the SR register */
  RTC->TSR = 1U;
}

int main(void)
{
  irclk_ini();
  lcd_ini();
  led_green_ini();
  led_red_ini();
  buttons_ini();

  /**
   * Cada vez que se pulse un botón débese comprobar e reiniciar o RTC.
   *
   * Pseudocódigo:
   *
   * while 1
   *  lcd_clear();
   *  esperar botón SW1
   *    (comprobar RTC + esperar botón SW1) * 2
   *      se non é o esperado "continue;"
   *    (comprobar RTC + esperar botón SW3) * 3
   *      se non é o esperado "continue;"
   *    (comprobar RTC + esperar botón SW1) * 3
   *      se non é o esperado "continue;"
   *  lcd_SOS();
   *  delay();
   * lcd_clear();
   */

  RTC_ini();
  RTC_start();
  RTC_stop();
  RTC_reset();

  return 0;
}
