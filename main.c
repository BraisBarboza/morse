#include "MKL46Z4.h"
#include "lcd.h"

volatile int button = 0, count = 0;

void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); // 0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;
  for (i = 0; i < 5000000; i++);
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

void RTCSIntHandler(void){
  count += 1;
}

void RTC_reset()
{
  RTC->CR |= RTC_CR_SWR_MASK;
  RTC->CR &= ~RTC_CR_SWR_MASK;

  /* Set TSR register to 0x1 to avoid the timer invalid (TIF) bit being set in the SR register */
  RTC->TSR = 1U;
}

void RTC_ini()
{
  SIM->SOPT2 |= SIM_SOPT2_CLKOUTSEL(1); // 0 para 1 Hz, 1 para oscilador
  SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(1); // RTC_CLKIN
  SIM->SCGC6 |= SIM_SCGC6_RTC_MASK;
  NVIC_EnableIRQ(21);

  RTC->LR |= RTC_LR_LRL(1);
  RTC->LR |= RTC_LR_CRL(1);
  RTC->LR |= RTC_LR_TCL(1);

  //To clear the Time invalid Flag
  RTC->TSR = 1;
  /* Enable time seconds interrupts */
  RTC->IER |= RTC_IER_TSIE(1);

  /* Enable oscillator */
  RTC->CR = RTC_CR_OSCE(1);
  /* Setup the update mode and supervisor access mode */
  RTC->CR |= RTC_CR_UM(1);
  RTC->CR |= RTC_CR_SUP(1);
  RTC->CR |= RTC_CR_WPS(1);

  /* Configure the RTC time compensation register */
  RTC->TCR |= RTC_TCR_CIR(0);
  RTC->TCR |= RTC_TCR_TCR(0);
  
}

void RTC_start()
{
  RTC->SR |= RTC_SR_TCE_MASK; // Time counter is enabled.
}

void RTC_stop()
{
  RTC->SR &= ~RTC_SR_TCE_MASK; // Time counter is disabled.
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
   *      se non é o esperado "continue; // Volve a while(1)"
   *    (comprobar RTC + esperar botón SW3) * 3
   *      se non é o esperado "continue; // Volve a while(1)"
   *    (comprobar RTC + esperar botón SW1) * 3
   *      se non é o esperado "continue; // Volve a while(1)"
   *  lcd_SOS();
   *  delay();
   * lcd_clear();
   */

  RTC_ini();
  
  while(1){
    RTC_start();
    lcd_clear();
    lcd_blink(0);
    button = 0; // Reset botón pulsado

    /* Primeira S */
    /* Punto 1 */
    while (button != 1);
    count = 0; // Iníciase conta de novo
    button = 0; // Reset botón pulsado
    
    /* Punto 2 */
    while (button == 0);
    if (count > 1 || button != 1){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }

    /* Punto 3 */
    while (button == 0);
    if (count > 1 || button != 1){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }
    lcd_set(5,2);
    lcd_blink(1);

    /* Primeira O */
    /* Liña 1 */
    while (button == 0){
      if (count == 1)
        lcd_blink(0);
    }
    if (count > 6 /*|| count < 1*/ || button != 2){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }
    lcd_blink(0); // Por se actualiza count inesperadamente.

    /* Liña 2 */
    while (button == 0);
    if (count > 1 || button != 2){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }

    /* Liña 3 */
    while (button == 0);
    if (count > 1 || button != 2){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }
    lcd_set(0,3);
    lcd_blink(1);

    /* Segunda S */
    /* Punto 1 */
    while (button == 0){
      if (count == 1)
        lcd_blink(0);
    }
    if (count > 6 /*|| count < 1*/ || button != 1){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }
    lcd_blink(0); // Por se actualiza count inesperadamente.
    
    /* Punto 2 */
    while (button == 0);
    if (count > 1 || button != 1){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }

    /* Punto 3 */
    while (button == 0);
    if (count > 1 || button != 1){
      button = 0; // Reset botón pulsado
      continue; // Volve a while(1)
    }else{
      button = 0; // Reset botón pulsado
      count = 0; // Iníciase conta de novo
    }
    lcd_set(5,4);
    lcd_blink(1);
    delay();
  }
  return 0;
}
