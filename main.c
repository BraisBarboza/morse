#include "MKL46Z4.h"
#include "lcd.h"

// LED (RG)
// LED_GREEN = PTD5 (pin 98)
// LED_RED = PTE29 (pin 26)

// SWICHES
// RIGHT (SW1) = PTC3 (pin 73)
// LEFT (SW2) = PTC12 (pin 88)

// Enable IRCLK (Internal Reference Clock)
// see Chapter 24 in MCU doc
unsigned volatile int was_sw1_pressed=0;
unsigned volatile int was_sw2_pressed=0;
unsigned int par=0;
unsigned int miss=0;
unsigned int hits=0;
void irclk_ini()
{
  MCG->C1 = MCG_C1_IRCLKEN(1) | MCG_C1_IREFSTEN(1);
  MCG->C2 = MCG_C2_IRCS(0); //0 32KHZ internal reference clock; 1= 4MHz irc
}

void delay(void)
{
  volatile int i;

  for (i = 0; i < 1000000; i++);
}

// RIGHT_SWITCH (SW1) = PTC3
void sw1_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3);
}
int sw1_check()
{
  return( !(GPIOC->PDIR & (1 << 3)) );
}

int sw2_check()
{
  return( !(GPIOC->PDIR & (1 << 12)) );
}
void PORTDIntHandler(void)
{
  PORTC->ISFR = 0xFFFFFFFF; // Clear IRQ
    if(sw1_check()){
    was_sw1_pressed=1;
    }
    while(sw1_check());
    if(sw2_check()){
    was_sw2_pressed=1;
    }
    while(sw2_check());
    if ((par && was_sw1_pressed)||(!par && was_sw2_pressed)){
      hits = hits+1;
    }else miss=miss+1;
    lcd_display_time(hits, miss);

}
void sw1_ini_irq()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge

  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}
void sws_init()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);

  PORTC->PCR[3] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  PORTC->PCR[12] |= PORT_PCR_IRQC(0xA); // IRQ on falling edge
  
  // IRQ#31: Pin detect for PORTS C & D
  NVIC_SetPriority(31, 0); // Max priority for IRQ#31
  NVIC_EnableIRQ(31);      // Enable IRQ#31
}
// LEFT_SWITCH (SW2) = PTC12
void sw2_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 12);
}



// RIGHT_SWITCH (SW1) = PTC3
// LEFT_SWITCH (SW2) = PTC12
void sws_ini()
{
  SIM->COPC = 0;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  PORTC->PCR[3] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  PORTC->PCR[12] |= PORT_PCR_MUX(1) | PORT_PCR_PE(1);
  GPIOC->PDDR &= ~(1 << 3 | 1 << 12);
}

// LED_GREEN = PTD5
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

// LED_RED = PTE29
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

// LED_RED = PTE29
// LED_GREEN = PTD5
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

// Hit condition: (else, it is a miss)
// - Left switch matches red light
// - Right switch matches green light

int main(void)
{
  irclk_ini(); // Enable internal ref clk to use by LCD
  lcd_ini();
  led_green_ini();
  led_red_ini();
  sws_init();
  int red_status=0;
  int green_status=0;
  // 'Random' sequence :-)
  volatile unsigned int sequence = 0x32B14D98,
    index = 0;

  while (index < 32) {
      if (sequence & (1 << index)) { //odd
        par=1;
        //
        // Switch on green led
        // [...]
        //
        if (red_status)
        {
          red_status=!red_status;
          led_red_toggle();
        }
        
        if (!green_status)
        {
          green_status=!green_status;
          led_green_toggle();
        }
      while(!(was_sw2_pressed||was_sw1_pressed)){
      }
      was_sw1_pressed=0;
      was_sw2_pressed=0;
      } else {
        par=0;
        //even
        //
        // Switch on red led
        // [...]
        //
        if (green_status)
        {
          green_status=!green_status;
          led_green_toggle();
        }
        
        if (!red_status)
        {
          red_status=!red_status;
          led_red_toggle();
        }
      while(!(was_sw2_pressed||was_sw1_pressed)){
      }
      was_sw1_pressed=0;
      was_sw2_pressed=0;
    }
    // [...] 
    index=index+1;
}
  NVIC_DisableIRQ(31);
  LCD->AR|=LCD_AR_BLINK(1);
  // Stop game and show blinking final result in LCD: hits:misses
  // [...]


  return 0;
}
