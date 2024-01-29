#include "stm32f10x.h"

uint32_t __msTick = 0;    // khai báo bien dem cho moi lan ngat systick
void delay_ms(uint32_t ms); // khai bao ham delay theo don vi ms
void SysTick_Handler(void); // khai bao ham ngat systick

int main(void)
{
  /* ___SYSTICK___ */

  /* Cau hình Systick 1ms */
  SysTick_Config(SystemCoreClock / 1000);   // khoi tao systick voi tan so 1000Hz tuong ung 1ms

  /* Cau hình muc do uu tien ngat SysTick */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0)); // dat muc do uu tien ngat

  /* ___GPIO___ */

  /* Bat Clock PortC */
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // 1: Enable

  /* Set PC13 */
  GPIOC->BSRR |= GPIO_BSRR_BS13;

  /* Cau hinh PC13 */
  GPIOC->CRH &= ~GPIO_CRH_MODE13; // Clear
  GPIOC->CRH |= GPIO_CRH_MODE13_1; // 10: Output mode, max speed 2 MHz
  GPIOC->CRH &= ~GPIO_CRH_CNF13; // 00: General purpose output push-pull
	// cau hinh chan PC13 noi voi LED


  while (1)
  {
    /* Reset PC13 */
    GPIOC->ODR ^=(1<<13);
    delay_ms(500);  // delay 500ms va toggle PC13
  }
}

/* Hàm delay ms su dung SysTick */
void delay_ms(uint32_t ms)
{
  uint32_t startTick = __msTick;
  while (__msTick - startTick < ms);  // viet ham delay su dung bien dem __msTick de so sanh 
}

/* Trình phuc vu ngat SysTick */
void SysTick_Handler(void)
{
  /* Tang bien tick */ 
  __msTick++; // moi lan ngat se dem len bien __msTick 
}