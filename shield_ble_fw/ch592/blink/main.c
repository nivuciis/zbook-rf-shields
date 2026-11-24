#include <stdint.h>

/* Registradores da porta A (CH592SFR.h) */
#define R32_PA_DIR    (*(volatile uint32_t *)0x400010A0)
#define R32_PA_OUT    (*(volatile uint32_t *)0x400010A8)
#define R32_PA_CLR    (*(volatile uint32_t *)0x400010AC)
#define R32_PA_PD_DRV (*(volatile uint32_t *)0x400010B4)

/*
 * PA4: ligue com um jumper no LED2 do kit (o LED acende em nivel baixo).
 * PA9 (UART1_TX, ligado no RX da zbook) pisca junto, para conferir pela zbook
 * que o programa esta rodando.
 */
#define LED_PIN ((1u << 4) | (1u << 9))

static void delay(void)
{
	for (volatile uint32_t i = 0; i < 40000; i++) {
	}
}

int main(void)
{
	R32_PA_PD_DRV &= ~LED_PIN; /* saida push-pull 5 mA */
	R32_PA_DIR |= LED_PIN;

	while (1) {
		R32_PA_CLR = LED_PIN; /* acende */
		delay();
		R32_PA_OUT |= LED_PIN; /* apaga */
		delay();
	}
}
