#include "main.h"
#include "printf.h"

uint16_t adcResults[ADC_BUFFER_SIZE];

static volatile bool samplesReady;
static volatile uint32_t ticks;

static float fftInput[ADC_BUFFER_SIZE], fftOutput[ADC_BUFFER_SIZE];
static arm_rfft_fast_instance_f32 fftInstance;

void _init() { }

ISR_Handler SysTick_Handler(void) {
	ticks++;
}

ISR_Handler DMA1_Channel1_IRQHandler(void) {
	// Shut off the alarm clock
	DMA1->IFCR = DMA_IFCR_CGIF1;
	samplesReady = true;
}

ISR_Handler ADC1_2_IRQHandler(void) {
	// Shut off the alarm clock and the interrupt enable
	ADC1->IER &= ~ADC_IER_ADRDYIE;
	ADC1->ISR = ADC_ISR_ADRDY;
	// Start ADC
	ADC1->CR |= ADC_CR_ADSTART;
}

static void doFFT() {
	// Convert ADC data to floats
	float *fftInputPointer = &fftInput[0], max = 0.0f;
	uint32_t freq = 0U;
	uint16_t *adcResultPointer = &adcResults[0];
	for (uint32_t i = ADC_BUFFER_SIZE; i; i--)
		*fftInputPointer++ = *adcResultPointer++;
	// Real FFT alternates the magnitude and phase
	arm_rfft_fast_f32(&fftInstance, fftInput, fftOutput, 0U);
	// Compute magnitude of each component
	arm_cmplx_mag_f32(fftOutput + 2, fftInput, (ADC_BUFFER_SIZE >> 1) - 1);
	// Go through the first half of the input and find the maximum value
	arm_max_f32(fftInput, (ADC_BUFFER_SIZE >> 1) - 1, &max, &freq);
	printf("%d at %d\r\n", (int)max, (int)((freq + 1) * 4688));
}

int main(void) {
	uint32_t divider = 0U;
	ticks = 0U;
	__enable_fault_irq();
	__enable_irq();
	arm_rfft_fast_init_f32(&fftInstance, ADC_BUFFER_SIZE);
	while (1) {
		if (samplesReady) {
			// ADC samples are ready!
			samplesReady = false;
			ioSetOutput(PIN_LED_2, divider < 512U);
			divider++;
			if (divider >= 1024U) {
				doFFT();
				divider = 0U;
			}
		}
		uint32_t time = ticks;
		ioSetOutput(PIN_LED_1, (time % 2000U) < 1000U);
		__WFI();
	}
	return 0;
}
