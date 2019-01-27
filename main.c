#include "asf.h"
#include "buttons.h"
#include "lcd.h"
#include "SSD1306_commands.h"


/** Set default LED blink period to 250ms*3 */
#define DEFAULT_LED_FREQ   4

/** LED blink period */
#define LED_BLINK_PERIOD    3

/** LED blink period */
static volatile uint32_t led_blink_period = 0;

/**
 *  \brief Handler for System Tick interrupt.
 *
 *  Process System Tick Event
 */
void SysTick_Handler(void)
{
}

/**
 * \brief Interrupt handler for TC0 interrupt. Toggles the state of LEDs.
 */
void TC0_Handler(void)
{
    /* Clear status bit to acknowledge interrupt */
    tc_get_status(TC0, 0);

    led_blink_period++;

    if (led_blink_period == LED_BLINK_PERIOD) {
        int buttons = btn_state();

        if(buttons)
            uart_write(UART0, buttons + 0x30);

        pio_toggle_pin(PIO_PA8_IDX);
        led_blink_period = 0;
    }
}

/**
 * \brief Configure Timer Counter 0 to generate an interrupt with the specific
 * frequency.
 *
 * \param freq Timer counter frequency.
 */
static void configure_tc(uint32_t freq)
{
    uint32_t ul_div;
    uint32_t ul_tcclks;
    uint32_t ul_sysclk = sysclk_get_cpu_hz();

    /* Disable TC first */
    tc_stop(TC0, 0);
    tc_disable_interrupt(TC0, 0, TC_IER_CPCS);

    /** Configure TC with the frequency and trigger on RC compare. */
    tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
    tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
    tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / 4);

    /* Configure and enable interrupt on RC compare */
    NVIC_EnableIRQ((IRQn_Type)ID_TC0);
    tc_enable_interrupt(TC0, 0, TC_IER_CPCS);

    /** Start the counter. */
    tc_start(TC0, 0);
}

static void configure_console(void)
{
    sam_uart_opt_t uart_settings;
    uart_settings.ul_mck = sysclk_get_peripheral_hz();
    uart_settings.ul_baudrate = 115200Ul;
    uart_settings.ul_mode = UART_MR_PAR_NO;

    pio_configure(PIOA, PIO_PERIPH_A, (PIO_PA9A_URXD0 | PIO_PA10A_UTXD0), PIO_DEFAULT);
    sysclk_enable_peripheral_clock(ID_UART0);
    uart_init(UART0, &uart_settings);
}

/**
 * \brief Configure timer ISR to fire regularly.
 */
static void init_timer_isr(void)
{
    SysTick_Config((sysclk_get_cpu_hz() / 1000) * 100); // 100 ms
}

/**
 * \brief initialize pins, watchdog, etc.
 */
static void init_system(void)
{
    /* Disable the watchdog */
    wdt_disable(WDT);

    /* Initialize the system clock */
    sysclk_init();

    /* Configure LED pins */
    pio_configure(PIOA, PIO_OUTPUT_0, (PIO_PA7 | PIO_PA8), PIO_DEFAULT);

    /* Enable PMC clock for key/slider PIOs  */
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);

    /* Configure PMC */
    pmc_enable_periph_clk(ID_TC0);

    /* Configure the default TC frequency */
    configure_tc(DEFAULT_LED_FREQ);

    /* Enable the serial console */
    configure_console();

    btn_init();
    lcd_init();
}

/**
 *  \brief getting-started Application entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */

uint8_t cmds[1]={SSD1306_DISPLAYALLON};
uint8_t mode[3]={0x00, SSD1306_MEMORYMODE, SSD1306_HORIZONTAL};
uint8_t columnAddress[4]={0x00, SSD1306_COLUMNADDR, 0x00, 0x10};
uint8_t pageAddress[]={0xB4, SSD1306_SETLOWCOLUMN, 0x03,SSD1306_SETHIGHCOLUMN,0x16};
uint8_t address[]={SSD1306_COLUMNADDR, 0x00, 0x00, SSD1306_PAGEADDR, 0x00, 0x00};

uint8_t init[] = {0x00,
	0xAE,		// display off
	0xA8, 0x3F,	// mux ratio
	0xD3, 0x00,	// display offset
	0x40,		// display start line
	0xA0,		// segment re-map
	0xC0, 		// com output scan direction
	0xDA, 0x12, // com pins hw configuration
	0x81, 0x7F, // contrast control
	0xA4,		// disable entire display on
	0xA6, 		// normal display
	0xD5, 0x80, // osc frequency
	0x8D, 0x14, // en charge pump regulator
	0xAF, 0xB1		// display on
};
uint8_t init2[] = {0x00,0xae,0xa8,0x3f,0xd3,0x00,0x40,0xa0,0xc0,
	      0xda,0x12,0x81,0xff,0xa4,0xa6,0xd5,0x80,0x8d,0x14,
	0xaf,0x20,0x00};


uint8_t buffer[128]={0xff,};

int main(void)
{
//	uint8_t buffer[1*51+1]={0xff,};
//	buffer[0]=0x40;

	/* Initialize the SAM system */
    init_system();

    /* Configure timer ISR to fire regularly */
    init_timer_isr();

    uart_write(UART0, 'U');

    uint8_t siema[5]= "siema";
    SSD1306_setString(0,0,siema, 5,1);

    SSD1306_drawBitmap();

    /* Loop forever */
    for (;;) {
        pio_toggle_pin(PIO_PA7_IDX);
        for(uint16_t i = 0; i < 65535; ++i) __NOP;
    }
}
