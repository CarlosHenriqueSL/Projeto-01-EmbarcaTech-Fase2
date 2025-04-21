/*
 * Lê o joystick (eixo X), exibe num OLED SSD1306, controla LEDs WS2812,
 * gera feedback sonoro e luminoso, e envia dados pela UART0.
 * Ao retornar ao centro e pressionar o botão do joystick, registra o valor maximo do adc joystick
 * alcançado desde o último retorno ao centro.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "inc/ssd1306.h"
#include "numeros/numeros.h"
#include "blink.pio.h"

// Pinagem e configurações
#define WS2812_PIN 7
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C

#define LED_PIN_RED 13 // PWM para LED vermelho
#define BUZZER_PIN 21  // PWM para buzzer

// Joystick ADC (GPIO27 corresponde ao ADC1)
#define JOY_X_ADC 1
#define JOY_GPIO_ADX 27

// Botões físicos
#define JOY_BUTTON_PIN 22
#define BUTTON_A 5
#define BUTTON_B 6
#define DEBOUNCE_MS 400 // tempo mínimo entre transições

// PWM e buzzer
#define PWM_WRAP 4095u
#define BUZZER_FREQ_MIN 200.0
#define BUZZER_FREQ_MAX 2000.0

// Centro e deadzone do joystick
#define JOY_CENTER 2048
#define JOY_DEADZONE 40

// UART0
#define UART_ID uart0
#define UART_BAUD 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1

//===== Variáveis globais =====
static volatile bool sistema_ligado = true;
static volatile bool feedback_pending = false;
static volatile double valor_atual_do_adc = 0.0; // última leitura normalizada (0.0–1.0)
static volatile double valor_maximo_do_adc = 0.0;
static uint8_t porcentagem_atual = 0; // percentuais de 0 a 100

// debounce de cada botão
static uint32_t last_irq_time_joy = 0;
static uint32_t last_irq_time_a = 0;
static uint32_t last_irq_time_b = 0;

// Inicializa UART0 para comunicação
void init_uart(void)
{
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Inicializa I2C, ADC, PWM e demais periféricos
void init_peripherals(void)
{
    // I2C para OLED
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // ADC para joystick
    adc_init();
    adc_gpio_init(JOY_GPIO_ADX);

    // PWM para LED e buzzer
    gpio_set_function(LED_PIN_RED, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_led = pwm_gpio_to_slice_num(LED_PIN_RED);
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice_led, PWM_WRAP);
    pwm_set_wrap(slice_buzzer, PWM_WRAP);
    pwm_set_enabled(slice_led, true);
    pwm_set_enabled(slice_buzzer, false);
}

// Desliga display, LEDs e buzzer
void desligar(ssd1306_t *ssd, PIO pio, uint sm, bool *prev_on,
              uint slice_led, uint slice_buzzer)
{
    pwm_set_gpio_level(LED_PIN_RED, 0);
    pwm_set_enabled(slice_buzzer, false);
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);
    for (int i = 0; i < NUM_PIXELS; i++)
        pio_sm_put_blocking(pio, sm, 0);
    *prev_on = false;
    sleep_ms(100);
}

// Constrói valor 32-bit GRB para WS2812
uint32_t matrix_rgb(double r, double g, double b)
{
    unsigned char R = r * 255;
    unsigned char G = g * 255;
    unsigned char B = b * 255;
    // WS2812 espera os dados no formato GRB nos 24 bits mais significativos
    return (G << 24) | (R << 16) | (B << 8);
}

// Envia padrão para LEDs WS2812
void desenho_pio(double *pattern, PIO pio, uint sm)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        uint32_t val = matrix_rgb((uint8_t)(pattern[24 - i] * 255), 0, 0);
        pio_sm_put_blocking(pio, sm, val);
    }
}

// Seleciona padrão baseado na porcentagem
double *definir_desenho()
{
    switch (porcentagem_atual)
    {
    case 20:
        return vintePorcento;
    case 40:
        return quarentaPorcento;
    case 60:
        return sessentaPorcento;
    case 80:
        return oitentaPorcento;
    case 100:
        return cemPorcento;
    default:
        return zeroPorcento;
    }
}

// Interrupção com debounce
void gpio_irq_handler(uint gpio, uint32_t events)
{
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (gpio == JOY_BUTTON_PIN)
    {
        if (now - last_irq_time_joy < DEBOUNCE_MS)
            return;
        last_irq_time_joy = now;
        if (events & GPIO_IRQ_EDGE_FALL)
        {
            double total = valor_maximo_do_adc;
            printf("Valor maximo lido: %.3f\n", total);
            if (total <= 0.1)
                porcentagem_atual = 0;
            else if (total <= 0.2)
                porcentagem_atual = 20;
            else if (total <= 0.4)
                porcentagem_atual = 40;
            else if (total <= 0.6)
                porcentagem_atual = 60;
            else if (total <= 0.8)
                porcentagem_atual = 80;
            else
                porcentagem_atual = 100;
            feedback_pending = true;
            valor_maximo_do_adc = 0.0;
        }
    }
    else if (gpio == BUTTON_A)
    {
        if (now - last_irq_time_a < DEBOUNCE_MS)
            return;
        last_irq_time_a = now;
        sistema_ligado = !sistema_ligado;
    }
    else if (gpio == BUTTON_B)
    {
        if (now - last_irq_time_b < DEBOUNCE_MS)
            return;
        last_irq_time_b = now;
        porcentagem_atual = 0;
    }
}

// Inicializa pinos GPIO (LED, buzzer e botões)
void init_gpio(void)
{
    // LED e buzzer
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Botões com pull-up e interrupção
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);
    gpio_init(JOY_BUTTON_PIN);
    gpio_set_dir(JOY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(JOY_BUTTON_PIN);

    // Registra interrupções para todos os botões (primeiro define callback)
    gpio_set_irq_enabled_with_callback(JOY_BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
}

int main(void)
{
    // Inicializa USB-serial para debug e UART0
    stdio_init_all();
    init_uart();

    // Inicializa GPIO, I2C, ADC, PWM e display
    init_gpio();
    init_peripherals();

    // Configuração inicial display SSD1306
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Variável para controle do loop
    bool prev_on = false;

    // Configuração do PIO
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    uint sm = pio_claim_unused_sm(pio, true);
    blink_program_init(pio, sm, offset, WS2812_PIN);

    // PWM slices
    uint slice_led = pwm_gpio_to_slice_num(LED_PIN_RED);
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);

    // Loop principal
    while (true)
    {
        if (!sistema_ligado)
        {
            desligar(&ssd, pio, sm, &prev_on, slice_led, slice_buzzer);
            sleep_ms(100);
            continue;
        }

        if (!prev_on)
        {
            pwm_set_gpio_level(LED_PIN_RED, (uint32_t)(valor_maximo_do_adc * PWM_WRAP));
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Sistema iniciado", 4, 25);
            ssd1306_send_data(&ssd);
            prev_on = true;
            sleep_ms(1000);
        }

        // Limpa display e desenha o retângulo da borda
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 0, 0, ssd.width, ssd.height, true, false);

        // Leitura ADC do joystick
        adc_select_input(JOY_X_ADC);
        uint16_t adc_x = adc_read();

        // Desenha um quadrado móvel
        uint8_t square_x = (adc_x * (ssd.width - 8)) / 4095;
        ssd1306_rect(&ssd, 31, square_x, 8, 8, true, true);
        ssd1306_send_data(&ssd);

        // Normaliza leitura e aplica deadzone
        int delta = abs((int)adc_x - JOY_CENTER);
        if (delta < JOY_DEADZONE)
            delta = 0;
        valor_atual_do_adc = (double)delta / JOY_CENTER;
        if (valor_atual_do_adc > valor_maximo_do_adc)
        {
            valor_maximo_do_adc = valor_atual_do_adc;
        }

        // Atualiza LEDs WS2812
        double *desenho = definir_desenho();
        desenho_pio(desenho, pio, sm);

        // Envia dados pela UART
        char message[64];
        sprintf(message, "Proximidade: %f\n", valor_atual_do_adc);
        uart_puts(UART_ID, message);
        

        // Feedback se solicitado pelo botão do joystick
        if (feedback_pending)
        {
            // LED vermelho proporcional ao valor do adc
            pwm_set_gpio_level(LED_PIN_RED, (uint32_t)(valor_maximo_do_adc * PWM_WRAP));
            // Buzzer com tom decrescente
            double start = BUZZER_FREQ_MIN + valor_maximo_do_adc * (BUZZER_FREQ_MAX - BUZZER_FREQ_MIN);
            for (int i = 0; i < 10; i++)
            {
                double freq = start * (1.0 - (double)i / 10.0);
                if (freq < 1.0)
                    break;
                uint32_t wrap = (uint32_t)(125000000.0 / freq) - 1;
                pwm_set_wrap(slice_buzzer, wrap);
                pwm_set_chan_level(slice_buzzer, pwm_gpio_to_channel(BUZZER_PIN), wrap / 2);
                pwm_set_enabled(slice_buzzer, true);
                sleep_ms(50);
            }
            pwm_set_enabled(slice_buzzer, false);
            feedback_pending = false;
        }

        sleep_ms(25);
    }
    return 0;
}
