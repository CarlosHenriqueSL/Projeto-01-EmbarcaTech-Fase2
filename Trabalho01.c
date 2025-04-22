/*
 * Lê o joystick (eixo X), exibe num OLED SSD1306, controla LEDs WS2812,
 * gera feedback sonoro e luminoso, e envia dados pela UART0.
 * Ao retornar ao centro e pressionar o botão do joystick, registra o valor máximo do ADC
 * joystick alcançado desde o último reset (via botão B).
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/numeros.h"
#include "blink.pio.h"

// ==== PINAGEM E CONFIGURAÇÕES GERAIS ====

// WS2812 na GPIO7 (PIO)
#define WS2812_PIN 7

// I2C1 para OLED SSD1306 (SDA=GPIO14, SCL=GPIO15)
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
#define DEBOUNCE_MS 400

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
static volatile double valor_atual_adc = 0.0; // leitura instantânea (0.0–1.0)
static volatile double valor_max_adc = 0.0;   // máximo desde último reset/B
static uint8_t porcentagem_atual = 0;

// debounce de cada botão
static uint32_t last_irq_time_joy = 0;
static uint32_t last_irq_time_a = 0;
static uint32_t last_irq_time_b = 0;

// Inicializa UART0 para debug
void init_uart()
{
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
}

// Inicializa I2C, ADC, PWM e demais periféricos
void init_peripherals()
{
    // --- I2C para OLED SSD1306 ---
    i2c_init(I2C_PORT, 400000);           
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);               // Pull-up interno
    gpio_pull_up(I2C_SCL);

    // --- ADC para joystick ---
    adc_init();                          // Liga periférico ADC
    adc_gpio_init(JOY_GPIO_ADX);         // Associa ADC1 ao GPIO27

    // --- PWM para LED e buzzer ---
    gpio_set_function(LED_PIN_RED, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    uint slice_led    = pwm_gpio_to_slice_num(LED_PIN_RED);
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);
    pwm_set_wrap(slice_led, PWM_WRAP);          
    pwm_set_wrap(slice_buzzer, PWM_WRAP);
    pwm_set_enabled(slice_led, true);           // Ativa PWM do LED
    pwm_set_enabled(slice_buzzer, false);       // Mantém buzzer desligado
}

// Desliga display, LEDs e buzzer
void desligar(ssd1306_t *ssd, PIO pio, uint sm,
              bool *prev_on, uint slice_led, uint slice_buzzer)
{
    pwm_set_gpio_level(LED_PIN_RED, 0);          // Apaga LED vermelho
    pwm_set_enabled(slice_buzzer, false);        // Desliga buzzer

    // Limpa tela OLED
    ssd1306_fill(ssd, false);
    ssd1306_send_data(ssd);

    // Zera todos os pixels WS2812
    for (int i = 0; i < NUM_PIXELS; i++) {
        pio_sm_put_blocking(pio, sm, 0);
    }

    *prev_on = false;                            // Próxima vez, reativa tela
    sleep_ms(100);
}

// Envia padrão para LEDs WS2812
uint32_t matrix_rgb(double r, double g, double b)
{
    unsigned char R = r * 255;
    unsigned char G = g * 255;
    unsigned char B = b * 255;
    return (G << 24) | (R << 16) | (B << 8);
}

// Envia padrão para LEDs WS2812
void desenho_pio(double *desenho, PIO pio, uint sm)
{
    for (int i = 0; i < NUM_PIXELS; i++)
    {
        pio_sm_put_blocking(pio, sm,
                            matrix_rgb(desenho[24 - i], 0, 0));
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

    // --- Botão do joystick pressionado ---
    if (gpio == JOY_BUTTON_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        if (now - last_irq_time_joy < DEBOUNCE_MS) return;
        last_irq_time_joy = now;

        // Converte valor máximo em porcentagem
        double total = valor_max_adc;
        printf("Valor máximo lido: %.3f\n", total);
        if      (total <= 0.1) porcentagem_atual = 0;
        else if (total <= 0.2) porcentagem_atual = 20;
        else if (total <= 0.4) porcentagem_atual = 40;
        else if (total <= 0.6) porcentagem_atual = 60;
        else if (total <= 0.8) porcentagem_atual = 80;
        else                   porcentagem_atual = 100;

        feedback_pending = true;  // solicita feedback 
    }

    // --- Botão A: sistema on/off ---
    else if (gpio == BUTTON_A && (events & GPIO_IRQ_EDGE_FALL)) {
        if (now - last_irq_time_a < DEBOUNCE_MS) return;
        last_irq_time_a = now;
        sistema_ligado = !sistema_ligado;
    }

    // --- Botão B faz reset de valores máximos ---
    else if (gpio == BUTTON_B && (events & GPIO_IRQ_EDGE_FALL)) {
        if (now - last_irq_time_b < DEBOUNCE_MS) return;
        last_irq_time_b = now;

        porcentagem_atual = 0;
        valor_max_adc    = 0.0;
        feedback_pending = false;
        pwm_set_gpio_level(LED_PIN_RED, 0);  // apaga LED
    }
}

void init_gpio()
{
    // Configura LED e buzzer como saída digital
    gpio_init(LED_PIN_RED);
    gpio_set_dir(LED_PIN_RED, GPIO_OUT);
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Botões com pull-up interno
    gpio_init(BUTTON_A);
    gpio_set_dir(BUTTON_A, GPIO_IN);
    gpio_pull_up(BUTTON_A);

    gpio_init(BUTTON_B);
    gpio_set_dir(BUTTON_B, GPIO_IN);
    gpio_pull_up(BUTTON_B);

    gpio_init(JOY_BUTTON_PIN);
    gpio_set_dir(JOY_BUTTON_PIN, GPIO_IN);
    gpio_pull_up(JOY_BUTTON_PIN);

    // Habilita interrupções
    gpio_set_irq_enabled_with_callback(JOY_BUTTON_PIN,
                                       GPIO_IRQ_EDGE_FALL, true,
                                       &gpio_irq_handler);
    gpio_set_irq_enabled(BUTTON_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BUTTON_B, GPIO_IRQ_EDGE_FALL, true);
}

int main(void)
{
    stdio_init_all();   // Inicializa STDIO para printf

    init_uart();        // UART para debug
    init_gpio();        // GPIOs e interrupções
    init_peripherals();// I2C, ADC e PWM

    // Inicializa display OLED
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Configuração PIO para LEDs WS2812
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &blink_program);
    uint sm     = pio_claim_unused_sm(pio, true);
    blink_program_init(pio, sm, offset, WS2812_PIN);

    // Slices PWM para LED e buzzer
    uint slice_led    = pwm_gpio_to_slice_num(LED_PIN_RED);
    uint slice_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);

    bool prev_on = false; // Indica se sistema já exibiu tela de "iniciado"

    while (true)
    {
        // Se sistema desligado, apaga tudo e pula
        if (!sistema_ligado)
        {
            desligar(&ssd, pio, sm, &prev_on, slice_led, slice_buzzer);
            sleep_ms(100);
            continue;
        }

        // Na primeira vez após ligar, mostra a mensagem de início
        if (!prev_on)
        {
            pwm_set_gpio_level(LED_PIN_RED,
                               (uint32_t)(valor_max_adc * PWM_WRAP));
            ssd1306_fill(&ssd, false);
            ssd1306_draw_string(&ssd, "Sistema iniciado", 4, 25);
            ssd1306_send_data(&ssd);
            prev_on = true;
            sleep_ms(500);
        }

        // === Loop de leitura do joystick e atualização de display ===
        ssd1306_fill(&ssd, false);
        ssd1306_rect(&ssd, 0, 0, ssd.width, ssd.height, true, false);

        // Lê ADC do eixo X
        adc_select_input(JOY_X_ADC);
        uint16_t adc_x = adc_read();

        // Mapeia o valor do ADC para a posição do retângulo no display 
        uint8_t sq_x = (adc_x * (ssd.width - 8)) / 4095;
        ssd1306_rect(&ssd, 31, sq_x, 8, 8, true, true);
        ssd1306_send_data(&ssd);

        // Calcula deslocamento absoluto do centro e aplica deadzone
        int delta = abs((int)adc_x - JOY_CENTER);
        if (delta < JOY_DEADZONE) delta = 0;

        valor_atual_adc = (double)delta / JOY_CENTER;
        // Atualiza valor máximo se passou do anterior
        if (valor_atual_adc > valor_max_adc) {
            valor_max_adc = valor_atual_adc;
        }

        // Atualiza LEDs WS2812 de acordo com a porcentagem
        double *desenho = definir_desenho();
        desenho_pio(desenho, pio, sm);

        // Se o feedback for solicitado (botão do joystick), emite luz e som
        if (feedback_pending)
        {
            if (porcentagem_atual == 0) {
                pwm_set_gpio_level(LED_PIN_RED, 0);
            } else {
                // Ajusta LED vermelho ao valor máximo registrado
                pwm_set_gpio_level(LED_PIN_RED,
                    (uint32_t)(valor_max_adc * PWM_WRAP));

                // Toca buzzer em decaimento de frequência
                double start = BUZZER_FREQ_MIN +
                        valor_max_adc * (BUZZER_FREQ_MAX - BUZZER_FREQ_MIN);
                for (int i = 0; i < 10; i++)
                {
                    double freq = start * (1.0 - (double)i / 10.0);
                    if (freq < 1.0) break;
                    uint32_t wrap = (uint32_t)(125000000.0 / freq) - 1;
                    pwm_set_wrap(slice_buzzer, wrap);
                    pwm_set_chan_level(slice_buzzer,
                        pwm_gpio_to_channel(BUZZER_PIN), wrap / 2);
                    pwm_set_enabled(slice_buzzer, true);
                    sleep_ms(50);
                }
                pwm_set_enabled(slice_buzzer, false);
            }
            // Limpa estado de feedback após execução
            valor_max_adc   = 0.0;
            feedback_pending = false;
        }

        sleep_ms(25);
    }

    return 0;
}