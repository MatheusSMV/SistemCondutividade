#include "hardware/i2c.h" //Biblioteca para controlar o barramento I2C no Raspberry Pi Pico.
#include "inc/ssd1306.h" //Biblioteca para controle do display OLED SSD1306, incluindo renderização de texto.
#include "inc/font.h" //Biblioteca onde está o nosso alfabeto.
#include "inc/ws2812.pio.h" //Para controlar a minha matriz de leds.
#include <stdio.h> //Biblioteca padrão C.
#include "pico/stdlib.h" //Biblioteca das funções básicas do Raspberry Pi Pico, como inicialização e controle de GPIOs.
#include "hardware/pio.h" //Biblioteca das funções para manipulação do PIO (Programmable I/O) para controlar a matriz de LEDs WS2812.
#include "hardware/clocks.h" //Biblioteca das funções para trabalhar com temporizadores.
#include "hardware/timer.h" //Biblioteca das funções para trabalhar com controle de relógios.
#include "hardware/adc.h" //Biblioteca para o conversor analógico digital.       
#include "anim.h"
#include "math.h"
#include <stdlib.h>
#include <string.h>

//Definição de pinos
#define I2C_PORT i2c1 //Definição das portas I2C.
#define I2C_SDA 14 //Definição das portas I2C.
#define I2C_SCL 15 //Definição das portas I2C.
#define endereco 0x3C //Endereço do display no barramento I2C.

#define Led_G 11 //Definindo o pino 11 do led verde.
#define Led_B 12 //Definindo o pino 12 do led azul.
#define Led_R 13 //Defidindo o pino 13 do led vermelho.

#define botao_a 5 //Definindo o botao a no pino 5.
#define botao_b 6 //Definindo o botao b no pino 6.

#define Buzzer_A 21 //Definindo o Buzzer no pino 21.
#define Buzzer_B 10 //Definindo o Buzzer no pino 10.

#define VRX_PIN 27 //Definindo o Pino do Potênciometro X.
#define VRY_PIN 26 //Definindo o Pino do Potênciometro Y.
#define Debounce_b 500 //Debounce para o botão 22.
#define Debounce_a 500 //Debounce para o botão 22.

#define IS_RGBW false //Apenas o RGB sem LED Branco.
#define WS2812_PIN 7 //A Matriz de LED GPIO 7.
#define NUM_PIXELS 25 // A matriz de Led 5x5

//Variáveis globais
uint8_t led_r = 50; //Potência do nosso led vermelho da matriz de leds.
uint8_t led_g = 0; //Verde desligado.
uint8_t led_b = 0; //Azul desligado

// Variáveis de decisão
bool experimento = true;
bool press_b = false;
bool press_a = false;
bool inicio = false;
bool led_buffer[NUM_PIXELS]; //Armazinar o Número de Pixels

float Ti_f = 0; 
float Te_f = 0; 
float deltaTi_f = 0; 
float deltaTe_f = 0;

// Função de interrupção para os botões
void botao_irq_handler(uint gpio, uint32_t events) {
    static uint32_t ultima_interrup_b = 0; //Capturar o tempo da interrupção.
    static uint32_t ultima_interrup_a = 0; //Capturar o tempo da interrupção.
    uint32_t tempo_atual = to_ms_since_boot(get_absolute_time()); // Tempo atual em ms desde o início do sistema

    //Verifica se o botão B foi pressionado.
    if (gpio == botao_b && tempo_atual - ultima_interrup_b > Debounce_b) {
        ultima_interrup_b = tempo_atual; //Atualiza o tempo da última interrupção.
        experimento = false;
        press_b = true;
        inicio = false;
        printf("Experimento Finalizado. \n");
    } else {
        press_b = false;
    }

    // Verifica se o botão A foi pressionado
    if (gpio == botao_a && tempo_atual - ultima_interrup_a > Debounce_a) {
        ultima_interrup_a = tempo_atual; //Atualiza o tempo da última interrupção.
        experimento = true;
        press_a = true;
        inicio = true;
        printf("Experimento Iniciado. \n");
    } else {
        press_a = false;
    }
}

//Função que envia um valor de cor para o pino de controle da matriz de LEDs WS2812.
static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio0, 0, pixel_grb << 8u);
}

//Função que converte valores RGB em um valor de 32 bits no formato GRB.
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)(r) << 8) | ((uint32_t)(g) << 16) | (uint32_t)(b);
}

//Função para configurar as cores dos leds conforme a leitura e o setado nas variáveis globais.
void set_one_led(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t color = urgb_u32(r, g, b);
    for (int i = 0; i < NUM_PIXELS; i++) {
        put_pixel(led_buffer[i] ? color : 0);
    }
}

void exibir_tabela() {
    // Cabeçalho da tabela de dados
    printf("+-------------------------------------------------------------+\n");
    printf("|                     Tabela de Dados                         |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| %-46s | %-13s|\n", "Descrição", "Valor");
    printf("+-------------------------------------------------------------+\n");

    // Valores fixos
    float ri_al = 0.00485, re_al = 0.0110, ri_m = 0.0110, re_m = 0.0182;
    float K_al = 237.00, l_c = 0.0635, uf = 4.39, r_ele = 15.4;

    // Exibição dos dados
    printf("| %-46s  %13.6f \n", "Raio Interno do Alumínio (m)", ri_al);
    printf("| %-46s  %13.6f \n", "Raio Externo do Alumínio (m)", re_al);
    printf("| %-46s  %13.6f \n", "Raio Interno da Manta de Isolante (m)", ri_m);
    printf("| %-46s  %13.6f \n", "Raio Externo da Manta de Isolante (m)", re_m);
    printf("| %-46s  %13.6f \n", "Condutividade Térmica do Alumínio (W/mK)", K_al);
    printf("| %-46s  %13.6f \n", "Comprimento do Conjunto (m)", l_c);
    printf("| %-46s  %13.6f \n", "Tensão da Fonte de Alimentação (V)", uf);
    printf("| %-46s  %13.6f \n", "Resistência Elétrica (Ohms)", r_ele);
    printf("+-------------------------------------------------------------+\n");

    // Cabeçalho da tabela de resultados
    printf("\n+-------------------------------------------------------------+\n");
    printf("|                   Tabela de Resultados                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| %-46s | %-13s|\n", "Descrição", "Valor");
    printf("+-------------------------------------------------------------+\n");

    // Cálculos dos resultados
    float Q = (uf * uf) / r_ele;
    float R_a = (log(re_al / ri_al)) / (2 * 3.14 * l_c * K_al);
    float R_i = ((Ti_f - Te_f) / Q) - R_a;
    float K_i = (log(re_m / ri_m)) / (2 * 3.14 * l_c * R_i);

    printf("| %-46s  %13.6f \n", "Potência Dissipada (W)", Q);
    printf("| %-46s  %13.6f \n", "Resistência Condutiva do Alumínio (W/K)", R_a);
    printf("| %-46s  %13.6f \n", "Resistência Condutiva do Isolante (W/K)", R_i);
    printf("| %-46s  %13.6f \n", "Condutividade Térmica do Isolante (W/mK)", K_i);
    printf("+-------------------------------------------------------------+\n");
}

int cor_atual = 0;

int main() {
    stdio_init_all(); //Inicializando serial.

    PIO pio = pio0; //Definindo o PIO 0 para controlar a matriz de leds.
    int sm = 0; //Maquina de estado = 0.
    uint offset = pio_add_program(pio, &ws2812_program); //Programa para controlar os leds da matriz.
    ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW); //Inicializa o PIO com o programa WS2812.

    adc_init(); //Iniciando o ADC.

    adc_gpio_init(VRX_PIN); //Iniciando o PINO VRX.
    adc_gpio_init(VRY_PIN); //Iniciando o PINO VRY.

    //I2C Inicialização.
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    gpio_init(Led_G);
    gpio_set_dir(Led_G, GPIO_OUT); //Inicializando o LED verde.
    gpio_put(Led_G, false);

    gpio_init(Led_B);
    gpio_set_dir(Led_B, GPIO_OUT); //Inicializando o LED Azul.
    gpio_put(Led_B, false);

    gpio_init(Led_R);
    gpio_set_dir(Led_R, GPIO_OUT); //Inicializando o LED Vermelho.
    gpio_put(Led_R, false);

    gpio_init(Buzzer_A);
    gpio_set_dir(Buzzer_A, GPIO_OUT); //Inicializando o Buzzer A.
    gpio_put(Buzzer_A, false);

    gpio_init(Buzzer_B);
    gpio_set_dir(Buzzer_B, GPIO_OUT); //Inicializando o Buzzer B.
    gpio_put(Buzzer_B, false);

    gpio_init(botao_b);
    gpio_set_dir(botao_b, GPIO_IN); //Inicializando o Botão do JOY e habilitando Pull-Up
    gpio_pull_up(botao_b);
    gpio_set_irq_enabled_with_callback(botao_b, GPIO_IRQ_EDGE_FALL, true, &botao_irq_handler); //Chamar a função de interrupção na borda de descida.

    gpio_init(botao_a);
    gpio_set_dir(botao_a, GPIO_IN); //Inicializando o Botão do JOY e habilitando Pull-Up
    gpio_pull_up(botao_a);
    gpio_set_irq_enabled_with_callback(botao_a, GPIO_IRQ_EDGE_FALL, true, &botao_irq_handler); //Chamar a função de interrupção na borda de descida.

    bool cor = true; //Cor nesse caso verdadeiro é branco
    uint32_t tempo_passado = 0;
    float Ti_a = 25;
    float Te_a = 25;
    int contador = 0; 
    int contador_f = 0;

    while (true) {
        if (experimento == true && inicio == true) {
            animacao_inicio(); // Exibe a animação de início
            set_one_led(0, 10, 0); // Atualiza a cor do LED
            adc_select_input(1); //Canal do VRX
            uint16_t VRX_Valor = adc_read(); //Lendo o valor do VRX.
            adc_select_input(0); //Canal do VRY  
            uint16_t VRY_Valor = adc_read(); //Lendo o valor do VRY.

            float Ti = 25 + ((VRX_Valor - 2048) * 75) / 2048; // Temperatura Interna ---> VRX (No meio temperatura ambiente)
            float Te = 25 + ((VRY_Valor - 2048) * 75) / 2048;  //Temperatura Externa --> vRY (No meio temperatura ambiente)

            float deltaTi = Ti - Ti_a;
            float deltaTe = Te - Te_a;

            //Para print
            Ti_f = Ti; 
            Te_f = Te; 
            deltaTi_f = deltaTi; 
            deltaTe_f = deltaTe; 
            contador_f = contador;

            uint64_t tempo_atual = to_ms_since_boot(get_absolute_time()); //Pegando o tempo atual

            if (tempo_atual - tempo_passado >= 1000) {
                printf("Ti = %.2f °C | Te = %.2f °C | DTi = %.2f °C | DTe = %.2f °C | Tempo: %d s\n", Ti, Te, deltaTi, deltaTe, contador);
                contador++;
                tempo_passado = tempo_atual; //Atualizando o tempo passado.

                Ti_a = Ti; //Atualizando a temperatura interna anterior
                Te_a = Te; //Atualizando a temperatura externa anterior
            } else {
                0;
            }

            if (Ti < 20) {
                gpio_put(Led_G, false);
                gpio_put(Led_B, true);
                gpio_put(Led_R, false);
            } else if (Ti >= 20 && Ti < 50) {
                gpio_put(Led_G, true);
                gpio_put(Led_B, false);
                gpio_put(Led_R, false);
            } else if (Ti >= 50 && Ti < 75) {
                gpio_put(Led_G, true);
                gpio_put(Led_B, false);
                gpio_put(Led_R, true);
            } else if (Ti >= 75) {
                gpio_put(Led_G, false);
                gpio_put(Led_B, false);
                gpio_put(Led_R, true);
            }

            ssd1306_fill(&ssd, !cor); // Limpar Display

            int eixo_x = 5 + (VRX_Valor * (128 - 3)) / 4096;  // Vou converter o valor do ADC para o pixel desejado, com o valor minimo de 5 para não ultrapassar a borda.
            int eixo_y = 5 + (64 - 5 - ((VRY_Valor * (64 - 5)) / 4096)); // Vou converter o valor do ADC para o pixel desejado, com o valor minimo de 5 para não ultrapassar a borda.
            // Limitar para garantir que o valor de eixo_x não ultrapasse 115 e de eixo_y não ultrapasse 50. Respeitar as bordas.
            if (1900 < VRX_Valor && VRX_Valor < 2000 && 2000 < VRY_Valor && VRY_Valor < 2100) {  //Condição para eu simular o ponto central em  X e Y.
                ssd1306_rect(&ssd, 28, 60, 8, 8, cor, !cor); // Desenha um retângulo
            } else {
                eixo_x = (eixo_x > 115) ? 115 : eixo_x; //Limitando a 115.
                eixo_y = (eixo_y > 50) ? 50 : eixo_y; //Limitando a 50.     //Para não sair da bora.
                ssd1306_rect(&ssd, eixo_y, eixo_x, 8, 8, cor, !cor); // Desenha um retângulo
            }

            ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo
            ssd1306_rect(&ssd, 3, 3, 61, 58, cor, !cor); // Desenha um retângulo
            ssd1306_rect(&ssd, 3, 3, 122, 29, cor, !cor); // Desenha um retângulo

            ssd1306_send_data(&ssd);

        } else if (experimento == false && press_b == true) {
            contador = 0;  
            animacao_fim();
            set_one_led(10, 0, 0);

            for (int i = 0; i < 20; i++) { // 5 bips
                gpio_put(Buzzer_A, true);
                gpio_put(Buzzer_B, true);
                sleep_us(500); // 500 µs = frequência de 1000 Hz (mais audível)
                gpio_put(Buzzer_A, false);
                gpio_put(Buzzer_B, false);
                sleep_us(500); // Pausa entre os bips
            }

            // Imprime tabela no terminal
            printf("\n===== TABELA DE EXPERIMENTAL =====\n");
            printf("| %-20s | %-20s |\n", "Grandeza", "Valor");
            printf("|----------------------|----------------------|\n");
            printf("| Temperatura Interna Final (Ti) | %.2f °C\n", Ti_f);
            printf("| Temperatura Externa Final (Te) | %.2f °C\n", Te_f);
            printf("| Variação Ti (ΔTi)              | %.2f °C\n", deltaTi_f);  // Supondo 25 como inicial
            printf("| Variação Te (ΔTe)              | %.2f °C\n", deltaTe_f);
            printf("| Tempo total do experimento     | %d segundos\n", contador_f);
            printf("==============================================\n");

            exibir_tabela();
            press_b = false; // Reseta o estado para não imprimir de novo
        }

        if (!experimento && !press_b) {
            sleep_ms(100); // Pequena pausa para evitar loop infinito
            continue;      // Aguarda até que sistema seja reativado pelo botão A
        }
    }
}