#include "hardware/i2c.h" //Biblioteca para controlar o barramento I2C no Raspberry Pi Pico.
#include "inc/ssd1306.h" //Biblioteca para controle do display OLED SSD1306, incluindo renderização de texto.
#include "inc/font.h" //Biblioteca onde está o nosso alfabeto.
#include <stdio.h> //Biblioteca padrão C.
#include "pico/stdlib.h" //Biblioteca das funções básicas do Raspberry Pi Pico, como inicialização e controle de GPIOs.
#include "hardware/pio.h" //Biblioteca das funções para manipulação do PIO (Programmable I/O) para controlar a matriz de LEDs WS2812.
#include "hardware/clocks.h" //Biblioteca das funções para trabalhar com temporizadores.
#include "hardware/timer.h" //Biblioteca das funções para trabalhar com controle de relógios.
#include "hardware/adc.h" //Biblioteca para o conversor analógico digital.     
#include "hardware/pwm.h" //Biblioteca para o PWM.   
//------------------------------------------------------------------------
//Definição de pinos
#define I2C_PORT i2c1 //Definição das portas I2C.
#define I2C_SDA 14 //Definição das portas I2C.
#define I2C_SCL 15 //Definição das portas I2C.
#define endereco 0x3C //Endereço do display no barramento I2C.
//--------------------------------------------------------------------------
#define Led_G 11 //Definindo o pino 11 do led verde.
#define Led_B 12 //Definindo o pino 12 do led azul.
#define Led_R 13 //Defidindo o pino 13 do led vermelho.
volatile bool estado_led = false; //Definir o estado do Led verde como desligado, variável que varia o longo do cod.
volatile bool pwm_led = true; //Definir o estado do Led verde como desligado, variável que varia o longo do cod.
//--------------------------------------------------------------------------
#define botao_a 5 //Definindo o botao a no pino 5.
//--------------------------------------------------------------------------
#define VRX_PIN 27 //Definindo o Pino do Potênciometro X.
#define VRY_PIN 26 //Definindo o Pino do Potênciometro Y.
#define Botao_JOY 22 //Definindo o pino do botão do JOY.
#define Debounce 500//Debounce para o botão 22.
#define Debounce_a 400//Debounce para o botão 22.
//--------------------------------------------------------------------------
#define L_monitor 128 //Largura do monitor.
#define H_monitor 64 //Altura do monitor.
#define T_quadrado 8 //Tamanho do quadrado.
//--------------------------------------------------------------------------
int borda = 0; //Variável  para controlar a borda.
//--------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------
void pwm_cfg(){

    //----------------------------------------------------------------------------------------
    //Led Azul
    gpio_set_function(Led_B , GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM
    uint slice_B = pwm_gpio_to_slice_num(Led_B); //obter o canal PWM da GPIO
    pwm_set_wrap(slice_B, 4096); //definir o valor de wrap
    pwm_set_gpio_level(Led_B , VRY_PIN); //definir o ciclo de trabalho (duty cycle) do pwm
    pwm_set_enabled(slice_B, true); //habilita o pwm no slice correspondente

    //----------------------------------------------------------------------------------------
    //Led Vermelho
    gpio_set_function(Led_R , GPIO_FUNC_PWM); //habilitar o pino GPIO como PWM
    uint slice_R = pwm_gpio_to_slice_num(Led_R); //obter o canal PWM da GPIO
    pwm_set_wrap(slice_R, 4096); //definir o valor de wrap
    pwm_set_gpio_level(Led_R , VRX_PIN); //definir o ciclo de trabalho (duty cycle) do pwm
    pwm_set_enabled(slice_R, true); //habilita o pwm no slice correspondente    

}
//---------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------
// Função de interrupção para os botões
void botao_irq_handler(uint gpio, uint32_t events) {
    static uint32_t ultima_interrup_joy = 0; //Capturar o tempo da interrupção.
    static uint32_t ultima_interrup_a = 0; //Capturar o tempo da interrupção.
    uint32_t tempo_atual = to_ms_since_boot(get_absolute_time()); // Tempo atual em ms desde o início do sistema

    //Verifica se o botão JOY foi pressionado.
    if (gpio == Botao_JOY && tempo_atual - ultima_interrup_joy > Debounce) {
        estado_led = !estado_led; //Alterna o estado do LED Verde.
        gpio_put(Led_G, estado_led); //Atualiza o estado do LED.
        ultima_interrup_joy = tempo_atual; //Atualiza o tempo da última interrupção.

        borda = (borda + 1) % 3; //Alterna entre os valores de borda (0, 1, 2). - Vai e volta.
    }

    // Verifica se o botão A foi pressionado
    if (gpio == botao_a && tempo_atual - ultima_interrup_a > Debounce_a) {
        pwm_led = !pwm_led;  //Altero o pwm_led ligando e desligando em função de A.
        ultima_interrup_a = tempo_atual; //Atualiza o tempo da última interrupção.
    }
}
//---------------------------------------------------------------------------------------------

int main() {

  stdio_init_all(); //Inicializando serial.

  adc_init(); //Iniciando o ADC.

//---------------------------------------------------------------------------------------------
  adc_gpio_init(VRX_PIN); //Iniciando o PINO VRX.
  adc_gpio_init(VRY_PIN); //Iniciando o PINO VRY.
//---------------------------------------------------------------------------------------------  

//-----------------------------------------------------------------------------------------------
  //I2C Inicialização.
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);                                //Inicialização do I²C com o Display OLED.
  gpio_pull_up(I2C_SCL);
  ssd1306_t ssd; 
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT);
  ssd1306_config(&ssd);
  ssd1306_send_data(&ssd);
//-----------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------
pwm_cfg(); //Inicianlizando as configurações do PWW, para os leds Azul e Vermelho.
uint slice_B = pwm_gpio_to_slice_num(Led_B); //Para ajuste do PWM em função da variação do ADC.
uint slice_R = pwm_gpio_to_slice_num(Led_R); //Para ajuste do PWM em função da variação do ADC.
//----------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------
gpio_init(Led_G);
gpio_set_dir(Led_G, GPIO_OUT); //Inicializando o LED verde.
gpio_put(Led_G, false);
//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
gpio_init(Botao_JOY);
gpio_set_dir(Botao_JOY, GPIO_IN); //Inicializando o Botão do JOY e habilitando Pull-Up
gpio_pull_up(Botao_JOY);
gpio_set_irq_enabled_with_callback(Botao_JOY, GPIO_IRQ_EDGE_FALL, true, &botao_irq_handler); //Chamar a função de interrupção na borda de descida.
//-----------------------------------------------------------------------------------------------

gpio_init(botao_a);
gpio_set_dir(botao_a, GPIO_IN); //Inicializando o Botão do JOY e habilitando Pull-Up          //** Solução colocar em uma interrupção.
gpio_pull_up(botao_a);
gpio_set_irq_enabled_with_callback(botao_a, GPIO_IRQ_EDGE_FALL, true, &botao_irq_handler); //Chamar a função de interrupção na borda de descida.
//-----------------------------------------------------------------------------------------------
bool cor = true; //Cor nesse caso verdadeiro é branco


  while (true) {
    //-------------------------------------------------------------------------------------------
    adc_select_input(1); //Canal do VRX
    uint16_t VRX_Valor = adc_read(); //Lendo o valor do VRX.
    adc_select_input(0); //Canal do VRY  
    uint16_t VRY_Valor = adc_read(); //Lendo o valor do VRY.
    //--------------------------------------------------------------------------------------------

    //-------------------------------------------------------------------------------------------------------------
    if(pwm_led){
    int16_t pwm_VRX = (VRX_Valor > 1967) ? (VRX_Valor - 1967) * 2 : (1967- VRX_Valor) * 2; //Condição para os LEDS
    int16_t pwm_VRY = (VRY_Valor > 2030) ? (VRY_Valor - 2030) * 2 : (2030 - VRY_Valor) * 2;//Condição para os LEDS

    if(1900<VRX_Valor && VRX_Valor<2000 ){
        pwm_set_gpio_level(Led_R, 0);              //Condição para eu simular o ponto central em X.
    }else { pwm_set_gpio_level(Led_R, pwm_VRX);}

    if(2000<VRY_Valor && VRY_Valor<2100){
        pwm_set_gpio_level(Led_B, 0);              //Condição para eu simular o ponto central em Y.
    }else { pwm_set_gpio_level(Led_B, pwm_VRY);}
    //-------------------------------------------------------------------------------------------------------------
    
    //Setando o valor para a intensidade do LED utilizando o ADC VRY.

    }else {

        pwm_set_gpio_level(Led_B , 0); //Level do Led Azul 0.
        pwm_set_gpio_level(Led_R , 0); //Level do Led Vermelho 0.

    }
    //------------------------------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------------------------
    if(borda==0){
        ssd1306_fill(&ssd, !cor); // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha a borda 0   
    }else if(borda==1){
        ssd1306_fill(&ssd, !cor); // Limpa o display
        ssd1306_rect(&ssd, 2, 2, 124, 60, cor, !cor); // Desenha a borda 1
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha a borda 0 
    }else if(borda==2){
        ssd1306_fill(&ssd, !cor); // Limpa o display
        ssd1306_rect(&ssd, 1, 1, 126, 62, cor, !cor); // Desenha a borda 2
        ssd1306_rect(&ssd, 2, 2, 124, 60, cor, !cor); // Desenha a borda 1
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha a borda 0 
    }else 0;
    //---------------------------------------------------------------------------------------------------------------
   
    //-----------------------------------------------------------------------------------------------------------------
    int eixo_x = 5 + (VRX_Valor * (128 - 3)) / 4096;  // Vou converter o valor do ADC para o pixel desejado, com o valor minimo de 5 para não ultrapassar a borda.
    int eixo_y = 5 + (64 - 5 - ((VRY_Valor * (64-5)) / 4096)); // Vou converter o valor do ADC para o pixel desejado, com o valor minimo de 5 para não ultrapassar a borda.
    // Limitar para garantir que o valor de eixo_x não ultrapasse 115 e de eixo_y não ultrapasse 50. Respeitar as bordas.
    if(1900<VRX_Valor && VRX_Valor<2000 && 2000<VRY_Valor && VRY_Valor<2100 ){  //Condição para eu simular o ponto central em  X e Y.
        ssd1306_rect(&ssd, 28, 60, 8, 8, cor, !cor); // Desenha um retângulo              
    }else {     
        eixo_x = (eixo_x > 115) ? 115 : eixo_x; //Limitando a 115.
        eixo_y = (eixo_y > 50) ? 50 : eixo_y; //Limitando a 50.     //Para não sair da bora.
        ssd1306_rect(&ssd, eixo_y, eixo_x, 8, 8, cor, !cor); // Desenha um retângulo 
    }
    //------------------------------------------------------------------------------------------------------------------

    ssd1306_send_data(&ssd);
  }
}