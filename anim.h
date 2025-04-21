#ifndef ANIM_H
#define ANIM_H

#define NUM_PIXELS 25 // A matriz de Led 5x5

bool led_buffer[NUM_PIXELS]; //Armazinar o Número de Pixels

void limpar_matriz() {
  for (int i = 0; i < NUM_PIXELS; i++) {
      led_buffer[i] = false;
  }
}



void animacao_inicio() {
  limpar_matriz();
  led_buffer[3] = true;
  led_buffer[5] = true;
  led_buffer[7] = true;
  led_buffer[11] = true;
  led_buffer[19] = true;
 
}


void animacao_fim() {
  limpar_matriz();
  led_buffer[0] = true;
  led_buffer[1] = true;
  led_buffer[2] = true;
  led_buffer[3] = true;
  led_buffer[4] = true;
  led_buffer[5] = true;
  led_buffer[6] = true;
  led_buffer[9] = true;
  led_buffer[10] = true;
  led_buffer[12] = true;
  led_buffer[14] = true;
  led_buffer[15] = true;
  led_buffer[18] = true;
  led_buffer[19] = true;
  led_buffer[20] = true;
  led_buffer[21] = true;
  led_buffer[22] = true;
  led_buffer[23] = true;
  led_buffer[24] = true;
}


#endif