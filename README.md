# Trabalho 01: Sensor de proximidade com a BitDogLab

---

## Aluno: 
- **Carlos Henrique Silva Lopes**

## **Link do vídeo de Ensaio**
https://drive.google.com/file/d/1qTuIB8wwaVzRuXHxBdSdAlvGBP0ivmfI/view?usp=sharing

## Objetivos
O projeto simula um sensor de proximidade usando a placa BitDogLab e diversos periféricos: joystick, botões, buzzer, display OLED SSD1306, matriz de LEDs WS2812 e LED RGB. Movimentando o joystick, o usuário define uma distância simulada, que é então registrada ao retornar ao centro e pressionar o botão do joystick. O sistema gera feedback visual (OLED, LEDs e LED RGB), sonoro (buzzer) e envia o valor medido via UART0. Um segundo botão permite resetar o valor máximo registrado.

### Principais Arquivos
- **`Trabalho01.c:`** Contém a lógica principal para fazer a leitura do joystick, dos botões e acionamento dos feedbacks visuais e sonoros.
- **`lib/:`** Contém os arquivos com caracteres em hexadecimal, arquivos para escrever no ssd1306, e os desenhos que aparecerão na matriz de leds.
- **`README.md:`** Documentação detalhada do projeto.

