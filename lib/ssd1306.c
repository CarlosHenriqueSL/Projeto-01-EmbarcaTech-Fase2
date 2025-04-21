#include "ssd1306.h"
#include "font.h"

void ssd1306_init(ssd1306_t *ssd, uint8_t width, uint8_t height, bool external_vcc, uint8_t address, i2c_inst_t *i2c) {
  ssd->width = width;
  ssd->height = height;
  ssd->pages = height / 8U;
  ssd->address = address;
  ssd->i2c_port = i2c;
  ssd->bufsize = ssd->pages * ssd->width + 1;
  ssd->ram_buffer = calloc(ssd->bufsize, sizeof(uint8_t));
  ssd->ram_buffer[0] = 0x40;
  ssd->port_buffer[0] = 0x80;
}

void ssd1306_config(ssd1306_t *ssd) {
  ssd1306_command(ssd, SET_DISP | 0x00);
  ssd1306_command(ssd, SET_MEM_ADDR);
  ssd1306_command(ssd, 0x01);
  ssd1306_command(ssd, SET_DISP_START_LINE | 0x00);
  ssd1306_command(ssd, SET_SEG_REMAP | 0x01);
  ssd1306_command(ssd, SET_MUX_RATIO);
  ssd1306_command(ssd, HEIGHT - 1);
  ssd1306_command(ssd, SET_COM_OUT_DIR | 0x08);
  ssd1306_command(ssd, SET_DISP_OFFSET);
  ssd1306_command(ssd, 0x00);
  ssd1306_command(ssd, SET_COM_PIN_CFG);
  ssd1306_command(ssd, 0x12);
  ssd1306_command(ssd, SET_DISP_CLK_DIV);
  ssd1306_command(ssd, 0x80);
  ssd1306_command(ssd, SET_PRECHARGE);
  ssd1306_command(ssd, 0xF1);
  ssd1306_command(ssd, SET_VCOM_DESEL);
  ssd1306_command(ssd, 0x30);
  ssd1306_command(ssd, SET_CONTRAST);
  ssd1306_command(ssd, 0xFF);
  ssd1306_command(ssd, SET_ENTIRE_ON);
  ssd1306_command(ssd, SET_NORM_INV);
  ssd1306_command(ssd, SET_CHARGE_PUMP);
  ssd1306_command(ssd, 0x14);
  ssd1306_command(ssd, SET_DISP | 0x01);
}

void ssd1306_command(ssd1306_t *ssd, uint8_t command) {
  ssd->port_buffer[1] = command;
  i2c_write_blocking(
    ssd->i2c_port,
    ssd->address,
    ssd->port_buffer,
    2,
    false
  );
}

void ssd1306_send_data(ssd1306_t *ssd) {
  ssd1306_command(ssd, SET_COL_ADDR);
  ssd1306_command(ssd, 0);
  ssd1306_command(ssd, ssd->width - 1);
  ssd1306_command(ssd, SET_PAGE_ADDR);
  ssd1306_command(ssd, 0);
  ssd1306_command(ssd, ssd->pages - 1);
  i2c_write_blocking(
    ssd->i2c_port,
    ssd->address,
    ssd->ram_buffer,
    ssd->bufsize,
    false
  );
}

void ssd1306_pixel(ssd1306_t *ssd, uint8_t x, uint8_t y, bool value) {
  uint16_t index = (y >> 3) + (x << 3) + 1;
  uint8_t pixel = (y & 0b111);
  if (value)
    ssd->ram_buffer[index] |= (1 << pixel);
  else
    ssd->ram_buffer[index] &= ~(1 << pixel);
}

void ssd1306_fill(ssd1306_t *ssd, bool value) {
    for (uint8_t y = 0; y < ssd->height; ++y) {
        for (uint8_t x = 0; x < ssd->width; ++x) {
            ssd1306_pixel(ssd, x, y, value);
        }
    }
}



void ssd1306_rect(ssd1306_t *ssd, uint8_t top, uint8_t left, uint8_t width, uint8_t height, bool value, bool fill) {
  for (uint8_t x = left; x < left + width; ++x) {
    ssd1306_pixel(ssd, x, top, value);
    ssd1306_pixel(ssd, x, top + height - 1, value);
  }
  for (uint8_t y = top; y < top + height; ++y) {
    ssd1306_pixel(ssd, left, y, value);
    ssd1306_pixel(ssd, left + width - 1, y, value);
  }

  if (fill) {
    for (uint8_t x = left + 1; x < left + width - 1; ++x) {
      for (uint8_t y = top + 1; y < top + height - 1; ++y) {
        ssd1306_pixel(ssd, x, y, value);
      }
    }
  }
}

void ssd1306_line(ssd1306_t *ssd, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool value) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);

    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;

    int err = dx - dy;

    while (true) {
        ssd1306_pixel(ssd, x0, y0, value); // Desenha o pixel atual

        if (x0 == x1 && y0 == y1) break; // Termina quando alcança o ponto final

        int e2 = err * 2;

        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }

        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}


void ssd1306_hline(ssd1306_t *ssd, uint8_t x0, uint8_t x1, uint8_t y, bool value) {
  for (uint8_t x = x0; x <= x1; ++x)
    ssd1306_pixel(ssd, x, y, value);
}

void ssd1306_vline(ssd1306_t *ssd, uint8_t x, uint8_t y0, uint8_t y1, bool value) {
  for (uint8_t y = y0; y <= y1; ++y)
    ssd1306_pixel(ssd, x, y, value);
}

// Função para desenhar um caractere
void ssd1306_draw_char(ssd1306_t *ssd, char c, uint8_t x, uint8_t y) {
  uint16_t index = 0;
  // useRotation indica que o caractere está armazenado em ordem coluna (maiusculas e dígitos)
  bool useRotation = false;
  
  if (c >= 'A' && c <= 'Z') {
    index = (c - 'A' + 11) * 8; // Maiúsculas
    useRotation = true;
  } else if (c >= 'a' && c <= 'z') {
    index = (37 + (c - 'a')) * 8; // Minúsculas
    useRotation = false;
  } else if (c >= '0' && c <= '9') {
    index = (c - '0' + 1) * 8; // Números
    useRotation = true;
  }
  
  if (!useRotation) {
    // Para minúsculas: cada byte representa uma linha do caractere
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t line = font[index + i];
      for (uint8_t j = 0; j < 8; ++j) {
        bool pixel = (line >> (7 - j)) & 0x01;
        ssd1306_pixel(ssd, x + j, y + i, pixel);
      }
    }
  } else {
    // Para maiúsculas e dígitos: os dados estão armazenados por coluna,
    // então iteramos as colunas e depois as linhas para "rotacionar" o desenho
     for (uint8_t i = 0; i < 8; ++i) {
     uint8_t line = font[index + i];
        for (uint8_t j = 0; j < 8; ++j) {
            ssd1306_pixel(ssd, x + i, y + j, line & (1 << j));
    }
  }
}
}

// Função para desenhar um caractere ampliado (centralizado no display)
// scale: fator de escala (ex.: 2 para 16x16, 3 para 24x24, etc.)
void ssd1306_draw_big_char(ssd1306_t *ssd, char c, uint8_t scale) {
  uint16_t index = 0;
  bool useRotation = false; // indica se os dados da fonte estão armazenados por coluna

  if (c >= 'A' && c <= 'Z') {
    index = (c - 'A' + 11) * 8; // Maiúsculas
    useRotation = true;
  } else if (c >= 'a' && c <= 'z') {
    index = (37 + (c - 'a')) * 8; // Minúsculas
    useRotation = false;
  } else if (c >= '0' && c <= '9') {
    index = (c - '0' + 1) * 8; // Dígitos
    useRotation = true;
  } else {
    return; // caractere não suportado
  }

  // Tamanho do caractere ampliado (assumindo fonte 8x8)
  uint8_t charSize = 8 * scale;

  // Calcula deslocamentos para centralizar o caractere no display
  uint8_t x_offset = (ssd->width  - charSize) / 2;
  uint8_t y_offset = (ssd->height - charSize) / 2;

  if (!useRotation) {
    // Para minúsculas: cada byte representa uma linha (já estão "em pé")
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t line = font[index + i];
      for (uint8_t j = 0; j < 8; ++j) {
        bool pixel = (line >> (7 - j)) & 0x01;
        if (pixel) {
          // Desenha um bloco de tamanho (scale x scale)
          for (uint8_t a = 0; a < scale; a++) {
            for (uint8_t b = 0; b < scale; b++) {
              ssd1306_pixel(ssd, x_offset + j * scale + b, y_offset + i * scale + a, true);
            }
          }
        }
      }
    }
  } else {
    // Para maiúsculas e dígitos: os dados estão armazenados por coluna.
    // Para exibi-los com a mesma orientação dos minúsculos
    for (uint8_t i = 0; i < 8; ++i) {
      uint8_t col = font[index + i];
      for (uint8_t j = 0; j < 8; ++j) {
        // Usa a mesma ordem de bits que nos minúsculos
        bool pixel = (col >> (7 - j)) & 0x01;
        if (pixel) {
          uint8_t drawX, drawY;
          drawX = i - 7;
          drawY = 7 - j;
          // Desenha um bloco de tamanho (scale x scale) com a rotação aplicada
          for (uint8_t a = 0; a < scale; a++) {
            for (uint8_t b = 0; b < scale; b++) {
              ssd1306_pixel(ssd, 28 + x_offset + drawX * scale + b, y_offset + drawY * scale + a, true);
            }
          }
        }
      }
    }
  }
}

// Função para desenhar uma string
void ssd1306_draw_string(ssd1306_t *ssd, const char *str, uint8_t x, uint8_t y) {
    while (*str) {
        // Verifica se o próximo caractere cabe na linha
        if (x + 7 > ssd->width) {
            x = 0;
            y += 7;
            if (y + 7 > ssd->height) break;
        }
        ssd1306_draw_char(ssd, *str++, x, y);
        x += 7;
    }
}
