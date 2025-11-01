/**
* @file ws2812b_control.hpp
* 
* @brief Implementation of the WS2812B class
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>


// WS2812B LED Strip Control Class
class WS2812B
{
private:
  int spi_fd;
  uint8_t* spi_buffer;
  int num_leds;

  // WS2812B timing requirements (from datasheet):
  // '1' bit: 0.8us high, 0.45us low (total 1.25us)
  // '0' bit: 0.4us high, 0.85us low (total 1.25us)
  // 
  // Using SPI at 2.5 MHz (400ns per SPI bit)
  // '1': 110 (800ns high, 400ns low) = (0.8us, 0.4us)
  // '0': 100 (400ns high, 800ns low) = (0.4us, 0.8us)
  const uint8_t BIT_0 = 0b100;
  const uint8_t BIT_1 = 0b110;


  /**
   * @brief Encode a single byte into SPI bit pattern
   * 
   * @param byte The byte to encode
   * @param output Pointer to output buffer (must be at least 8 bytes)
   */
  void encode_byte(uint8_t byte, uint8_t* output)
  {
    // Each WS2812B byte becomes 24 SPI bits (3 bytes)
    output[0] = 0;
    output[1] = 0;
    output[2] = 0;

    for (int i = 0; i < 8; i++)
    {
      uint8_t pattern;
      if (byte & (0x80 >> i))
      {
        pattern = BIT_1;
      }
      else
      {
        pattern = BIT_0;
      }

      // Pack 3-bit pattern into the output buffer
      int bit_pos = i * 3;
      int byte_pos = bit_pos / 8;
      int shift = 5 - (bit_pos % 8);

      if (shift >= 0)
      {
        output[byte_pos] |= (pattern << shift);
      }
      else
      {
        output[byte_pos] |= (pattern >> -shift);
        output[byte_pos + 1] |= (pattern << (8 + shift));
      }
    }
  }


public:
  /**
   * @brief WS2812B constructor
   * 
   * @param spi_device The SPI device path (e.g., "/dev/spidev1.0")
   * @param led_count Number of LEDs in the strip
   */
  WS2812B(const char* spi_device, int led_count)
  {
    num_leds = led_count;

    // Open SPI device
    spi_fd = open(spi_device, O_RDWR);
    if (spi_fd < 0)
    {
      perror("Failed to open SPI device");
      exit(1);
    }

    // Configure SPI
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = 2500000; // 2.5 MHz

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) < 0)
    {
      perror("Failed to set SPI mode");
      exit(1);
    }

    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
    {
      perror("Failed to set bits per word");
      exit(1);
    }

    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
    {
      perror("Failed to set speed");
      exit(1);
    }

    // Allocate buffer: 3 bytes (RGB) * 8 bits * 1 byte per bit + reset
    spi_buffer = (uint8_t*)malloc(num_leds * 24 + 300); // +300 for reset
    memset(spi_buffer, 0, num_leds * 24 + 300);
  }


  /**
   * @brief Set color of an individual pixel
   * 
   * @param index Pixel index
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   */
  void set_pixel(int index, uint8_t r, uint8_t g, uint8_t b)
  {
    if (index < 0 || index >= num_leds)
      return;

    int offset = index * 24;

    // WS2812B order is GRB
    encode_byte(g, &spi_buffer[offset]);
    encode_byte(r, &spi_buffer[offset + 8]);
    encode_byte(b, &spi_buffer[offset + 16]);
  }


  /**
   * @brief Send the buffer to the LED strip
   * 
   */
  void show()
  {
    // Add reset signal (low for >50us)
    memset(&spi_buffer[num_leds * 24], 0, 300);

    struct spi_ioc_transfer transfer;
    transfer.tx_buf = (unsigned long)spi_buffer;
    transfer.rx_buf = 0;
    transfer.len = num_leds * 24 + 300;
    transfer.speed_hz = 2500000;
    transfer.delay_usecs = 0;
    transfer.bits_per_word = 8;
    transfer.cs_change = 0;
    transfer.pad = 0;

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &transfer) < 0)
    {
      perror("Failed to send SPI message");
    }
  }


  /**
   * @brief Clear all LEDs
   *
   */
  void clear()
  {
    for (int i = 0; i < num_leds; i++)
      set_pixel(i, 0, 0, 0);

    show();
  }


  /**
   * @brief Fill all LEDs with a specific color
   * 
   * @param r Red component (0-255)
   * @param g Green component (0-255)
   * @param b Blue component (0-255)
   */
  void fill(uint8_t r, uint8_t g, uint8_t b)
  {
    for (int i = 0; i < num_leds; i++)
      set_pixel(i, r, g, b);

    show();
  }


  /**
   * @brief WS2812B destructor
   */
  ~WS2812B()
  {
    close(spi_fd);
    free(spi_buffer);
    spi_buffer = NULL;
  }
};
