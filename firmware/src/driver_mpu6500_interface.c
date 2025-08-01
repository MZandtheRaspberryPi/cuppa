
#include "driver_mpu6500_interface.h"

static inline void cs_select() {
  asm volatile("nop \n nop \n nop");
  gpio_put(PIN_CS, 0);  // Active low
  asm volatile("nop \n nop \n nop");
}

static inline void cs_deselect() {
  asm volatile("nop \n nop \n nop");
  gpio_put(PIN_CS, 1);
  asm volatile("nop \n nop \n nop");
}

/**
 * @defgroup mpu6500_interface_driver mpu6500 interface driver function
 * @brief    mpu6500 interface driver modules
 * @ingroup  mpu6500_driver
 * @{
 */

/**
 * @brief  interface iic bus init
 * @return status code
 *         - 0 success
 *         - 1 iic init failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_init(void) { return 1; }

/**
 * @brief  interface iic bus deinit
 * @return status code
 *         - 0 success
 *         - 1 iic deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_iic_deinit(void) { return 1; }

/**
 * @brief      interface iic bus read
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf,
                                   uint16_t len) {
  return 1;
}

/**
 * @brief     interface iic bus write
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf,
                                    uint16_t len) {
  return 1;
}

/**
 * @brief  interface spi bus init
 * @return status code
 *         - 0 success
 *         - 1 spi init failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_init(void) {
  stdio_init_all();

  // This example will use SPI0 at 1MHz.
  spi_init(SPI_PORT, 1000 * 1000);
  gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
  // Make the SPI pins available to picotool
  bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

  // Chip select is active-low, so we'll initialise it to a driven-high state
  gpio_init(PIN_CS);
  gpio_set_dir(PIN_CS, GPIO_OUT);
  gpio_put(PIN_CS, 1);
  // Make the CS pin available to picotool
  bi_decl(bi_1pin_with_name(PIN_CS, "SPI CS"));

  return 0;
}

/**
 * @brief  interface spi bus deinit
 * @return status code
 *         - 0 success
 *         - 1 spi deinit failed
 * @note   none
 */
uint8_t mpu6500_interface_spi_deinit(void) {
  spi_deinit(SPI_PORT);
  return 0;
}

/**
 * @brief      interface spi bus read
 * @param[in]  reg register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t mpu6500_interface_spi_read(uint8_t reg, uint8_t *buf, uint16_t len) {
  reg |= READ_BIT;
  cs_select();
  spi_write_blocking(SPI_PORT, &reg, 1);
  // sleep_ms(10);
  spi_read_blocking(SPI_PORT, 0, buf, len);
  cs_deselect();
  // sleep_ms(10);
  return 0;
}

/**
 * @brief     interface spi bus write
 * @param[in] reg register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t mpu6500_interface_spi_write(uint8_t reg, uint8_t *buf, uint16_t len) {
  reg |= 0x7f;  // remove read bit as this is a write
  cs_select();
  spi_write_blocking(SPI_PORT, &reg, 1);
  spi_write_blocking(SPI_PORT, buf, len);
  cs_deselect();
  // sleep_ms(10);
  return 0;
}

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void mpu6500_interface_delay_ms(uint32_t ms) { sleep_ms(ms); }

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void mpu6500_interface_debug_print(const char *const fmt, ...) { printf(fmt); }

/**
 * @brief     interface receive callback
 * @param[in] type irq type
 * @note      none
 */
void mpu6500_interface_receive_callback(uint8_t type) { return; }

/**
 * @brief     interface dmp tap callback
 * @param[in] count tap count
 * @param[in] direction tap direction
 * @note      none
 */
void mpu6500_interface_dmp_tap_callback(uint8_t count, uint8_t direction) {
  return;
}

/**
 * @brief     interface dmp orient callback
 * @param[in] orientation dmp orientation
 * @note      none
 */
void mpu6500_interface_dmp_orient_callback(uint8_t orientation) { return; }