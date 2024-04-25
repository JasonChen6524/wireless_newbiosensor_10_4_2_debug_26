#include "sl_emlib_gpio_simple_init.h"
#include "sl_emlib_gpio_init_EN5V_config.h"
#include "sl_emlib_gpio_init_G_INT1_config.h"
#include "sl_emlib_gpio_init_MFIO_config.h"
#include "sl_emlib_gpio_init_bio_reset_config.h"
#include "em_gpio.h"
#include "em_cmu.h"

void sl_emlib_gpio_simple_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_EN5V_PORT,
                  SL_EMLIB_GPIO_INIT_EN5V_PIN,
                  SL_EMLIB_GPIO_INIT_EN5V_MODE,
                  SL_EMLIB_GPIO_INIT_EN5V_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_G_INT1_PORT,
                  SL_EMLIB_GPIO_INIT_G_INT1_PIN,
                  SL_EMLIB_GPIO_INIT_G_INT1_MODE,
                  SL_EMLIB_GPIO_INIT_G_INT1_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_MFIO_PORT,
                  SL_EMLIB_GPIO_INIT_MFIO_PIN,
                  SL_EMLIB_GPIO_INIT_MFIO_MODE,
                  SL_EMLIB_GPIO_INIT_MFIO_DOUT);

  GPIO_PinModeSet(SL_EMLIB_GPIO_INIT_BIO_RESET_PORT,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_PIN,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_MODE,
                  SL_EMLIB_GPIO_INIT_BIO_RESET_DOUT);
}
