#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"
#include "spi_mcp23s17.h"

static const char TAG[] = "main";

void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "Initializing bus SPI3...");
    spi_bus_config_t buscfg={   // (1)ピン設定用の構造体
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    //Initialize the SPI bus
    ret = spi_bus_initialize(MCP23S17_HOST, &buscfg, SPI_DMA_DISABLED); // (2)バスの初期化
    ESP_ERROR_CHECK(ret);

    mcp23s17_config_t mcp23s17_config = {   // (3)ピンとSPIホスト設定用の構造体
        .cs_io = PIN_NUM_CS,
        .host = MCP23S17_HOST,
        .miso_io = PIN_NUM_MISO,
        .int_io = PIN_NUM_INT,
        .intr_used = false,
        .led = 0,
    };

#ifdef CONFIG_INTR_USED
    mcp23s17_config.intr_used = true; // (4) 割り込みを使う場合
    gpio_install_isr_service(0);
#endif

    mcp23s17_handle_t mcp23s17_handle;

    ESP_LOGI(TAG, "Initializing device...");
    ret = spi_mcp23s17_init(&mcp23s17_config, &mcp23s17_handle);    // (5)MCP23S17の初期化
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_write(mcp23s17_handle, IOCON, IOCON_COMMAND);    // IOCONの設定
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_write(mcp23s17_handle, IODIRA, IO_ALL_OUTPUT);   // GPIOA方向レジスタの設定
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_write(mcp23s17_handle, IODIRB, IO_INPUT_D7_ONLY);    // GPIOB方向レジスタの設定
    ESP_ERROR_CHECK(ret);
    ret = spi_mcp23s17_write(mcp23s17_handle, GPPUB, IO_PULLUP_D7_ONLY);    // GIPOBプルアップレジスタの設定
    ESP_ERROR_CHECK(ret);

    uint8_t ex = 0;
    uint8_t input_data;

    // GPIOA: オール出力, GPIOB: D7だけが入力、それ以外は出力
    // GPIOA: 定期的にGPIOA0ビットを反転させてLチカする
    // GPIOB:は D7をレジスタ設定でプルアップしておき、GNDに落とすとD7がLになりGPIOB0でLEDを点灯させる

    spi_mcp23s17_write(mcp23s17_handle, GPIOB, 1);

    while (1) {
        vTaskDelay(100);
        spi_mcp23s17_write(mcp23s17_handle, GPIOA, ex & 1); // GPIOA0でLチカ
        ret = spi_mcp23s17_read(mcp23s17_handle, GPIOB, &input_data);
        if (input_data & 0x80)
        {
           spi_mcp23s17_write(mcp23s17_handle, GPIOB, 1); // GPIOB0で消灯
        }
        else
        {
           spi_mcp23s17_write(mcp23s17_handle, GPIOB, 0); // GPIOB0で点灯
        }
        ex++;
    }
}