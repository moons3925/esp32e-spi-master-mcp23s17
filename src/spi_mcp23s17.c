#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <unistd.h>
#include "esp_log.h"
#include <sys/param.h>
#include "sdkconfig.h"
#include "spi_mcp23s17.h"

struct mcp23s17_context_t{
    mcp23s17_config_t *cfg;        ///< Configuration by the caller.
    spi_device_handle_t spi;    ///< SPI device handle
    xSemaphoreHandle ready_sem; ///< Semaphore for ready signal
};

typedef struct mcp23s17_context_t mcp23s17_context_t;

#define MCP23S17_CLK_FREQ 10000000 // 10MHz
#define MCP23S17_CS_PIN PIN_NUM_CS // chip select pin

#define MCP23S17_INPUT_DELAY_NS   ((1000*1000*1000/MCP23S17_CLK_FREQ)/2)
#define MCP23S17_BUSY_TIMEOUT_MS  5

#define CMD_WRITE 0x40  // A0,A1,A2=GNDに接続した場合のハードウェアアドレス
#define CMD_READ (CMD_WRITE | 1)

static const char TAG[] = "spi_mcp23s17";

void cs_high(spi_transaction_t* t)
{
    ESP_EARLY_LOGV(TAG, "cs high %d.", ((mcp23s17_context_t*)t->user)->cfg->cs_io);
    gpio_set_level(((mcp23s17_context_t*)t->user)->cfg->cs_io, 1);
}

void cs_low(spi_transaction_t* t)
{
    gpio_set_level(((mcp23s17_context_t*)t->user)->cfg->cs_io, 0);
    ESP_EARLY_LOGV(TAG, "cs low %d.", ((mcp23s17_context_t*)t->user)->cfg->cs_io);
}

void ready_negative_edge_isr(void* arg)
{
    mcp23s17_context_t* ctx = (mcp23s17_context_t*)arg;
    xSemaphoreGive(ctx->ready_sem);
    ESP_EARLY_LOGV(TAG, "ready detected.");

    ctx->cfg->led = true;

}

esp_err_t spi_mcp23s17_init(mcp23s17_config_t *cfg, mcp23s17_context_t** out_ctx)
{
    esp_err_t err = ESP_OK;
    if (cfg->intr_used && cfg->host == SPI1_HOST) {
        ESP_LOGE(TAG, "interrupt cannot be used on SPI1 host.");
        return ESP_ERR_INVALID_ARG;
    }

    mcp23s17_context_t* ctx = (mcp23s17_context_t*)malloc(sizeof(mcp23s17_context_t));
    if (!ctx) return ESP_ERR_NO_MEM;

    *ctx = (mcp23s17_context_t) {
        .cfg = cfg,
    };

    spi_device_interface_config_t devcfg={
        .command_bits = 16,
        .clock_speed_hz = MCP23S17_CLK_FREQ,
        .mode = 0,          //SPI mode 0
        .spics_io_num = MCP23S17_CS_PIN,
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = cs_low,
        .post_cb = cs_high,
        .input_delay_ns = MCP23S17_INPUT_DELAY_NS,
        .cs_ena_pretrans = 1,
        .cs_ena_posttrans = 1,
    };
    //Attach the MCP23S17 to the SPI bus
    err = spi_bus_add_device(ctx->cfg->host, &devcfg, &ctx->spi);
    if  (err != ESP_OK) {
        goto cleanup;
    }

    gpio_set_level(ctx->cfg->cs_io, 1);
    gpio_config_t cs_cfg = {
        .pin_bit_mask = BIT64(ctx->cfg->cs_io),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&cs_cfg);

    if (ctx->cfg->intr_used) {
        ctx->ready_sem = xSemaphoreCreateBinary();
        if (ctx->ready_sem == NULL) {
            err = ESP_ERR_NO_MEM;
            goto cleanup;
        }
        cs_cfg.pin_bit_mask = BIT64(ctx->cfg->int_io),
        cs_cfg.mode = GPIO_MODE_INPUT,
        cs_cfg.intr_type = GPIO_INTR_NEGEDGE;
        gpio_config(&cs_cfg);

        gpio_set_intr_type(ctx->cfg->int_io, GPIO_INTR_NEGEDGE); // 立下りエッジで割り込みが入る
        err = gpio_isr_handler_add(ctx->cfg->int_io, ready_negative_edge_isr, ctx);
        if (err != ESP_OK) {
            goto cleanup;
        }
        gpio_intr_enable(ctx->cfg->int_io);
    }
    *out_ctx = ctx;
    return ESP_OK;

cleanup:
    if (ctx->spi) {
        spi_bus_remove_device(ctx->spi);
        ctx->spi = NULL;
    }
    if (ctx->ready_sem) {
        vSemaphoreDelete(ctx->ready_sem);
        ctx->ready_sem = NULL;
    }
    free(ctx);
    return err;
}

esp_err_t spi_mcp23s17_deinit(mcp23s17_context_t* ctx)
{
    spi_bus_remove_device(ctx->spi);
    if (ctx->cfg->intr_used) {
        vSemaphoreDelete(ctx->ready_sem);
    }
    free(ctx);
    return ESP_OK;
}

esp_err_t spi_mcp23s17_read(mcp23s17_context_t* ctx, uint8_t addr, uint8_t* out_data)
{
    spi_transaction_t t = {
        .cmd = CMD_READ << 8 | addr,
        .length = 0,
        .rxlength = 8,
        .flags = SPI_TRANS_USE_RXDATA,
        .user = ctx,
    };
    esp_err_t err = spi_device_polling_transmit(ctx->spi, &t);
    if (err!= ESP_OK) return err;

    *out_data = t.rx_data[0];
    return ESP_OK;
}

esp_err_t spi_mcp23s17_write(mcp23s17_context_t* ctx, uint8_t addr, uint8_t data)
{
    esp_err_t err;
    err = spi_device_acquire_bus(ctx->spi, portMAX_DELAY);
    if (err != ESP_OK) return err;

    spi_transaction_t t = {
        .cmd = CMD_WRITE << 8 | addr,
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
        .user = ctx,
    };
    err = spi_device_polling_transmit(ctx->spi, &t);
    spi_device_release_bus(ctx->spi);
    return err;
}
