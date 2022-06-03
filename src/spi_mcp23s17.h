
#pragma once
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#define MCP23S17_HOST SPI3_HOST // SPI3を使う
#define PIN_NUM_CS 5 // GPIO5
#define PIN_NUM_CLK 18 // GPIO18
#define PIN_NUM_MISO 19 // GPIO19
#define PIN_NUM_MOSI 21 // GPIO21
#define PIN_NUM_INT 4 // GPIO4

#define MCP23S17_ADRS 0x40  // A0,1,2 端子をGNDに接続した場合のハードウェアアドレス

// BANKビットの値によってレジスタアドレスがかわる
// BANK=0時のレジスタアドレス

#define IOCON 0x0a      // コンフィグレーションレジスタ
#define IODIRA 0x00     // IO方向レジスタA
#define IODIRB 0x01     // IO方向レジスタB
#define GPINTENA 0x04      // 状態変化割り込み　制御レジスタA
#define GPINTENB 0x05      // 状態変化割り込み　制御レジスタB
#define DEFVALA 0x06      // 状態変化割り込み用　規定値レジスタA
#define DEFVALB 0x07      // 状態変化割り込み用　規定値レジスタB
#define INTCONA 0x08      // 状態変化割り込み用　比較指定レジスタA
#define INTCONB 0x09      // 状態変化割り込み用　比較指定レジスタB
#define GPPUA 0x0c      // プルアップレジスタA
#define GPPUB 0x0d      // プルアップレジスタB
#define INTFA 0x0e      // フラグレジスタA
#define INTFB 0x0f      // フラグレジスタB
#define INTCAPA 0x10      // 割り込みキャプチャレジスタA
#define INTCAPB 0x11      // 割り込みキャプチャレジスタB
#define GPIOA 0x12      // ポートレジスタA
#define GPIOB 0x13      // ポートレジスタB

#define IO_ALL_OUTPUT 0x00  // 方向レジスタへの全ビット出力設定値
#define IO_ALL_INPUT 0xff   // 方向レジスタへの全ビット入力設定値
#define IO_INPUT_D7_ONLY 0x80   // 方向レジスタへの入出力設定値（D7のみ入力でそれ以外は出力）
#define IO_PULLUP_D7_ONLY 0x80  // プルアップレジスタへの設定値（D7のみプルアップする）
#define IO_INTERRUPT_D7_ONLY 0x80  // 状態変化割り込みレジスタへの設定値（D7変化で割り込み）
#define IO_COMPARE_D7 0x80

#define IOCON_COMMAND 0x28 // シーケンシャルにしない、ハードウェアアドレス有効

//#define CONFIG_INTR_USED    // 割り込みを使う場合に定義する

typedef struct {
    spi_host_device_t host;
    gpio_num_t cs_io;
    gpio_num_t miso_io;
    gpio_num_t int_io;  // 割り込みに使うGPIO
    bool intr_used; // true で割り込みを使う
    bool led;   // 割り込みが入るたびにセットするフラグ
} mcp23s17_config_t;

typedef struct mcp23s17_context_t* mcp23s17_handle_t;

esp_err_t spi_mcp23s17_init(mcp23s17_config_t *config, mcp23s17_handle_t* out_handle);
esp_err_t spi_mcp23s17_deinit(mcp23s17_handle_t handle);
esp_err_t spi_mcp23s17_read(mcp23s17_handle_t handle, uint8_t addr, uint8_t* out_data);
esp_err_t spi_mcp23s17_write(mcp23s17_handle_t handle, uint8_t addr, uint8_t data);
void cs_high(spi_transaction_t* t);
void cs_low(spi_transaction_t* t);
void ready_negative_edge_isr(void* arg);
