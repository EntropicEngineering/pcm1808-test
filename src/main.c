/*
 * Ingest I2S data from PCM1808 & squirt it out via USB serial
 */

#include <stdio.h>
#include <string.h>
#include <device.h>
#include <drivers/uart.h>
#include <zephyr.h>
#include <usb/usb_device.h>
#include <logging/log.h>
#include <nrfx_i2s.h>

LOG_MODULE_REGISTER(pcm1808_test, LOG_LEVEL_INF);

#define DATA_BUFFER_WORD_COUNT (0x1000)
static uint8_t volatile m_data_buffers[2][DATA_BUFFER_WORD_COUNT * 4] = {0};
static nrfx_i2s_buffers_t m_i2s_buffers = { .p_rx_buffer = (uint32_t *) (m_data_buffers[0]), .p_tx_buffer = NULL};

static struct device *uart_dev;

static struct {
    uint8_t *p_tx_buffer;
    int bytes_remaining;
} volatile serial_status = {0};

static void uart_IRQ_handler(const struct device *dev, void __unused *user_data) {
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (serial_status.bytes_remaining && uart_irq_tx_ready(dev)) {
            int sent = uart_fifo_fill(dev, serial_status.p_tx_buffer, serial_status.bytes_remaining);
            if (sent < serial_status.bytes_remaining) {
                serial_status.p_tx_buffer += sent;
                serial_status.bytes_remaining -= sent;
            } else {
                serial_status.p_tx_buffer = NULL;
                serial_status.bytes_remaining = 0;
                // Disable tx IRQ immediately after data is finished sending
                uart_irq_tx_disable(uart_dev);
            }
        }
    }
}

static void i2s_event_handler(nrfx_i2s_buffers_t const *p_released, uint32_t status) {
    static bool expect_initial_void_buffer = true;
    static uint8_t next_buffer = 1;
    if (status & NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
        if (expect_initial_void_buffer) {
            expect_initial_void_buffer = false;
        } else if (p_released->p_rx_buffer != NULL) {
            next_buffer = (uint8_t) ((uint8_t *) p_released->p_rx_buffer == m_data_buffers[0]);
            // Only enable tx IRQ immediately before sending data
            uart_irq_tx_enable(uart_dev);
            int sent = uart_fifo_fill(uart_dev, (uint8_t *) p_released->p_rx_buffer, DATA_BUFFER_WORD_COUNT * 4);
            if (sent < DATA_BUFFER_WORD_COUNT * 4) {
                serial_status.p_tx_buffer = sent + (uint8_t *) p_released->p_rx_buffer;
                serial_status.bytes_remaining = DATA_BUFFER_WORD_COUNT * 4 - sent;
            } else {
                serial_status.p_tx_buffer = NULL;
                serial_status.bytes_remaining = 0;
                // Disable tx IRQ immediately after data is finished sending
                uart_irq_tx_disable(uart_dev);
            }
        } else {
            LOG_ERR("I2S data corruption occurred");
        }
        m_i2s_buffers.p_rx_buffer = (uint32_t *) m_data_buffers[next_buffer];
        int ret = nrfx_i2s_next_buffers_set(&m_i2s_buffers);
        if (ret != NRFX_SUCCESS) {
            LOG_ERR("I2S error setting next buffer: %u", ret);
        }
    } else {
        LOG_INF("I2S status: %u", status);
    }
}

void main(void) {

    uint32_t baudrate, dtr = 0U;
    int ret;

    nrfx_i2s_config_t i2c_cfg = NRFX_I2S_DEFAULT_CONFIG(15, 14, NRFX_I2S_PIN_NOT_USED, 32, NRFX_I2S_PIN_NOT_USED);
    i2c_cfg.mode = NRF_I2S_MODE_MASTER;
    i2c_cfg.format = NRF_I2S_FORMAT_ALIGNED;
    i2c_cfg.alignment = NRF_I2S_ALIGN_LEFT;
    i2c_cfg.sample_width = NRF_I2S_SWIDTH_24BIT;
    i2c_cfg.channels = NRF_I2S_CHANNELS_STEREO;
    i2c_cfg.mck_setup = NRF_I2S_MCK_32MDIV16;
    i2c_cfg.ratio = NRF_I2S_RATIO_48X;

    ret = nrfx_i2s_init(&i2c_cfg, i2s_event_handler);
    if (ret != NRFX_SUCCESS) {
        LOG_ERR("I2S initialization failed: %u", ret);
        return;
    }
    IRQ_CONNECT(37, NRFX_I2S_DEFAULT_CONFIG_IRQ_PRIORITY, nrfx_i2s_irq_handler, NULL, 0);

    uart_dev = (struct device *) device_get_binding("CDC_ACM_0");
    if (!uart_dev) {
        LOG_ERR("CDC ACM device not found");
        return;
    }

    ret = usb_enable(NULL);
    if (ret != 0) {
        LOG_ERR("Failed to enable USB: %u", ret);
        return;
    }

    LOG_INF("Wait for DTR");

    while (true) {
        uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_DTR, &dtr);
        if (dtr) {
            break;
        } else {
            /* Give CPU resources to low priority threads. */
            k_sleep(K_MSEC(100));
        }
    }

    LOG_INF("DTR set");

    /* Wait 1 sec for the host to do all settings */
    k_busy_wait(1000000);

    ret = uart_line_ctrl_get(uart_dev, UART_LINE_CTRL_BAUD_RATE, &baudrate);
    if (ret) {
        LOG_WRN("Failed to get baudrate, ret code %d", ret);
    } else {
        LOG_INF("Baudrate detected: %u", baudrate);
    }

    uart_irq_callback_set(uart_dev, uart_IRQ_handler);

    ret = nrfx_i2s_start(&m_i2s_buffers, DATA_BUFFER_WORD_COUNT, 0);
    if (ret != NRFX_SUCCESS) LOG_ERR("I2S failed to start: %u", ret);
}
