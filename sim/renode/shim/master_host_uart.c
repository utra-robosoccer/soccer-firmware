/* Renode-only replacement for the master's USB-CDC layer.
 *
 * Renode's STM32F4 model has no USB OTG device, so the host link (normally USB CDC) is carried
 * on USART2 instead. This file stands in for USB_DEVICE/App/usbd_cdc_if.c + usb_device.c and
 * the ST USB stack. Everything above it -- usb_link.c (TX slot ring, RX ring, frame scanner),
 * master_app.c, the whole protocol -- is the production code, unmodified.
 *
 * What this does NOT exercise: the USB stack, endpoint timing, 64 B packetisation, ZLPs.
 */
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usb_link.h"

/* stm32f4xx_it.c still references the OTG handle; nothing here ever uses it. */
PCD_HandleTypeDef hpcd_USB_OTG_FS;
USBD_HandleTypeDef hUsbDeviceFS;

#define HOST_UART USART2

void MX_USB_DEVICE_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    HOST_UART->BRR = 0x0010;   /* fastest divisor: Renode paces UART bytes by baud, and a host link
                                  that mimics a 115200 wire would take ~25 ms per 270 B command */
    HOST_UART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* Same ISR-context role as CDC_Receive_FS: bytes go straight into the production RX ring. */
void USART2_IRQHandler(void)
{
    while (HOST_UART->SR & USART_SR_RXNE) {
        uint8_t b = (uint8_t)HOST_UART->DR;
        usb_rx_from_isr(&b, 1u);
    }
}

/* Whole-frame blocking write. usb_link.c treats USBD_OK as "frame handed to the stack". */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
    for (uint16_t i = 0; i < Len; i++) {
        while (!(HOST_UART->SR & USART_SR_TXE)) { }
        HOST_UART->DR = Buf[i];
    }
    return USBD_OK;
}
