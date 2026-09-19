#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_pwr.h"
#include "misc.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>

/* ===================== Pins ===================== */
#define HX711_DOUT_PIN   GPIO_Pin_0
#define HX711_SCK_PIN    GPIO_Pin_1
#define HX711_GPIO       GPIOB

#define LCD_ADDR         0x27
#define LCD_BACKLIGHT    0x08
#define LCD_EN           0x04
#define LCD_RW           0x02
#define LCD_RS           0x01

#define BUTTON_PIN       GPIO_Pin_0
#define BUTTON_GPIO      GPIOA

/* ===================== Scale configuration ===================== */
#define HX711_DEADBAND       1500
#define WEIGHT_ZERO_LIMIT    0.002f
#define HX711_SCALE_DEFAULT  417000.0f

/* ===================== Queue message ===================== */
typedef struct {
    float weight_kg;
    uint8_t tare_event;
} ScaleMessage_t;

static QueueHandle_t xDisplayQueue = NULL;
static QueueHandle_t xUartQueue = NULL;

/* ===================== Globals ===================== */
static uint8_t backlight_mask = LCD_BACKLIGHT;
static int32_t hx_offset = 0;
static float hx_scale = HX711_SCALE_DEFAULT;

/* ===================== Delay helpers ===================== */
static void delay_us(uint32_t us)
{
    while (us--) {
        for (volatile int i = 0; i < 7; i++);
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

/* ===================== HX711 ===================== */
static int32_t HX711_ReadRaw(void)
{
    uint32_t count = 0;

    while (GPIO_ReadInputDataBit(HX711_GPIO, HX711_DOUT_PIN));

    for (int i = 0; i < 24; i++) {
        GPIO_SetBits(HX711_GPIO, HX711_SCK_PIN);
        delay_us(1);

        count <<= 1;

        GPIO_ResetBits(HX711_GPIO, HX711_SCK_PIN);
        delay_us(1);

        if (GPIO_ReadInputDataBit(HX711_GPIO, HX711_DOUT_PIN)) {
            count++;
        }
    }

    /* 25th pulse -> gain 128, channel A */
    GPIO_SetBits(HX711_GPIO, HX711_SCK_PIN);
    delay_us(1);
    GPIO_ResetBits(HX711_GPIO, HX711_SCK_PIN);

    /* Sign extension from 24-bit to signed 32-bit */
    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return (int32_t)count;
}

/* ===================== I2C -> PCF8574 ===================== */
static int I2C_WriteByteToPcf(uint8_t data)
{
    uint8_t addr7 = LCD_ADDR;

    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

    I2C_GenerateSTART(I2C1, ENABLE);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C1, addr7 << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C1, data);
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0;
}

static void pcf8574_write(uint8_t value)
{
    I2C_WriteByteToPcf(value | backlight_mask);
}

static void pcf8574_set_backlight(uint8_t on)
{
    backlight_mask = on ? LCD_BACKLIGHT : 0;
    pcf8574_write(0);
}

/* ===================== LCD ===================== */
static void lcd_write4(uint8_t value, uint8_t mode)
{
    uint8_t data = (mode ? LCD_RS : 0) & ~LCD_RW;

    data |= (value & 0xF0) | backlight_mask;

    I2C_WriteByteToPcf(data);
    I2C_WriteByteToPcf(data | LCD_EN);
    delay_us(1);
    I2C_WriteByteToPcf(data & ~LCD_EN);
    delay_us(40);
}

static void lcd_send(uint8_t value, uint8_t mode)
{
    lcd_write4(value & 0xF0, mode);
    lcd_write4((value << 4) & 0xF0, mode);
}

static void LCD_Init(void)
{
    delay_ms(40);

    lcd_write4(0x30, 0);
    delay_ms(5);

    lcd_write4(0x30, 0);
    delay_ms(2);

    lcd_write4(0x30, 0);
    delay_ms(2);

    lcd_write4(0x20, 0);
    delay_ms(2);

    lcd_send(0x28, 0);
    lcd_send(0x08, 0);
    lcd_send(0x01, 0);
    delay_ms(3);
    lcd_send(0x06, 0);
    lcd_send(0x0C, 0);
}

static void LCD_SetCursor(uint8_t row, uint8_t col)
{
    lcd_send(0x80 | (row ? 0x40 : 0) | col, 0);
}

static void LCD_Print(const char *str)
{
    while (*str) {
        lcd_send((uint8_t)*str++, 1);
    }
}

/* ===================== Hardware init ===================== */
static void I2C1_Init_HW(void)
{
    GPIO_InitTypeDef gpio;
    I2C_InitTypeDef i2c;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    i2c.I2C_ClockSpeed = 100000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = 0;
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;

    I2C_Init(I2C1, &i2c);
    I2C_Cmd(I2C1, ENABLE);
}

static void UART_Init_HW(void)
{
    GPIO_InitTypeDef tx;
    GPIO_InitTypeDef rx;
    USART_InitTypeDef uart;

    RCC_APB2PeriphClockCmd(
        RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,
        ENABLE
    );

    tx.GPIO_Pin = GPIO_Pin_9;
    tx.GPIO_Mode = GPIO_Mode_AF_PP;
    tx.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &tx);

    rx.GPIO_Pin = GPIO_Pin_10;
    rx.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    rx.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &rx);

    uart.USART_BaudRate = 9600;
    uart.USART_WordLength = USART_WordLength_8b;
    uart.USART_StopBits = USART_StopBits_1;
    uart.USART_Parity = USART_Parity_No;
    uart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &uart);
    USART_Cmd(USART1, ENABLE);
}

static void UART_SendString(const char *str)
{
    while (*str) {
        while (!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
        USART_SendData(USART1, (uint16_t)*str++);
    }
}

static void HX711_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Pin = HX711_DOUT_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(HX711_GPIO, &gpio);

    gpio.GPIO_Pin = HX711_SCK_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(HX711_GPIO, &gpio);

    GPIO_ResetBits(HX711_GPIO, HX711_SCK_PIN);
}

/* ===================== TARE button ===================== */
static void Button_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    gpio.GPIO_Pin = BUTTON_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(BUTTON_GPIO, &gpio);
}

static uint8_t Button_Pressed(void)
{
    if (GPIO_ReadInputDataBit(BUTTON_GPIO, BUTTON_PIN) == Bit_RESET) {
        vTaskDelay(pdMS_TO_TICKS(20));

        if (GPIO_ReadInputDataBit(BUTTON_GPIO, BUTTON_PIN) == Bit_RESET) {
            while (GPIO_ReadInputDataBit(BUTTON_GPIO, BUTTON_PIN) == Bit_RESET) {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            return 1;
        }
    }

    return 0;
}

/* ===================== FreeRTOS tasks ===================== */

static void Task_ReadWeight(void *pvParameters)
{
    ScaleMessage_t msg;
    int64_t sum = 0;
    char text[48];

    (void)pvParameters;

    UART_SendString("Calibrating offset...\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));

    for (int i = 0; i < 10; i++) {
        sum += HX711_ReadRaw();
        vTaskDelay(pdMS_TO_TICKS(40));
    }

    hx_offset = (int32_t)(sum / 10);

    snprintf(text, sizeof(text), "Offset=%ld\r\n", (long)hx_offset);
    UART_SendString(text);

    msg.weight_kg = 0.0f;
    msg.tare_event = 0;

    for (;;) {
        msg.tare_event = 0;

        /* TARE */
        if (Button_Pressed()) {
            int64_t tare_sum = 0;

            /* Average several samples to make TARE less noisy */
            for (int i = 0; i < 5; i++) {
                tare_sum += HX711_ReadRaw();
                vTaskDelay(pdMS_TO_TICKS(20));
            }

            hx_offset = (int32_t)(tare_sum / 5);
            msg.tare_event = 1;
        }

        int32_t raw = HX711_ReadRaw() - hx_offset;
        float w;

        /* Anti-vibration / deadband filter */
        if ((raw > -HX711_DEADBAND) && (raw < HX711_DEADBAND)) {
            w = 0.0f;
        } else {
            w = (float)raw / hx_scale;
        }

        if (w < 0.0f) {
            w = 0.0f;
        }

        msg.weight_kg = w;

        xQueueOverwrite(xDisplayQueue, &msg);
        xQueueOverwrite(xUartQueue, &msg);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void Task_Display(void *pvParameters)
{
    ScaleMessage_t msg;
    float last = -1.0f;
    char line[17];

    (void)pvParameters;

    for (;;) {
        if (xQueueReceive(xDisplayQueue, &msg, portMAX_DELAY) == pdPASS) {

            if (msg.tare_event) {
                pcf8574_set_backlight(1);
                LCD_SetCursor(1, 0);
                LCD_Print(" TARE OK        ");
                vTaskDelay(pdMS_TO_TICKS(600));
                LCD_SetCursor(1, 0);
                LCD_Print("                ");
            }

            float w_display =
                (msg.weight_kg < WEIGHT_ZERO_LIMIT) ? 0.0f : msg.weight_kg;

            if (w_display < WEIGHT_ZERO_LIMIT) {
                pcf8574_set_backlight(0);
            } else {
                pcf8574_set_backlight(1);
            }

            if (fabsf(w_display - last) > 0.0001f) {
                last = w_display;

                snprintf(line, sizeof(line), "%5.3f kg", w_display);
                LCD_SetCursor(0, 0);
                LCD_Print("                ");
                LCD_SetCursor(0, 0);
                LCD_Print(line);
            }
        }
    }
}

static void Task_UART(void *pvParameters)
{
    ScaleMessage_t msg;
    char buf[64];

    (void)pvParameters;

    for (;;) {
        if (xQueueReceive(xUartQueue, &msg, portMAX_DELAY) == pdPASS) {

            if (msg.tare_event) {
                UART_SendString("TARE OK\r\n");
            }

            if (msg.weight_kg >= WEIGHT_ZERO_LIMIT) {
                snprintf(
                    buf,
                    sizeof(buf),
                    "W=%.3f kg\r\n",
                    msg.weight_kg
                );
                UART_SendString(buf);
            }

            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

/* ===================== Tickless Idle / STOP mode hooks ===================== */
void PreSleepProcessing(uint32_t xExpectedIdleTime)
{
    (void)xExpectedIdleTime;

    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);

    SystemInit();
}

void PostSleepProcessing(uint32_t xExpectedIdleTime)
{
    (void)xExpectedIdleTime;
}

/* ===================== Main ===================== */
int main(void)
{
    SystemInit();

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    HX711_GPIO_Init();
    UART_Init_HW();
    I2C1_Init_HW();
    Button_Init();
    LCD_Init();

    LCD_SetCursor(0, 0);
    LCD_Print("0.000 kg");

    xDisplayQueue = xQueueCreate(1, sizeof(ScaleMessage_t));
    xUartQueue = xQueueCreate(1, sizeof(ScaleMessage_t));

    if ((xDisplayQueue == NULL) || (xUartQueue == NULL)) {
        UART_SendString("Queue create failed!\r\n");
        while (1);
    }

    xTaskCreate(Task_ReadWeight, "ReadW", 256, NULL, 3, NULL);
    xTaskCreate(Task_Display,    "Disp",  256, NULL, 2, NULL);
    xTaskCreate(Task_UART,       "UART",  256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
