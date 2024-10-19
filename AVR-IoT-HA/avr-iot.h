//  AVR-IoT support heavily inspired by https://github.com/zst123/avr-iot-medical-storage-monitor
#ifndef _AVR_IOT_H
#define _AVR_IOT_H

// LEDs
#define LED_RED           (PIN_PD0) //TCA0 WO0 
#define LED_YELLOW        (PIN_PD1) //TCA0 WO1 
#define LED_GREEN         (PIN_PD2) //TCA0 WO2 
#define LED_BLUE          (PIN_PD3) //TCA0 WO3 
// Redefines as per silkscreen
#define LED_ERROR         LED_RED
#define LED_DATA          LED_YELLOW
#define LED_CONN          LED_GREEN
#define LED_WIFI          LED_BLUE

// Swtches - Need internal pullup
#define PIN_SW0           (PIN_PF6) // Only if Tools->Reset pin: "GPIO"
#define PIN_SW1           (PIN_PF5)

// I2C/TWI
#define PIN_I2C_SDA       (PIN_PA2)
#define PIN_I2C_SCL       (PIN_PA3)

// SPI
//#define PIN_SPI_MOSI      (PIN_PA4)
//#define PIN_SPI_MISO      (PIN_PA5)
//#define PIN_SPI_SCK       (PIN_PA6)

// USART COM Port
#define SerialCOM         Serial2
#define PIN_CDC_RX        
#define PIN_CDC_TX        

// (Re-)defines as per mikroBus silkscreen
#define SerialClick       Serial1
#define PIN_AN            (PIN_PD7)
#define PIN_RST           (PIN_PA0)
#define PIN_CS            (PIN_PC3)
#define PIN_SCK           PIN_SPI_SCK
#define PIN_SDO           PIN_SPI_MOSI
#define PIN_SDI           PIN_SPI_MISO
#define PIN_PWM           (PIN_PD4)
#define PIN_INT           (PIN_PD6)
#define PIN_RX            (PIN_PC1) //UART1 RX 
#define PIN_TX            (PIN_PC0) //UART1 TX 
#define PIN_SCL           PIN_I2C_SCL
#define PIN_SDA           PIN_I2C_SDA

// WINC1500 (SPI)
#define PIN_WIFI_CS       (PIN_PA7)
#define PIN_WIFI_IRQ      (PIN_PF2)
#define PIN_WIFI_RST      (PIN_PA1)
#define PIN_WIFI_EN       (PIN_PF3)

// TEMT6000
#define PIN_LIGHT_SENSOR  (PIN_PD5) 
float readLightPct(void) {
  //Get percent of maximum value (1023)
  return analogRead(PIN_LIGHT_SENSOR) * 100.f / 1023.f;
}
// MCP9808
#define ADDRESS_I2C_MCP9808 0x18
#define PIN_MCP9808_ALERT   (PIN_PC2)

// ATECC608A
#define ADDRESS_I2C_ATECC608A  0x58

#endif //_AVR_IOT_H
