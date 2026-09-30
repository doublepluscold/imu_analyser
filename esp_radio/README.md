# Прошивки ESP для бездротового IMU-демо

```
IMU-модуль --UART--> ESP-01 (esp-01/) --ESP-NOW--> ESP32-S3 (esp32s3/) --W5500/UDP--> ноутбук (imuview)
```

- `common/` - чистий C++ (framer MTData2, datagram, id_table), спільна бібліотека обох проєктів
- `esp-01/` - slave, PlatformIO, Arduino-ESP8266
- `esp32s3/` - master, PlatformIO, Arduino-ESP32 3.x (pioarduino), W5500; тут же юніт-тести
- `esp32c3/` - еталонні ESP-NOW експерименти, не чіпати

## ПОПЕРЕДЖЕННЯ: UART модуля

На UART модуля сидять OTA-бутлоадер і командний парсер. Випадковий байт може зіпсувати прошивку модуля.

- **TX ESP-01 НІКОЛИ не з'єднувати з RX модуля.** До RX модуля ESP-01 не підключається взагалі.
- Підключення: TX модуля (TTL-вихід мікроконтролера до ADM3232E, **не RS-232**) -> RX ESP-01 (GPIO3), і спільна земля. Більше нічого.
- ESP8266 при старті друкує діагностику на TX (74880 baud). Це була б каша в бутлоадері. Тому TX ESP-01 у бойовому режимі висить у повітрі.
- Прошивка slave нічого не пише в UART у бойовому режимі (`DEBUG_LOG 0`, UART в режимі RX-only).
- Налагодження (`DEBUG_LOG 1`, env `esp01_debug`): статистика йде на TX ESP-01 на USB-UART адаптер, потік модуля на RX подається з іншого джерела, TX з модулем не з'єднаний.
- Світлодіод GPIO2 мигає при кожному надісланому кадрі в обох режимах.
- ESP-01 живити від 3.3 В LDO, що дає до ~300 мА піками.

## Збірка

```bash
cd esp-01   && pio run                # бойова збірка
cd esp-01   && pio run -e esp01_debug # стенд, статистика на TX
cd esp32s3  && pio run
cd esp32s3  && pio test -e native     # юніт-тести на ПК
```

## Перший запуск

1. Прошити master, у USB-консолі побачити `master up: wifi mac=...`.
2. Прошити slave (усі однаковою прошивкою). За замовчуванням slave шле на broadcast. Для unicast: вписати MAC master у `MASTER_MAC` в `esp-01/src/config.h`.
3. Канал `WIFI_CHANNEL` (за замовчуванням 1) однаковий у `esp-01/src/config.h` і `esp32s3/src/config.h`.
4. На ноутбуці: IP `192.168.50.1/24`, `uv run imu net --seconds 10 --no-record`.

Скидання таблиці ID на master: утримати BOOT (GPIO0) 5 с при старті, або `reset_ids` у USB-консолі.

## Припущення (бо відповідей на п. 9 брифу не було)

- W5500 піни за вікі Waveshare ESP32-S3-ETH: SCK 13, MISO 12, MOSI 11, CS 14, INT 10, RST 9. LED - WS2812 на GPIO21.
- Wi-Fi канал 1, slave шле broadcast, поки не задано `MASTER_MAC`.
- PlatformIO. Master: `esp32-s3-devkitc-1`, flash 8 МБ.
- Master `192.168.50.2/24`, ноутбук `192.168.50.1`, UDP 5005; broadcast-режим через `UDP_USE_BROADCAST`.
- Схема ESP-01 <-> модуль: точку TTL-сигналу вибирає інженер, прошивка її не визначає.
