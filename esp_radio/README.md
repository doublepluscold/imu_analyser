# Прошивки ESP для бездротового IMU-демо

```
IMU-модуль --UART--> ESP32-C3 (esp32c3-slave/) --ESP-NOW--> ESP32-S3 (esp32s3/) --USB--> ноутбук (imuview)
                                                                       \--W5500/UDP--> (старий режим, OUTPUT_USB 0)
```

**Не працює / не знаю, де обрив: [BRINGUP.md](BRINGUP.md)** (схема підключення, діагностика кроками, таблиця діагнозів).

- `common/` - чистий C++ (framer MTData2, datagram, id_table, heartbeat зі статусом slave, генератор тестового кадру), спільна бібліотека обох проєктів
- `esp32c3-slave/` - slave, PlatformIO, Arduino-ESP32 3.x (pioarduino), ESP32-C3 Mini/SuperMini
- `esp-01/` - старий slave на ESP8266, архів, не використовується
- `esp32s3/` - master, PlatformIO, Arduino-ESP32 3.x (pioarduino); вихід по USB (за замовчуванням) або W5500/UDP; тут же юніт-тести
- `esp32c3/` - еталонні ESP-NOW експерименти, не чіпати

## ПОПЕРЕДЖЕННЯ: UART модуля

На UART модуля сидять OTA-бутлоадер і командний парсер. Випадковий байт може зіпсувати прошивку модуля.

- C3 тільки слухає: `Serial1.begin(..., RX_PIN, -1)`, TX не призначений. Не з'єднувати жодні TX-піни C3 з RX модуля.
- Підключення: TTL-лінія з даними конвертера -> RX C3 (GPIO20, `RX_PIN` у `esp32c3-slave/src/config.h`), і спільна земля. Рівні 3.3 В.
- Діагностика (`DEBUG_LOG 1`) іде в USB-CDC (`Serial`), окремий порт від UART модуля, тому безпечна.
- Живлення C3: USB-C. Світлодіод: `LED_PIN` у `config.h` (SuperMini: GPIO8).

## Збірка

```bash
cd esp32c3-slave && pio run                  # бойова збірка
cd esp32c3-slave && pio run -e esp32c3_debug  # статистика в USB-CDC
cd esp32c3-slave && pio run -e esp32c3_beacon # РАДІО-САМОТЕСТ: синтетичні кадри, UART ігнорується
cd esp32s3  && pio run
cd esp32s3  && pio test -e native     # юніт-тести на ПК
```

## Перший запуск

1. Прошити master (за замовчуванням `OUTPUT_USB 1`: дані йдуть по тому самому USB, Ethernet не потрібен). На порту побачити `# master up: wifi mac=...` (порт містить і бінарні датаграми, і текстові рядки з `# `).
2. Прошити slave (усі однаковою прошивкою). За замовчуванням slave шле на broadcast. Для unicast: вписати MAC master у `MASTER_MAC` в `esp32c3-slave/src/config.h`.
3. Канал `WIFI_CHANNEL` (за замовчуванням 1) однаковий у `esp32c3-slave/src/config.h` і `esp32s3/src/config.h`.
4. На ноутбуці: `uv run imu net --serial /dev/ttyACM0 --no-record` або GUI, транспорт "Wi-Fi: майстер ESP (USB)". (Старий режим `OUTPUT_USB 0`: IP `192.168.50.1/24`, `uv run imu net --seconds 10 --no-record`.)

Скидання таблиці ID на master: утримати BOOT (GPIO0) 5 с при старті, або `reset_ids` у USB-консолі.

## Припущення (бо відповідей на п. 9 брифу не було)

- W5500 піни за вікі Waveshare ESP32-S3-ETH: SCK 13, MISO 12, MOSI 11, CS 14, INT 10, RST 9. LED - WS2812 на GPIO21.
- Wi-Fi канал 1, slave шле broadcast, поки не задано `MASTER_MAC`.
- PlatformIO. Master: `esp32-s3-devkitc-1`, flash 8 МБ.
- Master `192.168.50.2/24`, ноутбук `192.168.50.1`, UDP 5005; broadcast-режим через `UDP_USE_BROADCAST`.
- Схема C3 <-> модуль: точку TTL-сигналу вибирає інженер, прошивка її не визначає. Для конвертера CP2102/SP3232 див. BRINGUP.md, п. 1.
