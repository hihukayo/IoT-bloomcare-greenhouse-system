# BloomcareGreenhouse

温室环境监测与执行系统。两块板子分工协作：

| 目录 | 目标板 | 角色 |
|------|--------|------|
| `firmware/stm32` | STM32F103ZET6（Keil MDK5） | 采集与执行端：读 DHT22 温湿度，驱动执行器 |
| `firmware/esp32s3` | ESP32-S3（ESP-IDF 6.1） | 通信与交互端：与 STM32 通信，将来接 OLED 与手机 App |

两块板子之间用 UART（115200 8N1）跑自定义二进制协议。

## 为什么放在一个仓库里

协议是两端共同遵守的契约，而它现在各存一份：

- `firmware/stm32/Components/Inc/comp_link.h`
- `firmware/esp32s3/components/protocol/inc/link_protocol.h`

任何一处改了帧格式、命令码或数据项 ID，另一端必须同步改，否则链路直接不通。
放在同一个仓库里，一次提交就能同时覆盖两端，也不会出现「一端升级、另一端忘记跟」
却查不出是哪天引入的问题。

## 目录结构

```
BloomcareGreenhouse/
├── firmware/
│   ├── stm32/          STM32F103 采集与执行端，Keil MDK5 工程
│   └── esp32s3/        ESP32-S3 网关，ESP-IDF 工程
└── README.md
```

各端的细节都在自己的 README 里：

- `firmware/stm32/README.md`：分层规则、协议要点、与 CubeMX 共存的注意事项
- `firmware/esp32s3/README.md`：主循环、停等与重发、烧录与监视

## 硬件连接

| ESP32-S3 | STM32F103 | 说明 |
|----------|-----------|------|
| GPIO17 (U1TXD) | PA3 (USART2_RX) | 网关发、节点收 |
| GPIO18 (U1RXD) | PA2 (USART2_TX) | 节点发、网关收 |
| GND | GND | 必须共地，否则收到乱码 |

两块板子可以各自用自己的 Type-C 供电。DHT22 的 DATA 接 STM32 的 PG11
（CubeMX 标签 `DHT22`），VCC / GND 别接反。

## 构建与烧录

STM32（也可以直接用 Keil 打开工程下载）：

```
UV4 -r firmware\stm32\MDK-ARM\BloomcareGreenhouse.uvprojx -o build_log.txt
```

ESP32-S3：

```
cd firmware\esp32s3
idf.py build
idf.py -p COM10 flash monitor
```

退出监视用 `Ctrl+]`。串口监视器和串口调试助手不能同时占用同一个 COM 口。

## 调试开关

默认都打印，注释掉宏定义就整段去掉，不占 flash 也不花时间。

| 端 | 开关 | 位置 |
|----|------|------|
| STM32 | `LINK_DEBUG` | `firmware/stm32/Components/Inc/comp_link.h` |
| STM32 | `BSP_UART_DEBUG` | `firmware/stm32/BSP/Inc/bsp_uart.h` |
| ESP32-S3 | `APP_DEBUG` | `firmware/esp32s3/components/bsp/inc/app_debug.h` |

## 维护约定

- 改协议：两端的两份协议头必须同一次提交改完，并各自编一遍。
- 引脚：驱动里只写 CubeMX 标签（如 `SEN_DHT22_GPIO_PIN = DHT22_Pin`），
  挪引脚只改 CubeMX。
- 只改自己的业务目录（STM32 的 `BSP/` `Components/` `Device/` `Tasks/`，
  ESP32 的 `components/` `main/`）；`Core/`、`Drivers/`、ESP-IDF 的组件
  只在 USER CODE 区或配置里动。
- 提交信息写清楚动了哪一端、为什么。
