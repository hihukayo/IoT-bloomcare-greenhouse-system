# BloomcareGreenhouse

STM32F103 温室环境监测节点（采集与执行端）。DHT11 采集温湿度，通过 USART2 上的
自定义协议帧上报给 ESP32-S3 网关，并接收网关下发的控制命令。

## 目录结构

```
BloomcareGreenhouse/
├── Core/                       CubeMX 生成，不放业务代码
│   ├── Inc/
│   └── Src/                    main.c 只负责初始化和主循环
├── Drivers/                    ST HAL 库，不修改
├── BSP/                        板级层：只搬字节、只碰寄存器
│   ├── Inc/  bsp_uart.h  bsp_delay.h  bsp_gpio.h
│   └── Src/  bsp_uart.c  bsp_delay.c  bsp_gpio.c
├── Components/                 协议层：纯软件，无硬件依赖，可跨平台
│   ├── Inc/  comp_link.h
│   └── Src/  comp_link.c
├── Device/                     设备层：单个芯片怎么读、怎么写
│   ├── Inc/  dev_manager.h
│   ├── Src/  dev_manager.c
│   ├── Sensors/
│   │   ├── Inc/  sen_dht11.h
│   │   └── Src/  sen_dht11.c
│   └── Actuators/              执行器驱动放这里（当前为空）
│       ├── Inc/
│       └── Src/
└── Tasks/                      应用层：业务逻辑
    ├── Inc/  task_comm.h  task_sensor.h  task_control.h
    └── Src/  task_comm.c  task_sensor.c  task_control.c
```

每一层内部都按 `Inc/`（头文件）和 `Src/`（源文件）分开存放。

## 分层规则

依赖方向只能向下，不能反向或跨层回调：

```
                    Tasks
                   /     \
                  v       v
        Components        Device
                  \       /
                   v     v
                      BSP
                       |
                       v
                      HAL
```

- **BSP**：USART1 调试口、USART2 网关链路的 DMA 环形缓冲与 IDLE 中断、TIM2 微秒延时、
  GPIO 端口时钟开关。只知道字节和寄存器，不知道帧格式。
- **Components**：链路协议本体（组帧、CRC16/MODBUS、解析、ACK/NAK、重传、QUERY 应答、
  重发去重）。完全不引用 HAL，数据靠 `Link_Transport_t` 四个函数指针传入，
  换 MCU 只需重新提供这四个函数。
- **Device**：`sen_dht11` 只管把 DHT11 读出来；`dev_manager` 维护传感器/执行器两张表，
  上层遍历表即可，不需要知道具体是哪颗芯片。
- **Tasks**：`task_comm` 把 BSP 传输绑给协议，`task_sensor` 定时采集并上报（同时应答 QUERY），
  `task_control` 执行网关下发的命令。

## 数据流

```
上行采集   task_sensor -> Dev_Manager_Collect -> sen_dht11 -> GPIO
           task_sensor -> Link_SendReport -> task_comm -> BSP_Uart2_Tx -> USART2
下行命令   USART2 -> BSP_Uart2_RxTake -> task_comm -> Link_Poll 解析
           -> task_control -> Dev_Manager_FindActuator -> actuator->set()
网关查询   task_comm -> Link_SetQueryHandler 的回调 -> task_sensor 缓存
           -> Link_SendItems(REPORT) -> USART2
```

## 协议要点

帧格式（小端）：`AA 55 | VER | ADDR | LEN | CMD | SEQ | PAYLOAD | CRC_L CRC_H`，
CRC16/MODBUS 覆盖 `VER` 到 `PAYLOAD` 末尾，低字节在前。

| CMD  | 值   | 方向        | 应答                     |
|------|------|-------------|--------------------------|
| REPORT    | 0x01 | 节点 -> 网关 | 不需要应答（周期上报、QUERY 的应答） |
| CONTROL   | 0x02 | 网关 -> 节点 | 必须回 ACK 或 NAK，回显 SEQ |
| HEARTBEAT | 0x03 | 双向        | 不需要应答                |
| QUERY     | 0x04 | 网关 -> 节点 | 用 REPORT 回显 SEQ，查不到回 NAK |
| ACK       | 0x80 | 应答        | 载荷为被确认的 CMD        |
| NAK       | 0x81 | 应答        | 载荷为 CMD + 原因码       |

数据项（每项 5 字节：ID 1B + int32 4B，数值乘 100 传输）：

| ID   | 值   | 含义                          |
|------|------|-------------------------------|
| TEMP | 0x01 | 空气温度（x100，单位 C）       |
| HUMI | 0x02 | 空气湿度（x100，单位 %RH）     |
| REPORT_MS | 0xF1 | CONTROL：上报周期（毫秒） |
| ERRCODE   | 0xF2 | 传感器故障码              |
| 执行器 | 0x80~0xEF | 由 `ItemToChannel()` 映射到通道 |

几条约定：

- CONTROL 在一个帧里可以带多个数据项，逐个执行；NAK 只带原因码，所以返回**第一个**失败原因。
- 重复的 CONTROL（同一 SEQ、3 秒窗口内）只重发上次的结果，不会重复执行命令。
- QUERY 的载荷首字节为要查询的 ID，`0` 表示要全部；应答走缓存，不会为了应答而二次读传感器。

## 新增一个传感器 / 执行器

1. 在 `Device/Sensors/Src/` 或 `Device/Actuators/Src/` 写驱动，实现 `Dev_Sensor_t`
   或 `Dev_Actuator_t` 里的函数，并导出一个实例（参考 `sen_dht11.c` 末尾的 `g_sen_dht11`）。
2. 头文件放进对应的 `Inc/` 目录。
3. 在 `Device/Src/dev_manager.c` 的 `s_sensors[]` / `s_actuators[]` 里加一行注册。
   槽位个数由 `sizeof` 推导，不需要手工维护计数器；暂时没有的驱动可以留 `NULL` 占位。
4. 新通道号加在 `Device/Inc/dev_manager.h` 的 `DEV_CH_xxx`，
   再在 `Tasks/Src/task_sensor.c` 的 `ChannelToItem()` 里映射到协议 ID。

驱动的引脚只写 CubeMX 的标签，例如 `SEN_DHT11_GPIO_PIN = DHT11_Pin`，
端口时钟用 `SEN_DHT11_GPIO_CLK_ENABLE()`（内部走 `BSP_GPIO_ClkEnable()`）。
换引脚只改 CubeMX，不用改驱动里的两处宏。

## 编译

- Keil MDK5 打开 `MDK-ARM/BloomcareGreenhouse.uvprojx`；
  命令行编译：`UV4 -b MDK-ARM\BloomcareGreenhouse.uvprojx -o build_log.txt`。
- 项目使用 MicroLIB（`printf` 重定向到 USART1）。

## 串口

| 接口   | 用途          | 参数          |
|--------|---------------|---------------|
| USART1 | 调试打印      | 115200 8N1    |
| USART2 | ESP32-S3 网关 | 115200 8N1    |

`Tasks/Src/task_comm.c` 每 10 秒打印一次收发计数，便于观察链路状态。

## 调试开关

`Components/Inc/comp_link.h` 里的 `LINK_DEBUG`、`BSP/Inc/bsp_uart.h` 里的 `BSP_UART_DEBUG`
注释掉即可去掉对应的计数与打印。注意 `#ifdef` 只判断宏是否定义，
关闭时要把整行注释掉，不要写成 `#define LINK_DEBUG 0`。

## 与 CubeMX 共存

CubeMX 只会重写 `Core/Inc`、`Core/Src` 里 `/* USER CODE BEGIN xxx */` 与
`/* USER CODE END xxx */` 之外的内容，本项目的业务代码全部放在 `BSP/`、`Components/`、
`Device/`、`Tasks/` 四个目录里，CubeMX 不会碰到它们。

重新生成代码后需要人工确认两件事：

1. 引脚标签（如 `DHT11_Pin` / `DHT11_GPIO_Port`）是否还在，改名会导致 `sen_dht11.h` 编译报错。
2. `MDK-ARM/BloomcareGreenhouse.uvprojx` 里的分组是否还包含 `BSP/`、`Components/`、
   `Device/`、`Tasks/` 下的 `.c` 文件，CubeMX 重新生成时可能重建工程文件。

中断钩子是唯一必须留在 `Core/Src/stm32f1xx_it.c` 里的代码，见该文件的 `USART2_IRQn` 分支，
它只负责清标志并通知 BSP，解析和应答都在主循环里做。
