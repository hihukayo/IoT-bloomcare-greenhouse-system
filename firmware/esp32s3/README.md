# BloomcareGateway

ESP32-S3 温室网关（通信与交互端）。通过 UART1 与 STM32F103 采集节点通信，
接收节点上报的温湿度，并可以向节点下发控制命令、发起查询。
OLED 显示屏与手机 App 的接入都预留在 Tasks 层。

## 目录结构

```
BloomcareGateway/
├── components/
│   ├── bsp/                   板级层：只搬字节、只碰寄存器
│   │   ├── inc/  bsp_uart.h  app_debug.h
│   │   └── src/  bsp_uart.c
│   ├── protocol/              协议层：纯软件，无硬件依赖
│   │   ├── inc/  link_protocol.h
│   │   └── src/  link_protocol.c
│   └── device/                设备层：网关背后的对端设备
│       ├── inc/  dev_stm32.h
│       └── src/  dev_stm32.c
├── main/
│   ├── main.c                 初始化各层，然后轮询
│   └── tasks/                 应用层：业务逻辑
│       ├── inc/  task_comm.h  task_sensor.h  task_gateway.h
│       └── src/  task_comm.c  task_sensor.c  task_gateway.c
└── CMakeLists.txt
```

每一层内部都按 `inc/`（头文件）和 `src/`（源文件）分开存放。

## 分层规则

依赖方向只能向下，不能反向或跨层回调：

```
                    Tasks
                   /     \
                  v       v
                Device   Protocol
                  |  \      |
                  v   \     |
                 BSP   \    |
                  |     \   |
                  v      v  v
              ESP-IDF 驱动 / FreeRTOS
```

- **BSP**：UART1 的驱动封装（115200 8N1），以及 `app_debug.h` 这个日志开关。
- **Protocol**：链路协议本体（组帧、CRC16/MODBUS、解析、发送帧构造），
  与 STM32 侧 `Components/comp_link.c` 是同一套帧格式。完全不引用 ESP-IDF 驱动。
- **Device**：`dev_stm32` 代表网关背后的那颗 STM32 节点，
  负责收发字节、解析帧、以及发送侧「发一帧等一个应答」的停等状态机。
- **Tasks**：`task_comm` 服务链路并统计，`task_sensor` 保存节点上报的值与在线状态，
  `task_gateway` 是网关对外的动作（改上报周期、查询数值）。

## 数据流

```
上行上报   USART1 -> bsp_uart -> dev_stm32 解析 -> 事件回调 -> task_sensor 缓存
下行控制   task_gateway -> dev_stm32 停等发帧 -> bsp_uart -> USART1
           ACK/NAK 回来后 dev_stm32 才返回结果
网关查询   task_gateway -> QUERY 帧 -> 节点回 REPORT（回显同一个 SEQ）
           -> dev_stm32 按 SEQ 配对 -> 结果返回给调用者
```

## 与 STM32 节点的接线

| ESP32-S3       | STM32F103  | 说明            |
|----------------|------------|-----------------|
| GPIO17 (U1TXD) | PA3 (USART2_RX) | 网关发、节点收 |
| GPIO18 (U1RXD) | PA2 (USART2_TX) | 节点发、网关收 |
| GND            | GND        | 必须共地        |

两块板子可以各自用自己的 Type-C 供电，但 GND 一定要连在一起，否则串口收到的是乱码。

## 协议要点

帧格式与命令码和节点侧完全一致，见
`../BloomcareGreenhouse/README.md` 的「协议要点」一节。网关这一侧要额外注意：

- 网关发出的帧，`ADDR` 填目标节点地址（当前 `LINK_ADDR_STM32 = 0x01`）。
  地址字段留着是为了将来一个网关挂多个同型号节点。
- CONTROL 与 QUERY 都是**停等**：一次只允许一个事务在飞，
  `LINK_ACK_TIMEOUT_MS`（200 ms）没等到应答就重发，最多重发 `LINK_ACK_RETRY`（3）次。
  同一个事务的重发复用同一个 SEQ，节点据此识别出「这是重发，不要重复执行」。
- 节点周期性上报的 REPORT 与心跳不进停等，它们通过事件回调交给 Tasks 层；
  只有 QUERY 的应答走 `dev_stm32_send_query()` 的返回值。

## 主循环

ESP-IDF 默认已启动 FreeRTOS，但本工程目前只有 `app_main` 一个任务，
各层都是「初始化一次 + 每轮轮询」的写法，和 STM32 侧结构保持一致：

```c
Task_Comm_Init();                       /* UART plus frame parser */
Task_Sensor_Init();                     /* subscribe to the values */
Task_Gateway_Init();                    /* outward actions */
while (1)
{
    now = App_Millis();
    Task_Comm_Poll(now);                /* read the link, log the frames */
    Task_Sensor_Poll(now);              /* online state of the node */
    Task_Gateway_Poll(now);             /* pending CONTROL and QUERY work */
    vTaskDelay(pdMS_TO_TICKS(APP_LOOP_DELAY_MS));
}
```

`dev_stm32_poll()` 会在 UART 上阻塞一小段时间（`DEV_STM32_READ_TIMEOUT`，50 ms），
这是目前唯一会拖慢循环的地方。等 OLED、WiFi 这类需要及时响应的外设接进来之后，
应该把 `Task_Comm` 单独放进一个 FreeRTOS 任务，其余层通过队列或事件组拿数据。

## 调试开关

`components/bsp/inc/app_debug.h` 里的 `APP_DEBUG` 控制全部日志：

- 定义时：`LOGI/LOGW/LOGE` 走 `ESP_LOGx`。
- 注释掉时：三个宏展开为空，既不占 flash 也不花时间。

它和 STM32 侧的 `LINK_DEBUG` 是同一个角色。注意 `#ifdef` 只判断宏是否定义，
关闭时要把整行注释掉，不要写成 `#define APP_DEBUG 0`。

链路统计每 10 秒打印一次（`task_comm.c` 的 `TASK_COMM_DUMP_MS`），
包含收帧计数、发帧计数、重发次数与超时次数。

`task_gateway.c` 里的 `TASK_GATEWAY_SELFTEST` 改成 `1`，会每 30 秒发一次 QUERY，
用来自测发送方向（不依赖手机 App）。

## 编译与烧录

```bash
idf.py build
idf.py -p COM10 flash monitor      # COM 号换成自己的
```

退出监视用 `Ctrl+]`。监视器和串口调试助手不能同时占用同一个 COM 口，
两边抢端口会报 `PermissionError(13)`。

本工程不依赖 `managed_components`，`idf.py build` 不需要访问外网。
如果给 CMake 加了需要下载的组件，可以用国内镜像：

```powershell
$env:IDF_GITHUB_ASSETS = "dl.espressif.com/github_assets"
```
