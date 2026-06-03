# uwb_driver - UWB 数据采集包

UWB AOA 模块数据采集、解析与滤波。

## 功能

- 从 ALX-AOA-FIT UWB 模块读取 AOA 数据
- 解析 37 字节帧协议（命令 0x2001）
- 卡尔曼滤波平滑距离和角度数据
- 发布标准化 UwbAoa 消息

## 节点

### uwb_aoa_driver_node

UWB 数据采集主节点。

**发布话题：**
- `uwb_aoa/data`（`UavDeltaMsgs/UwbAoa`）- 滤波后的 UWB 数据

**参数：**
- `serial_port`（string）- 串口路径，默认 `/dev/ttyUSB0`
- `baud_rate`（int）- 波特率，默认 `115200`
- `kalman_q`（double）- 过程噪声，默认 `0.1`
- `kalman_r`（double）- 测量噪声，默认 `0.1`

## 使用

```bash
# 单独启动
ros2 launch uwb_driver uwb_aoa_driver.launch.py

# 自定义参数
ros2 launch uwb_driver uwb_aoa_driver.launch.py serial_port:=/dev/ttyUSB1
```

## 配置

配置文件：`config/uwb_aoa_driver.yaml`

## UWB 协议

帧格式（37 字节）：
```
帧头(2B) + 长度(2B) + 命令(2B) + 数据(28B) + 校验(1B) + 帧尾(2B)
```

数据字段偏移：
- [6:10] Distance - uint32, cm
- [10:12] Azimuth - int16, 度（±90°）
- [12:14] Elevation - int16, 度（±30°）
