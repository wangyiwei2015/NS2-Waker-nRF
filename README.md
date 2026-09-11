# NS2唤醒器-基于ESP32
使用nRF SDK / SoftDevice / TraeCode

按下Wake按钮即可发送唤醒包

## 抓包说明

### 工具

nRF Connect或者LightBlue等App都可以

注意：iOS由于系统限制，无法获得扫描设备MAC，推荐使用Android最方便

### 找到Joy-Con设备

Joy-Con 2 的 MAC 地址：

```
78:81:8C:XX:XX:XX
```

其中`78:81:8C`是Nintendo注册的MAC前缀（据说还有更多别的），因此JC应该由此开头。

找出信号最强的符合条件的设备

### 获取厂商广播数据包

广播类似如下形式：

```
02 01 06 1B FF 53 05 …
```

需要找到并替换进代码中。

### 编译和使用

`make` 构建DEBUG版本，带日志和调试信息
`make RELEASE=1` 构建Release精简程序
烧录参考JLink的各种教程，或者直接请Agent运行make flash

---

#### Vibe-Coding说明

使用了 TRAE Agent
