## 项目简介

本项目是一个基于 STM32G474 的无感 PMSM FOC 电机控制工程。
主要用于验证三相逆变器驱动、低边电流采样、SVPWM、电流环控制、磁链观测器 / PLL、开环启动以及 FreeRTOS 任务框架在实际电机控制项目中的应用。
该README仅做项目简述
详细项目细节，参考框架.md

## 总览

|      参数      | 说明                                       |
| :------------: | ------------------------------------------ |
|      MCU       | STM32G474VET6                              |
|     控制板     | ATK-PD9010B                                |
|    控制对象    | 表贴式永磁同步电机电机/24V/4对极           |
|    PWM频率     | 16khz                                      |
|    电流采样    | 三低边电阻采样/扇区/动态采集点             |
|    母线电压    | 24V                                        |
|     SVPWM      | 由零序注入法升级为扇区+七段式              |
| 位置、速度估算 | 磁链观测器+PLL                             |
|     电流环     | PI控制，执行频率16khz，带宽1000hz          |
|     速度环     | PI控制，执行频率1khz，带宽约350hz          |
|    补偿方式    | 死区补偿、前馈补偿（反电动势、交叉耦合项） |
|    启动方式    | 开环 I/F 启动                              |
|    开发环境    | Keil MDK-ARM                               |
|  实时操作系统  | freertos                                   |
|    通信调试    | USART1/(Baudrate：115200)                  |
## 接线说明

![image-20260429051937476](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429051937476.png)

## 移植
电机的项目本质上是不太适合移植的
除非硬件完全一样，否则移植难度很大
移植至少要注意一下几个方面
#### 电机参数

极对数：4
相电阻 Rs：0.53 Ω
相电感 Ls：0.6 mH
磁链 Flux：0.0078 Wb

电机参数如果不一样

电流环、速度环、observer PLL、保护参数、前馈补偿参数均会不一样

#### 引脚映射

三相电流采样：
  U 相电流：PC1 -> ADC12_IN7
  V 相电流：PC2 -> ADC12_IN8
  W 相电流：PC3 -> ADC12_IN9

母线电压采样：
  VBUS：PA3 -> ADC1_IN4

三相 PWM 高边：
  U 高边：PA8  -> TIM1_CH1
  V 高边：PA9  -> TIM1_CH2
  W 高边：PA10 -> TIM1_CH3

三相 PWM 低边：
  U 低边：PB13 -> TIM1_CH1N
  V 低边：PB14 -> TIM1_CH2N
  W 低边：PB15 -> TIM1_CH3N

驱动与保护：
  M1_EN_DRIVER：PC13
  M1_BRAKE：PB12

调试：
  DAC_CH1：PA4
  DAC_CH2：PA6
  Start/Stop：PE14 -> EXTI15_10

#### 电流采集

本项目电流采集为低边三相电流采集，根据 SVPWM 扇区选择可信两相并重构第三相

并且采集点做了动态采集点

电流放大倍数为6倍，采样偏置电压为1.25V

如果电流采集采用其他方式，如：母线单电阻采样、高边电流采样等

电流采集部分需要重构

##运行以及波形查看
####三相电流波形图

![image-20260429070736791](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429070736791.png)

![image-20260429082316126](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429082316126.png)

####iq_ref  iq_fdb稳态波形图

![image-20260429194948659](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429194948659.png)

####id_ref  id_fdb稳态波形图

![image-20260429194855215](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429194855215.png)

####iq_ref  iq_fdb阶跃波形图

![image-20260429200658859](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429200658859.png)

####启动阶段波形图

![image-20260429202006499](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429202006499.png)
![image-20260429202401328](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429202401328.png)
![image-20260429202539228](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429202539228.png)

####Observer / PLL 跟踪图

![image-20260429213632469](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429213632469.png)

penloop_speed 和 obs_speed接近，差值不到1rad/s
observer速度估算正确
PLL速度环已锁住
pll_err = -0.063 说明PLL当前误差已经很低了
pll_integral = 382.93 积分限幅是1800，远没有达到积分限幅
angle_err = 19.011,说明固定角偏差还偏大

####速度响应波形图

![image-20260429231954150](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260429231954150.png)

方向对
速度能追目标
没有明显发散
目标附近没有大超调
速度反馈有明显纹波

#### 高频 ISR 执行时间图  

![image-20260430001615747](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260430001615747.png)

16khz高频周期是62.5μs
max_us 没超过 50us

周期余量62.5 - 40.35 ≈ 22.15 us

##仿真验证
###电流环仿真
####iq电流环离散跟踪

![image-20260430020457334](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260430020457334.png)

q轴电流环方向正确，PI 离散实现能稳定跟踪转矩电流给定。

####id电流环离散跟踪

![image-20260430020641543](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260430020641543.png)

d轴电流环稳定，但带宽比q轴低。


####2000转速下，电流环轨迹

![image-20260430020726269](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260430020726269.png)

在 2000rpm 下，Iq 最终仍能跟到 1A，但启动瞬间 Iq 和 Id 都有明显扰动。

在存在反电动势时，电流环仍能恢复到目标值，但动态过程会受到速度项耦合影响。

####3000转速下，电流环轨迹＆电压限幅边界

![image-20260430020904852](C:\Users\10095\AppData\Roaming\Typora\typora-user-images\image-20260430020904852.png)

3000rpm + 5A iq 时，|vdq| 接近并触及电压限幅，limit_flag 有动作。

即：

高速大电流工况下，dq 电压余量成为限制因素；电流环稳定性不仅取决于 PI，也受母线电压和反电动势约束。




