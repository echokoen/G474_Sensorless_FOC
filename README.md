## 项目简介

本项目是一个基于 STM32G474 的无感 PMSM FOC 电机控制工程。

搭建了PMSM无感FOC的控制软件架构，完成了电流采样、坐标变换、电流环，SVPWM无感闭环的完整控制链路。

### 在应用层

参考了ST的中、高频任务状态机

即IDLE：空闲状态  OFFSET_CALIB:自举电容充电  ALIGN：转子对齐  OPEN_LOOP：开环启动  RUN：观察器运行  其中OPEN_LOOP：内部分出SWITCH_OVER子状态，用于观察器的切换判断


### FREERTOS
仅仅搭建简单的应用框架，放入了按键、中频任务处理，后续工程落地可在此框架内进行二次开发。

### simulink
主要对观察器、FOC链路、电流采样进行建模

观察器模型：建立了电机稳态生成器模块 + flux磁链观察器模型 + PLL锁相环模型，在各种转速下的响应情况，该模型主要是对算法的仿真优化。

FOC链路：按照基本的电流环FOC链路，模块内直接放的代码，该模型主要是对代码进行的黑盒测试。

电流采样建模：该模型仅对电流采样进行逻辑梳理，仿真结果不具参考价值。

### 对于前馈补偿和死区补偿：

死区补偿主要是补在了SVPWM生成的三相电压上，数值上由死区时间Td、开关频率fsw、相电压u计算得来，并且做了按电流方向补偿和过零平滑处理

前馈补偿主要是做了反电动势补偿和交叉耦合项补偿，补在了电流环输出的Vq vd上，

反电动势补偿按照we * Φf *50% 进行补偿

交叉耦合项补偿按照V_couple = we * L * I %50% 补偿，


### 该README仅做项目简述

### 详细项目细节、模型仿真结果、参考框架.md和model文件夹

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
|    通信调试    | USART1/(Baudrate：921600)                  |
## 接线说明

<img width="2271" height="1261" alt="image" src="https://github.com/user-attachments/assets/8b6dba20-83d8-48f3-abb6-0c820c380406" />

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

## 运行以及波形查看
#### 三相电流波形图

<img width="785" height="465" alt="image" src="https://github.com/user-attachments/assets/c732f651-f8fa-4ad5-ac62-d13da71eedc9" />


#### iq_ref  iq_fdb稳态波形图

<img width="784" height="224" alt="image" src="https://github.com/user-attachments/assets/7783877e-98c8-447e-8ad1-e2a53f39f2a7" />


#### id_ref  id_fdb稳态波形图

<img width="782" height="222" alt="image" src="https://github.com/user-attachments/assets/c47b42b8-bc90-4f5b-b080-dddbbeac42c2" />


#### iq_ref  iq_fdb阶跃波形图

<img width="792" height="228" alt="image" src="https://github.com/user-attachments/assets/2c225662-fea1-49f1-9450-9cd690a3fe90" />


#### 启动阶段波形图

<img width="310" height="439" alt="image" src="https://github.com/user-attachments/assets/bb0441c3-3a5f-4713-9364-0dcb49cb8760" />
<img width="789" height="302" alt="image" src="https://github.com/user-attachments/assets/984570a6-4438-49d1-afc9-13eaf7973f3c" />
<img width="804" height="444" alt="image" src="https://github.com/user-attachments/assets/8cabdd72-c4e2-4876-a069-dde5080cedb8" />

#### Observer / PLL 跟踪图

<img width="795" height="613" alt="image" src="https://github.com/user-attachments/assets/dd6c0980-5420-4893-8216-317f6dbb2087" />


penloop_speed 和 obs_speed接近，差值不到1rad/s

observer速度估算正确

PLL速度环已锁住

pll_err = -0.063 说明PLL当前误差已经很低了

pll_integral = 382.93 积分限幅是1800，远没有达到积分限幅

angle_err = 19.011,说明固定角偏差还偏大

#### 速度响应波形图

<img width="794" height="796" alt="image" src="https://github.com/user-attachments/assets/3d3262a4-ed7b-4b9f-bb6b-9424f652303f" />


方向对
速度能追目标
没有明显发散
目标附近没有大超调
速度反馈有明显纹波

#### 高频 ISR 执行时间图  

<img width="791" height="575" alt="image" src="https://github.com/user-attachments/assets/b2e938be-eefb-45fb-829a-fbe845fb6e93" />


16khz高频周期是62.5μs
max_us 没超过 50us

周期余量62.5 - 40.35 ≈ 22.15 us

## 仿真验证
### 电流环仿真
#### iq电流环离散跟踪

<img width="813" height="685" alt="image" src="https://github.com/user-attachments/assets/a5b66cf0-7f77-436f-a3ae-f5a14a188b74" />


q轴电流环方向正确，PI 离散实现能稳定跟踪转矩电流给定。

#### id电流环离散跟踪

<img width="803" height="661" alt="image" src="https://github.com/user-attachments/assets/1959259a-fef2-4e78-860e-6ab6eebf39d7" />



d轴电流环稳定，但带宽比q轴低。


#### 2000转速下，电流环轨迹

<img width="809" height="657" alt="image" src="https://github.com/user-attachments/assets/daa0f3dc-1a1a-4b15-8796-9196de434020" />


在 2000rpm 下，Iq 最终仍能跟到 1A，但启动瞬间 Iq 和 Id 都有明显扰动。

在存在反电动势时，电流环仍能恢复到目标值，但动态过程会受到速度项耦合影响。

#### 3000转速下，电流环轨迹＆电压限幅边界

<img width="806" height="614" alt="image" src="https://github.com/user-attachments/assets/0ab6ddaa-ba24-4a6f-9a1d-9647b7908d57" />

3000rpm + 5A iq 时，|vdq| 接近并触及电压限幅，limit_flag 有动作。

即：

高速大电流工况下，dq 电压余量成为限制因素；电流环稳定性不仅取决于 PI，也受母线电压和反电动势约束。

### observer观察器仿真
#### 低速正转速度估算
<img width="822" height="664" alt="image" src="https://github.com/user-attachments/assets/aa782726-46d5-496b-81fa-6eaf1265c67d" />

反映 speed_raw 和 speed_obs 的动态响应：observer 能逐步建立速度估计，但低速下收敛过程比较慢，并且有明显动态摆动。

500rpm 低速下 Observer/PLL 可以形成速度估计，但低速收敛质量一般，说明低速是无感 observer 的困难区。
#### 低速反转方向
<img width="829" height="667" alt="image" src="https://github.com/user-attachments/assets/8fdcf2a3-9bcb-4e46-b91e-12a4a7b57c70" />

Observer/PLL 的速度方向符号链路是双向的，反转工况下不会直接符号错误。
#### 磁链观测本体

<img width="849" height="667" alt="image" src="https://github.com/user-attachments/assets/f07b757a-57f4-468c-a00e-f6e8e1d6ed6a" />

flux_alpha 和 flux_beta 呈正交周期波形，flux_mag 基本围绕目标磁链附近波动。

Flux Observer 能形成旋转磁链矢量，磁链幅值保持在合理范围，这是 PLL 估角的输入基础。
#### PLL 内部状态
<img width="887" height="665" alt="image" src="https://github.com/user-attachments/assets/924a1718-ae9d-4f33-adf9-c9b9cf141c83" />
展示 pll_err 和 pll_integrator 的收敛过程，能看出 PLL 是通过误差积分逐步建立速度估计的
PLL 积分项没有瞬间打满，而是逐步建立速度输出；该图适合说明 PLL 动态锁相过程。












