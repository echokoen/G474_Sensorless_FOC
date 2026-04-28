# PMSM 电流环离散 Simulink 模型说明

## 模型定位

`model/simulink/pmsm_current_loop_discrete.slx` 是当前 STM32G474 Sensorless PMSM FOC 工程的电流环离散验证模型。

它的目标是验证当前 C 代码里的电流环核心链路，而不是搭一个完整无感控制系统。模型只包含：

- Id/Iq 电流参考；
- 离散电流 PI；
- dq 电压矢量限幅；
- InvPark；
- 可选 SVPWM / 逆变器平均等效；
- PMSM dq 电流 plant；
- Clarke / Park 反馈；
- 理想电角度。

模型暂时不包含：

- Observer；
- PLL；
- Handover；
- Startup；
- 速度环；
- 真实三相开关管；
- Simscape 功率桥；
- ADC 采样窗口和三低边重构细节。

## 文件位置

模型文件：

- `model/simulink/pmsm_current_loop_discrete.slx`

参数脚本：

- `model/simulink/pmsm_current_loop_params.m`

模型增量更新脚本：

- `model/simulink/update_pmsm_current_loop_discrete_v2.m`

测试脚本：

- `model/simulink/run_current_loop_tests.m`

测试结果：

- `model/simulink/current_loop_test_results.mat`

## 参数来源

模型参数集中放在 `model/simulink/pmsm_current_loop_params.m`。

主要参数来自当前 C 工程：

- `Ts` 对应 `FOC_TS_SEC`；
- `Rs` 对应 `FOC_RS_OHM`；
- `Ld` / `Lq` 对应 `FOC_LD_H` / `FOC_LQ_H`；
- `flux_wb` 对应 `FOC_FLUX_WB`；
- `pole_pairs` 对应 `FOC_POLE_PAIRS`；
- `id_kp` / `id_ki` 对应 `FOC_ID_KP` / `FOC_ID_KI`；
- `iq_kp` / `iq_ki` 对应 `FOC_IQ_KP` / `FOC_IQ_KI`；
- `v_limit` 对应单轴 PI 输出限幅；
- `dq_v_limit` 对应 dq 电压矢量总限幅；
- `vd_cmd_sign` / `vq_cmd_sign` 对应 `FOC_VD_CMD_SIGN` / `FOC_VQ_CMD_SIGN`；
- `pwm_arr` / `pwm_half_arr` 对应 `FOC_PWM_ARR` / `FOC_PWM_HALF_ARR`。

`Vbus` 当前设置为 24V。当前 `foc_config.h` 里有母线电压采样比例 `FOC_VBUS_RATIO`，但没有一个固定的母线额定电压宏，所以模型先使用 24V 作为默认母线。

## 模型结构

主链路如下：

```text
id_ref / iq_ref
  -> Discrete_Current_PI_CodeLike
  -> DQ_Vector_Limit
  -> InvPark_CodeLike
  -> SVPWM bypass switch
  -> PMSM_DQ_Plant
  -> Clarke_Park_Feedback
  -> Unit Delay
  -> PI feedback
```

SVPWM 路径如下：

```text
InvPark_CodeLike
  -> SVPWM_Inverter_Equivalent
  -> SVPWM bypass switch
  -> PMSM_DQ_Plant
```

当 `use_svpwm_equivalent = 0` 时，模型绕过 SVPWM，直接把 InvPark 输出的 `valpha/vbeta` 输入 plant。这个是默认设置，适合先验证纯电流环。

当 `use_svpwm_equivalent = 1` 时，模型先经过 C-like SVPWM / 逆变器平均等效，再进入 plant。这个适合观察扇区、duty、调制限幅对电压输出的影响。

## 和 C 代码的对应关系

`Discrete_Current_PI_CodeLike` 对应：

- `Control/Model/foc_pi_controller.c`
- `Control/Loop/foc_current_loop.c`

PI 使用 `ref - meas`，先计算积分预测值，再做积分限幅，然后计算 `p + integral`，最后做输出限幅。anti-windup 条件保持和 C 代码一致。

`vd_pi = vd_cmd_sign * id_out` 和 `vq_pi = vq_cmd_sign * iq_out` 对应 `foc_current_loop.c` 里的：

```c
vdq.d = FOC_VD_CMD_SIGN * PIController_Run(...);
vdq.q = FOC_VQ_CMD_SIGN * PIController_Run(...);
```

`DQ_Vector_Limit` 对应 `foc_current_loop_limit_vector()`，限制的是 `sqrt(vd^2 + vq^2)`，不是 d/q 两轴分别限幅。

`InvPark_CodeLike` 对应 `ClarkePark_InvPark()`：

```c
alpha = d * cos(theta) - q * sin(theta);
beta  = d * sin(theta) + q * cos(theta);
```

`Clarke_Park_Feedback` 对应 `ClarkePark_Clarke()` 和 `ClarkePark_Park()`：

```c
alpha = ia;
beta  = (ia + 2 * ib) / sqrt(3);
d = alpha * cos(theta) + beta * sin(theta);
q = -alpha * sin(theta) + beta * cos(theta);
```

`SVPWM_Inverter_Equivalent` 对应 `FOC_Svpwm_ApplySvpwm()` 的扇区判断、T1/T2/T0 计算和 duty 分配。这里输出的是平均等效电压，不是实际开关波形。

## 符号方向说明

控制侧负号来自 C 代码的 `FOC_VD_CMD_SIGN` 和 `FOC_VQ_CMD_SIGN`，模型没有把它藏在别的地方。

PMSM plant 里还有 `plant_vd_effect_sign` 和 `plant_vq_effect_sign`，这是测试台 plant 侧的极性适配参数，用来模拟当前硬件实测“正 Vd/Vq 得到负 Id/Iq”的电压电流方向。它不是控制算法的一部分。

如果要切换到标准教材 dq plant，可以把：

```matlab
plant_vd_effect_sign = 1.0;
plant_vq_effect_sign = 1.0;
```

但这样模型表现会和当前工程里 `FOC_VD_CMD_SIGN = -1`、`FOC_VQ_CMD_SIGN = -1` 的硬件适配方向不同。

## 日志信号

模型输出 `current_loop_log`，列顺序如下：

- `id_ref`
- `iq_ref`
- `id`
- `iq`
- `vd_pi`
- `vq_pi`
- `vd_cmd`
- `vq_cmd`
- `limit_flag`
- `id_int`
- `iq_int`
- `valpha`
- `vbeta`
- `theta_e`
- `we`
- `duty_u`
- `duty_v`
- `duty_w`
- `sector`

这些信号同时接到了 Scope，方便快速看响应。

## 运行模型

在 MATLAB 中执行：

```matlab
addpath('D:/final_projects/G474_Sensorless_FOC/model/simulink');
run('D:/final_projects/G474_Sensorless_FOC/model/simulink/pmsm_current_loop_params.m');
open_system('D:/final_projects/G474_Sensorless_FOC/model/simulink/pmsm_current_loop_discrete.slx');
sim('pmsm_current_loop_discrete');
```

如果修改了模型结构，可以执行：

```matlab
update_pmsm_current_loop_discrete_v2;
```

## 运行测试

在 MATLAB 中执行：

```matlab
addpath('D:/final_projects/G474_Sensorless_FOC/model/simulink');
results = run_current_loop_tests;
```

当前测试包含：

- Id 阶跃测试；
- Iq 阶跃测试；
- 2000rpm 反电动势影响测试；
- 3000rpm 电压限幅测试。

测试结果保存到：

```text
model/simulink/current_loop_test_results.mat
```

## 当前限制

当前模型和真实代码仍有这些差异：

- plant 是离散 dq 平均模型，不是真实三相电机和逆变器开关模型；
- 默认绕过 SVPWM，优先验证纯电流环；
- SVPWM 等效只还原平均电压、duty 和 sector，不模拟实际 PWM 纹波；
- 没有模拟 ADC 注入采样时刻、三低边采样窗口和电流重构误差；
- 没有死区补偿和死区非线性；
- 没有前馈补偿路径，当前重点是基础电流 PI；
- 没有 Observer / PLL / Handover / Startup / 速度环；
- 没有保护状态机和故障停机动作。

## 下一步扩展建议

后续可以按小步方式继续增强：

- 增加前馈补偿开关，用于对比纯 PI 和 PI + BEMF FF；
- 增加死区电压误差模型，但先不要引入真实开关桥；
- 增加三低边采样窗口和电流重构误差模型；
- 增加参数扫描脚本，批量扫 PI 参数、Vbus、转速和负载；
- 增加和 C 日志对齐的导入脚本，把串口日志叠加到 Simulink 曲线上。
