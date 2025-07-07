# 五级流水线 CPU 实现

本项目基于 Verilog 实现了一个简洁的五级流水线 CPU，支持基本的 RISC-V 指令子集，采用经典的 IF → ID → EX → MEM → WB 流水线结构。控制转移类指令在 ID 阶段进行判断和跳转决策，并处理数据冒险、控制冒险等关键问题。

---

## 项目特点

- 实现完整的五级流水线结构：
  - IF（Instruction Fetch）
  - ID（Instruction Decode & Register Read）
  - EX（Execute / ALU）
  - MEM（Memory Access）
  - WB（Write Back）

- 支持的功能模块：
  - 控制单元（Control Unit）
  - 寄存器堆（Register File）
  - ALU 运算模块
  - 数据存储器（Data Memory）
  - 程序计数器与跳转逻辑
****
- 冒险处理：
  - 数据冒险支持转发路径：MEM→EX，WB→EX，WB→MEM 等
  - Load-Use 冒险处理：插入气泡或暂停流水线
  - 控制冒险处理：分支在 ID 阶段判断，分支失败时清除后续指令

- 指令支持（RISC-V RV32I 子集）：
  - 算术/逻辑指令：add, sub, and, or, xor, sll, srl, sra, slt, etc.
  - 立即数操作：addi, andi, ori, xori, slti, etc.
  - 访存指令：lw, sw
  - 跳转与分支：beq, bne, jal, jalr
  - 特殊指令：lui, auipc

- 可仿真可视化：
  - 支持 Icarus Verilog 仿真
  - 支持 VCD 波形输出，使用 GTKWave 观察流水线执行过程

---

## 使用环境

- IDE：vscode
- 仿真工具：Icarus Verilog
- 波形查看工具：GTKWave
