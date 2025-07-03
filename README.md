前递：
    输入：ID/EX的rs1,rs2
          EX/MEM的rd，RegWrite(rd!=0)
          MEM/WBd的rd，RegWrite(rd!=0)
    输出：forwardA,forwardB
    新添加：两个ALU多选器，(aluB需要再加一个与立即数多选的多选器。之前已经加过)一个前递单元



load-use、停顿
    输入：IF.ID的rs1，rs2
          ID/EX的memread，rd(rd!=0)
    输出：PC，IF/ID寄存器的写入
          为ID/EX控制信号添加多选器
    新添加：一个控制信号多选器，一个冒险检测单元
    修改：NPC根据stall去改
            IF/ID根据stall修改：保持不变
            ID.EX根据stall修改：清0