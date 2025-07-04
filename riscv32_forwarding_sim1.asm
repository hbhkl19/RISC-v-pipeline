# Test the RISC-V processor in simulation
# 已经能正确执行：addi, beq, jal
# 待验证：有条件与无条件分支指令后误读的指令是否能够正确清空，没有delay slot
# 不考虑分支指令与前面指令之间的数据依赖，所以添加了必要的nop指令

main:	addi x5, x0, 1
		addi x6, x0, 1
		addi x7, x0, 0			#x7 = 0
		addi x8, x0, 0     #0xc
		addi x0, x0, 0
		addi x0, x0, 0
		beq  x5, x6, br1   #0x18  #jump
		addi x8, x8, 1			
		addi x9, x9, 1		#0x20
		jal  x0, end      #0x24

br1:	addi x7, x7, 1		#0x28	#x7 = 1
		jal  x0, end		#0x2c
		addi x8, x8, 1		#0x30	
		addi x9, x9, 1		#0x34

end:	addi x7, x7, 1		#0x38