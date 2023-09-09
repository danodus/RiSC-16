    lui r1, 0x3FF       # movi r1,0xFFFF
    addi r1, r1, 0x3F
    addi r7, r0, 42     # initial display value
main_loop:
    sw r7, r1, 0        # write to display
    add r2, r0, r0      # delay
delay_loop:
    addi r2, r2, 1
    beq r2, r1, delay_done
    beq r0, r0, delay_loop
delay_done:
    addi r7, r7, 1      # increment display value
    beq r0, r0, main_loop