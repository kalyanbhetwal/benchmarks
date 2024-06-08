target remote :3333
load
monitor arm semihosting enable
break main
break checkpoint/mod.rs:404