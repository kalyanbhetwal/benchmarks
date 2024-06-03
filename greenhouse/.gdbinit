target remote :3333
load
monitor arm semihosting enable
break main
break main.rs:743
break checkpoint/mod.rs:404