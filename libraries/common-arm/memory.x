MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 128K 
}
SECTIONS
{
  .can : 
  {
    *(.can)
  } > RAM AT > RAM
}  
/* _stack_start is optional and we can define this later */