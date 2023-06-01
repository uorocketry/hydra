MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 1024K
  CAN : ORIGIN = 0x20000000, LENGTH = 64K
  RAM : ORIGIN = 0x20010000, LENGTH = 192K 
}
SECTIONS
{
  .can (NOLOAD) : 
  {
    *(.can .can.*);
  } > CAN
}  
/* _stack_start is optional and we can define this later */