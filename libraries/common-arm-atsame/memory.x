MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  FLASH : ORIGIN = 0x00000000, LENGTH = 256K
  CAN : ORIGIN = 0x20000000, LENGTH = 32K
  CAN_COMMAND : ORIGIN = 0x20008000, LENGTH = 32K
  RAM : ORIGIN = 0x20010000, LENGTH = 64K 
}
SECTIONS
{
  .can (NOLOAD) : 
  {
    *(.can .can.*);
  } > CAN
  .can_command (NOLOAD) : 
  {
    *(.can_command .can_command.*);
  } > CAN_COMMAND
}  
/* _stack_start is optional and we can define this later */