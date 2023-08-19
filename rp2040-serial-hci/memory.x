MEMORY
{
  /* NOTE 1 K = 1 KiBi = 1024 bytes */
  /* To suit Raspberry Pi RP2040 SoC */
  BOOT_LOADER : ORIGIN = 0x10000000, LENGTH = 0x100
  /* Adjust this to suit the size of your specific flash chip */
  FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100
  FIRMWARE : ORIGIN = 0x10100000, LENGTH = 0x38BD8  /* Added memory region */
  RAM : ORIGIN = 0x20000000, LENGTH = 264K
}

SECTIONS {

  /* ### Boot loader */
  .boot_loader ORIGIN(BOOT_LOADER) :
  {
    KEEP(*(.boot_loader*));
  } > BOOT_LOADER
  
  /* ### Firmware */
  .firmware ORIGIN(FIRMWARE) :
  {
    KEEP(*(.firmware*));
  } > FIRMWARE

} INSERT BEFORE .text;
