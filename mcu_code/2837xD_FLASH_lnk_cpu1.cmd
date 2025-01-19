
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are.

CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start

MEMORY
{
PAGE 0 :  /* Program Memory */
          /* Memory (RAM/FLASH) blocks can be moved to PAGE1 for data allocation */
          /* BEGIN is used for the "boot to Flash" bootloader mode   */

   BEGIN            : origin = 0x080000, length = 0x000002

   RAMLS0           : origin = 0x008000, length = 0x000800

   RAMLS2_5_RAMD0_1 : origin = 0x009000, length = 0x003000

   RAMGS13_15        : origin = 0x019000, length = 0x002F00

   RAMGS15_boot      : origin = 0x01BF00, length = 0x0000F8

   RESET            : origin = 0x3FFFC0, length = 0x000002

   /* Flash sectors */
   FLASHA           : origin = 0x080002, length = 0x001FFE	/* on-chip Flash */
   FLASHB           : origin = 0x082000, length = 0x002000	/* on-chip Flash */
   FLASHC           : origin = 0x084000, length = 0x002000	/* on-chip Flash */
   FLASHD           : origin = 0x086000, length = 0x002000	/* on-chip Flash */
   FLASHE           : origin = 0x088000, length = 0x008000	/* on-chip Flash */
   FLASHF           : origin = 0x090000, length = 0x008000	/* on-chip Flash */
   FLASHG           : origin = 0x098000, length = 0x008000	/* on-chip Flash */

   /*
   FLASHH           : origin = 0x0A0000, length = 0x008000
   FLASHI           : origin = 0x0A8000, length = 0x008000
   FLASHJ           : origin = 0x0B0000, length = 0x008000
   FLASHK           : origin = 0x0B8000, length = 0x002000
   FLASHL           : origin = 0x0BA000, length = 0x002000
   FLASHM           : origin = 0x0BC000, length = 0x002000
   FLASHN           : origin = 0x0BE000, length = 0x002000
   */

PAGE 1 : /* Data Memory */
         /* Memory (RAM/FLASH) blocks can be moved to PAGE0 for program allocation */

   BOOT_RSVD         : origin = 0x000002, length = 0x000122     /* Part of M0, BOOT rom will use this for stack */

   RAMM0_1           : origin = 0x000124, length = 0x0006D0
/* RAMM1_RSVD        : origin = 0x0007F8, length = 0x000008     Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

   CLA1_MSGRAMLOW    : origin = 0x001480, length = 0x000080
   CLA1_MSGRAMHIGH   : origin = 0x001500, length = 0x000080

   RAMLS1            : origin = 0x008800, length = 0x000800

   RAMGS0_11         : origin = 0x00C000, length = 0x00C000

   RAMGS12           : origin = 0x018000, length = 0x001000

/* RAMGS15_RSVD      : origin = 0x01BFF8, length = 0x000008     Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

/*
   EXT_SRAM_CS3      : origin = 0x300000, length = 0x040000
*/

   CPU2TOCPU1RAM     : origin = 0x03F800, length = 0x000400
   CPU1TOCPU2RAM     : origin = 0x03FC00, length = 0x000400

   CANA_MSG_RAM      : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM      : origin = 0x04B000, length = 0x000800

}

SECTIONS
{
   /* Allocate program areas: */
   .cinit              : > FLASHB,     PAGE = 0, ALIGN(8)
   .pinit              : > FLASHB,     PAGE = 0, ALIGN(8)
   .text               : >> FLASHB | FLASHC | FLASHD | FLASHE, PAGE = 0, ALIGN(8)
   codestart           : > BEGIN,      PAGE = 0, ALIGN(8)

   /* Allocate uninitalized data sections: */
   .stack              : > RAMM0_1,      PAGE = 1
   .ebss               : > RAMGS12,      PAGE = 1
   .esysmem            : > RAMGS12,      PAGE = 1

   /* Initalized sections go in Flash */
   .econst             : >> FLASHF,      PAGE = 0, ALIGN(8)
   .switch             : > FLASHB,       PAGE = 0, ALIGN(8)

   .reset              : > RESET,        PAGE = 0, TYPE = DSECT /* not used, */

   int_ram_data        : > RAMGS0_11,    PAGE = 1

/*
   ext_ram_data        : > EXT_SRAM_CS3, PAGE = 1 , TYPE = NOLOAD
*/

   Cla1ToCpuMsgRAM     : > CLA1_MSGRAMLOW,  PAGE = 1
   CpuToCla1MsgRAM     : > CLA1_MSGRAMHIGH, PAGE = 1

#ifdef __TI_COMPILER_VERSION__
	#if __TI_COMPILER_VERSION__ >= 15009000
	.TI.ramfunc        : {} LOAD = FLASHG,
                           RUN = RAMLS2_5_RAMD0_1,
                           LOAD_START(_RamfuncsLoadStart),
                           LOAD_SIZE(_RamfuncsLoadSize),
                           LOAD_END(_RamfuncsLoadEnd),
                           RUN_START(_RamfuncsRunStart),
                           RUN_SIZE(_RamfuncsRunSize),
                           RUN_END(_RamfuncsRunEnd),
                           PAGE = 0, ALIGN(8)
	#else
   ramfuncs            : LOAD = FLASHG,
                           RUN = RAMLS2_5_RAMD0_1,
                           LOAD_START(_RamfuncsLoadStart),
                           LOAD_SIZE(_RamfuncsLoadSize),
                           LOAD_END(_RamfuncsLoadEnd),
                           RUN_START(_RamfuncsRunStart),
                           RUN_SIZE(_RamfuncsRunSize),
                           RUN_END(_RamfuncsRunEnd),
                           PAGE = 0, ALIGN(8)
    #endif
#endif

   Cla1Prog            : LOAD = FLASHG,
                         RUN = RAMLS0,
                           LOAD_START(_Cla1funcsLoadStart),
                           LOAD_END(_Cla1funcsLoadEnd),
                           RUN_START(_Cla1funcsRunStart),
                           LOAD_SIZE(_Cla1funcsLoadSize),
                           PAGE = 0, ALIGN(8)

   /* CLA C compiler sections */
   // Must be allocated to memory the CLA has write access to
   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS1,  PAGE = 1

   .scratchpad      : > RAMLS1,       PAGE = 1
   .bss_cla         : > RAMLS1,       PAGE = 1
   .const_cla       :  LOAD = FLASHG,
                           RUN = RAMLS1,
                           RUN_START(_Cla1ConstRunStart),
                           LOAD_START(_Cla1ConstLoadStart),
                           LOAD_SIZE(_Cla1ConstLoadSize),
                           PAGE = 1

   cpu2bootsect     : LOAD = FLASHG,
                           RUN = RAMGS15_boot,
                           LOAD_START(_cpu2bootLoadStart),
                           LOAD_SIZE(_cpu2bootLoadSize),
                           LOAD_END(_cpu2bootLoadEnd),
                           RUN_START(_cpu2bootRunStart),
                           RUN_SIZE(_cpu2bootRunSize),
                           RUN_END(_cpu2bootRunEnd),
                           PAGE = 0, ALIGN(8)

   ramfuncs2        : LOAD = FLASHG,
                           RUN = RAMGS13_15,
                           LOAD_START(_Ramfuncs2LoadStart),
                           LOAD_SIZE(_Ramfuncs2LoadSize),
                           LOAD_END(_Ramfuncs2LoadEnd),
                           RUN_START(_Ramfuncs2RunStart),
                           RUN_SIZE(_Ramfuncs2RunSize),
                           RUN_END(_Ramfuncs2RunEnd),
                           PAGE = 0, ALIGN(8)

   /* The following section definitions are required when using the IPC API Drivers */
    GROUP : > CPU1TOCPU2RAM, PAGE = 1
    {
        PUTBUFFER
        PUTWRITEIDX
        GETREADIDX
        MSGRAM_CPU1_TO_CPU2
    }

    GROUP : > CPU2TOCPU1RAM, PAGE = 1
    {
        GETBUFFER           :   TYPE = DSECT
        GETWRITEIDX         :   TYPE = DSECT
        PUTREADIDX          :   TYPE = DSECT
        MSGRAM_CPU2_TO_CPU1 :   TYPE = DSECT
    }

   cpu2bss         : > CPU2TOCPU1RAM,       PAGE = 1
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
