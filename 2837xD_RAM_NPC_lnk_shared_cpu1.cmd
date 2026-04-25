// The user must define CLA_C in the project linker settings if using the
// CLA C compiler
// Project Properties -> C2000 Linker -> Advanced Options -> Command File
// Preprocessing -> --define
#ifdef CLA_C
// Define a size for the CLA scratchpad area that will be used
// by the CLA compiler for local symbols and temps
// Also force references to the special symbols that mark the
// scratchpad are.
CLA_SCRATCHPAD_SIZE = 0x100;
--undef_sym=__cla_scratchpad_end
--undef_sym=__cla_scratchpad_start
#endif //CLA_C


MEMORY
{
PAGE 0 :
   /* BEGIN is used for the "boot to SARAM" bootloader mode   */

   BEGIN           	: origin = 0x000000, length = 0x000002
   RAMM0           	: origin = 0x000123, length = 0x0002DD
   /*RAMD0          	: origin = 0x00B000, length = 0x000800*/
   /*RAMD1          	: origin = 0x00B800, length = 0x000800*/
   RAMD0_1			: origin = 0x00B000, length = 0x001000
   RAMLS2      		: origin = 0x009000, length = 0x000800
   /*RAMLS4      		: origin = 0x00A000, length = 0x000800*/
   /*RAMLS5      		: origin = 0x00A800, length = 0x000800*/
   RAMLS4_5         : origin = 0x00A000,   length = 0x001000
   //RAMGS14          : origin = 0x01A000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   RAMGS15          : origin = 0x01B000, length = 0x000FF8     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   //RAMGS11     : origin = 0x017000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   //RAMGS13     : origin = 0x019000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */

   /* Összevonjuk a RAMGS10, 11, 12, 13, 14 blokkokat */
   RAMGS_TEXT : origin = 0x016000, length = 0x005000

//   RAMGS15_RSVD     : origin = 0x01BFF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */
                                                                /* Only on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   RESET           	: origin = 0x3FFFC0, length = 0x000002

PAGE 1 :

   BOOT_RSVD   : origin = 0x000002, length = 0x000121     /* Part of M0, BOOT rom will use this for stack */
   RAMM1       : origin = 0x000400, length = 0x000400     /* on-chip RAM block M1 */
   RAMLS0      : origin = 0x008000, length = 0x000800
   RAMLS1      : origin = 0x008800, length = 0x000800
   RAMLS3      : origin = 0x009800, length = 0x000800
   RAMGS0      : origin = 0x00C000, length = 0x001000
   RAMGS1      : origin = 0x00D000, length = 0x001000
   RAMGS2      : origin = 0x00E000, length = 0x001000
   RAMGS3      : origin = 0x00F000, length = 0x001000
   RAMGS4      : origin = 0x010000, length = 0x001000
   RAMGS5      : origin = 0x011000, length = 0x001000
   RAMGS6      : origin = 0x012000, length = 0x001000
   //RAMGS7      : origin = 0x013000, length = 0x001000
   //RAMGS8      : origin = 0x014000, length = 0x001000
   //RAMGS9      : origin = 0x015000, length = 0x001000
   //RAMGS10     : origin = 0x016000, length = 0x001000

	/* RAMGS7, 8, 9 blokkok */
   RAMGS_EBSS  : origin = 0x013000, length = 0x003000

//   RAMGS11     : origin = 0x017000, length = 0x000FF8   /* Uncomment for F28374D, F28376D devices */

//   RAMGS11_RSVD : origin = 0x017FF8, length = 0x000008    /* Reserve and do not use for code as per the errata advisory "Memory: Prefetching Beyond Valid Memory" */

//   RAMGS11     : origin = 0x017000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   RAMGS12     : origin = 0x018000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   //RAMGS13     : origin = 0x019000, length = 0x001000     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */

   CANA_MSG_RAM     : origin = 0x049000, length = 0x000800
   CANB_MSG_RAM     : origin = 0x04B000, length = 0x000800
   
   CLA1_MSGRAMLOW   : origin = 0x001480,   length = 0x000080
   CLA1_MSGRAMHIGH  : origin = 0x001500,   length = 0x000080
}

SECTIONS
{
   codestart        : > BEGIN,     PAGE = 0
   //.text            : >>RAMD0_1 | RAMLS2 | RAMGS11 | RAMGS13 | RAMLS4_5 | RAMGS14,   PAGE = 0 // ÁTÍRVA
   .text            : > RAMGS_TEXT,   PAGE = 0
   //.cinit           : > RAMM0,     PAGE = 0
   .cinit           : > RAMD0_1,     PAGE = 0 //ÁTÍRVA
   .switch          : > RAMM0,     PAGE = 0
   .reset           : > RESET,     PAGE = 0, TYPE = DSECT /* not used, */
   .stack           : > RAMM1,     PAGE = 1
   
#if defined(__TI_EABI__)
   .bss             : > RAMLS3,    PAGE = 1
   .bss:output      : > RAMLS3,    PAGE = 0
   .init_array      : > RAMM0,     PAGE = 0
   .const           : > RAMLS3,    PAGE = 1
   .data            : > RAMLS3,    PAGE = 1
   .sysmem          : > RAMLS3,    PAGE = 1
#else
   .pinit           : > RAMM0,     PAGE = 0
   //.ebss            : >> RAMLS3 | RAMGS12 | RAMGS9 | RAMGS10,   PAGE = 1 // ÁTÍRVA
   .ebss            : > RAMGS_EBSS,   PAGE = 1
   .econst          : > RAMLS3,    PAGE = 1
   .esysmem         : > RAMLS3,    PAGE = 1
#endif

   Filter_RegsFile  : > RAMGS0,	   PAGE = 1

   SHARERAMGS0		: > RAMGS0,		PAGE = 1
   SHARERAMGS1		: > RAMGS1,		PAGE = 1
   SHARERAMGS2		: > RAMGS2,		PAGE = 1
   SHARERAMGS3		: > RAMGS3,		PAGE = 1
   SHARERAMGS4		: > RAMGS4,		PAGE = 1
   SHARERAMGS5		: > RAMGS5,		PAGE = 1
   SHARERAMGS6		: > RAMGS6,		PAGE = 1
   //SHARERAMGS7		: > RAMGS7,		PAGE = 1
   //SHARERAMGS8		: > RAMGS8,		PAGE = 1
   //SHARERAMGS9		: > RAMGS9,		PAGE = 1
   //SHARERAMGS10		: > RAMGS10,	PAGE = 1
   //SHARERAMGS11		: > RAMGS11,	PAGE = 0
   //SHARERAMGS12		: > RAMGS12,	PAGE = 1     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   //SHARERAMGS13		: > RAMGS13,	PAGE = 0     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   //SHARERAMGS14		: > RAMGS14,	PAGE = 0     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */
   SHARERAMGS15		: > RAMGS15,	PAGE = 0     /* Only Available on F28379D, F28377D, F28375D devices. Remove line on other devices. */

    /* CLA specific sections */
   Cla1Prog         : > RAMLS4_5, PAGE=0

   CLADataLS0		: > RAMLS0, PAGE=1
   CLADataLS1		: > RAMLS1, PAGE=1

   Cla1ToCpuMsgRAM  : > CLA1_MSGRAMLOW,   PAGE = 1
   CpuToCla1MsgRAM  : > CLA1_MSGRAMHIGH,  PAGE = 1

#ifdef __TI_COMPILER_VERSION__
   #if __TI_COMPILER_VERSION__ >= 15009000
    .TI.ramfunc : {} > RAMM0,      PAGE = 0
   #else
   ramfuncs         : > RAMM0      PAGE = 0   
   #endif
#endif

#ifdef CLA_C
   /* CLA C compiler sections */
   //
   // Must be allocated to memory the CLA has write access to
   //
   CLAscratch       :
                     { *.obj(CLAscratch)
                     . += CLA_SCRATCHPAD_SIZE;
                     *.obj(CLAscratch_end) } >  RAMLS1,  PAGE = 1

   .scratchpad      : > RAMLS1,       PAGE = 1
   .bss_cla		    : > RAMLS1,       PAGE = 1
   .const_cla	    : > RAMLS1,       PAGE = 1
#endif //CLA_C
}

/*
//===========================================================================
// End of file.
//===========================================================================
*/
