    //============================================================================\\
   //                                                                              \\
  //              <<<    PMSM TUSTIN SYNCHRONIZED TWO-LEVEL MPC   >>>               \\
 //                                  CPU_1 HEADER                                    \\
//====================================================================================\\


  //
 // Included Files
//
#include "F2837xD_Cla_defines.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


  //
 // Defines
//
#define BUFFER_SIZE       64
#define TABLE_SIZE        64
#define TABLE_SIZE_M_1    TABLE_SIZE-1

#define PIBYTWO           1.570796326794897
#define PI                3.141592653589793
#define TWOPI             6.283185307179586
#define INV2PI            0.159154943091895
#define OneOverSqrt3      0.577350269189626
#define Sqrt3             1.732050807568877


  //
 // CPU :: CLA Communication
//

extern float CPU_2_CLA[15];
extern float CLA_2_CPU[16];


// The following are symbols defined in the CLA assembly code
// Including them in the shared header file makes them
// .global and the main CPU can make use of them.

__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

#ifdef __cplusplus
}
#endif // extern "C"

//
// End of file
//
