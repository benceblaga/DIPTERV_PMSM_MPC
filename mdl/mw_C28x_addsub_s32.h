/* Copyright 2012 The MathWorks, Inc. */

/*********************************************
*                                           *  
*   TI_C28x fuction lib for S32 operations  *
*                                           *  
*********************************************/

#ifndef MW_C28X_ADDSUB_S32_H
#define MW_C28X_ADDSUB_S32_H

extern int32_T asm_c28x_add_s32_s32_s32_sat(int32_T a, int32_T b);
extern int32_T asm_c28x_sub_s32_s32_s32_sat(int32_T a, int32_T b);
/*
* operation:         s32 + s32
* saturation:        Yes
* rounding mode:     N/A 
*/
inline int32_T c28x_add_s32_s32_s32_sat(int32_T a, int32_T b)
{
	int32_T result;	
	result = asm_c28x_add_s32_s32_s32_sat(a, b);
	return result;
}

/*
 * operation:         s32 - s32
 * saturation:        Yes
 * rounding mode:     N/A 
 */
inline int32_T c28x_sub_s32_s32_s32_sat(int32_T a, int32_T b)
{
	int32_T result;	
	result = asm_c28x_sub_s32_s32_s32_sat(a,b);
	return result;
}

#endif
