/*
 * benchmark program:   complex_multiply.c
 * 
 * benchmark suite:     DSP-kernel
 * 
 * description:         complex_multiply - filter benchmarking
 * 
 * reference code:      target assembly
 * 
 * f. verification:     simulator based
 * 
 *  organization:        Aachen University of Technology - IS2 
 *                       DSP Tools Group
 *                       phone:  +49(241)807887 
 *                       fax:    +49(241)8888195
 *                       e-mail: zivojnov@ert.rwth-aachen.de 
 *
 * author:              Juan Martinez Velarde
 * 
 * history:             9-5-94 creation (Martinez Velarde)
 * 
 *                      $Author: schraut $
 *                      $Revision: 1.2 $
 *                      $Date: 1995/02/01 11:54:14 $
 */

#define STORAGE_CLASS static
#define TYPE  int

#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>

	void complex_multiply(TYPE *ar, TYPE *ai, TYPE *br, TYPE *bi, TYPE *cr, TYPE *ci){
	
		*cr  = *ar * *br - *ai * *bi ;
		*ci  = *ar * *bi + *ai * *br ;

	}
#ifdef __cplusplus
}
#endif

void
pin_down(TYPE *ar, TYPE *ai, TYPE *br, TYPE *bi, TYPE *cr, TYPE *ci)
{
  *ar = 2 ; 
  *ai = 1 ; 
  *br = 2 ; 
  *bi = 5 ; 
}

int main()
{
   STORAGE_CLASS TYPE ar, ai ;
   STORAGE_CLASS TYPE br, bi ;
   STORAGE_CLASS TYPE cr, ci ;

  pin_down(&ar, &ai, &br, &bi, &cr, &ci) ;
  
  complex_multiply(&ar, &ai, &br, &bi, &cr, &ci);
  
  printf("result cr: %d,ci: %d\n",cr,ci);
  
  pin_down(&ar, &ai, &br, &bi, &cr, &ci) ; 
  
  return 0;
}

