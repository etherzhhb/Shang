/*
 * benchmark program:   dot_product.c
 * 
 * benchmark suite:     DSP-kernel
 * 
 * description:         dot product benchmarking
 *
 * This program performs a dot product of the form Z=AB,
 * where A is a [1x2] vector and B is a [2x1] vector.
 * 
 *          A[1 x 2] * B[2 x 1] = Z
 *                  
 * vector A[1 x 2]= |a1 a2|
 *
 * vector B[2 x 1]= | b1 |
 *                  | b2 |
 *
 * dot product Z = a1*b1 + a2* b2
 * 
 * vector elements are stored as
 *
 * A[1 x 2] = { a1, a1 }
 * 
 * B[2 x 1] = { b1, b2 }
 * 
 * 
 * reference code:       none
 * 
 * f. verification:      with printf function
 * 
 * organization:         Aachen University of Technology - IS2 
 *                       DSP Tools Group
 *                       phone:  +49(241)807887 
 *                       fax:    +49(241)8888195
 *                       e-mail: zivojnov@ert.rwth-aachen.de 
 *
 * author:              Juan Martinez Velarde
 * 
 * history:             10-05-94 C Code creation (Martinez Velarde)
 *
 *                      $Author: schraut $
 *                      $Date: 1995/01/26 11:10:35 $
 *                      $Revision: 1.2 $
 */

#define STORAGE_CLASS  register
#define TYPE           int

#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>
void dot_product(TYPE *p_z,TYPE *p_a,TYPE *p_b) 
	{  TYPE f;
		 for (f=0;f<2;f++)
				*p_z  += *p_a++ * *p_b++ ;
	}
#ifdef __cplusplus
}
#endif

void pin_down(TYPE *Z)
{
  *Z = 0 ; 
}

TYPE main()
{ 
  static  TYPE A[2] = {2,1} ; 
  static  TYPE B[2] = {2,5} ;
  static  TYPE Z    = 0   ;

  STORAGE_CLASS TYPE *p_a = &A[0] ;
  STORAGE_CLASS TYPE *p_b = &B[0] ;
  STORAGE_CLASS TYPE *p_z = &Z ;
  
  //TYPE f ; 

  pin_down(&Z) ; 
  
  dot_product(p_z,p_a,p_b) ; 
  printf("Z: %d\n",Z);
  pin_down(&Z) ; 
  
  return(0)  ; 
}
