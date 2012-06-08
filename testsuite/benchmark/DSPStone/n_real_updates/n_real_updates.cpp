/*
 * benchmark program:   n_real_updates.c
 * 
 * benchmark suite:     DSP-kernel
 * 
 * description:         n_real_updates - filter benchmarking
 * 
 * This program performs n real updates of the form 
 *           D(i) = C(i) + A(i)*B(i),
 * where A(i), B(i), C(i) and D(i) are real numbers,
 * and i = 1,...,N
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
 * history:             25-5-94 creation (Martinez Velarde)
 *
 *                      $Author: schraut $
 *                      $Date: 1995/01/26 09:44:22 $
 *                      $Revision: 1.2 $
 */

#define STORAGE_CLASS register
#define TYPE          int
#define N             16

#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>
		void n_real_updates(TYPE *p_a,TYPE *p_b,TYPE *p_c,TYPE *p_d){
				STORAGE_CLASS TYPE i;
				for (i = 0 ; i < N ; i++) 
				*p_d++  = *p_c++ + *p_a++ * *p_b++ ;

		}
#ifdef __cplusplus
}
#endif

void
pin_down(TYPE *pa, TYPE *pb, TYPE *pc, TYPE *pd)
{
  STORAGE_CLASS int i ; 

  for (i=0 ; i < N ; i++)
    {
      *pa++ = 10 ; 
      *pb++ = 2 ; 
      *pc++ = 10 ; 
      *pd++ = 0 ; 
    }
}

TYPE main()
{
  static TYPE A[N], B[N], C[N], D[N] ; 
  
  STORAGE_CLASS TYPE *p_a = &A[0], *p_b = &B[0] ;
  STORAGE_CLASS TYPE *p_c = &C[0], *p_d = &D[0] ;
 // TYPE i ; 
  int j;

  pin_down(&A[0], &B[0], &C[0], &D[0]) ; 
  
  n_real_updates(p_a, p_b, p_c, p_d) ;
	
  for(j=0;j<16;j++)
			printf("D[%d]:%d\n",j,D[j]);
      
  pin_down(&A[0], &B[0], &C[0], &D[0]) ; 
  return(0)  ; 
}
