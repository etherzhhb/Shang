/*
 * benchmark program:   fir.c
 * 
 * benchmark suite:     DSP-kernel
 * 
 * description:         fir - filter benchmarking
 * 
 * reference code:      target assembly
 * 
 * f. verification:     simulator
 * 
 *  organization:        Aachen University of Technology - IS2 
 *                       DSP Tools Group
 *                       phone:  +49(241)807887 
 *                       fax:    +49(241)8888195
 *                       e-mail: zivojnov@ert.rwth-aachen.de 
 *
 * author:              Juan Martinez Velarde
 * 
 * history:             12-4-94 creation (Martinez Velarde) 
 *                      22-1-95 START and END PROFILING Macros included
 *                              fill_input routine -> 3 arguments (Schraut)
 *
 *                      $Date: 1995/01/25 17:14:52 $
 *                      $Author: schraut $
 *                      $Revision: 1.2 $
 */

#define STORAGE_CLASS register
#define TYPE  int
#define LENGTH 16

#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>
	TYPE fir(TYPE *px,TYPE *px2,TYPE *ph,TYPE  x0){
  
	TYPE y = 0;
	STORAGE_CLASS TYPE i ;
	  for (i = 0; i < LENGTH - 1; i++)
		{       
		  y += *ph-- * *px ; 
		  *px-- = *px2-- ; 
		}
  
	  y += *ph * *px ;
	  *px = x0 ;
    
    return (y);
	}
#ifdef __cplusplus
}
#endif

void
fill_input(TYPE * px, TYPE * ph, TYPE y)
{
  STORAGE_CLASS TYPE    i;
  
  for (i = 1; i <= LENGTH; i++) 
    {
      *px++ = i;
      *ph++ = i;
    }
  
}

TYPE
main()
{
  static TYPE  x[LENGTH];
  static TYPE  h[LENGTH];
  
  static TYPE  x0 = 100;

  //STORAGE_CLASS TYPE i ;
  STORAGE_CLASS TYPE *px, *px2 ;
  STORAGE_CLASS TYPE *ph ;
  STORAGE_CLASS TYPE y;
  
  fill_input(x, h, y);
  
  ph  = &h[LENGTH-1] ; 
  px  = &x[LENGTH-1]  ; 
  px2 = &x[LENGTH-2]  ; 
  
  y = fir(px,px2,ph,x0);
  
  printf("y: %d\n",y);
  
  fill_input(x, h, y);

  return (0); 
}
