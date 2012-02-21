/*
 *  benchmark program  : biquad_one_section.c
 *
 *  benchmark suite    : DSP-kernel
 *
 *  description        : benchmarking of an one iir biquad
 *                       
 *
 *	The equations of the filter are:
 *       w(n) =    x(n) - a1*w(n-1) - a2*w(n-2)
 *       y(n) = b0*w(n) + b1*w(n-1) + b2*w(n-2)
 *
 *		             w(n)
 *   x (n)------(-)---------->-|->----b0---(+)-------> y(n)
 *               A             |            A
 *               |           |1/z|          |
 *               |             | w(n-1)     |
 *               |             v            |
 *               |-<--a1-----<-|->----b1-->-|
 *               |             |            |
 *               |           |1/z|          |
 *               |             | w(n-2)     |
 *               |             v            |
 *               |-<--a2-----<--->----b2-->-|
 * 
 *     The values w(n-1) and w(n-2) are stored in w1 and w2
 * 
 *                                              
 *  reference code     : 
 *
 *  func. verification : from separate computation
 *
 *  organization       : Aachen University of Technology - IS2
 *                     : DSP Tools Group
 *                     : phone   : +49(241)807887
 *                     : fax     : +49(241)8888195
 *                     : e-mail  : zivojnov@ert.rwth-aachen.de
 *
 *  author             : Juan Martinez Velarde
 *
 *  history            : creation 19-3-1994
 *
 *                     $Author: schraut $
 *                     $Date: 1995/01/30 07:33:38 $
 *                     $Revision: 1.2 $
 */


#define STORAGE_CLASS register
#define TYPE int
#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>
	TYPE biquad_one_section(TYPE a1,TYPE a2,TYPE b0,TYPE b1,TYPE b2,TYPE w1,TYPE w2,TYPE x){
		TYPE y ;
    STORAGE_CLASS TYPE w ; 
    
		w  = x - a1 * w1 ; 
		w -= a2 * w2 ; 
		y  = b0 * w ;
		y += b1 * w1 ; 
		y += b2 * w2 ; 
  
		w2 = w1 ; 
		w1 = w  ; 

		return(y);
	}
#ifdef __cplusplus
}
#endif

TYPE 
pin_down(TYPE x)
{
  return ((TYPE) 7) ;
}


TYPE main()
{
  
  STORAGE_CLASS TYPE y ; 

  static TYPE x = 7, w1= 7 , w2 = 7 ;  
  static TYPE b0 = 7, b1 = 7 , b2 = 7  ;
  static TYPE a1 = 7, a2 = 7 ; 

	y = biquad_one_section(a1,a2,b0,b1,b2,w1,w2,x) ;
  printf("y:%d\n",y);
  
  x  = pin_down(x) ;
  w1 = pin_down(w1) ; 
  w2 = pin_down(w2) ; 

  return(0) ; 
}
