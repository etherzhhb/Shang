/*
 * benchmark program:   convolution.c
 * 
 * benchmark suite:     DSP-kernel
 * 
 * description:         convolution - filter benchmarking
 * 
 * reference code:      target assembly
 * 
 * f. verification:     none
 * 
 *  organization:        Aachen University of Technology - IS2 
 *                       DSP Tools Group
 *                       phone:  +49(241)807887 
 *                       fax:    +49(241)8888195
 *                       e-mail: zivojnov@ert.rwth-aachen.de 
 *
 * author:              Vojin Zivojnovic
 * 
 * history:             14-1-94 creation (Vojin Zivojnovic)
 *                      18-3-94 asm labels included (Martinez Velarde)
 *
 *                      $Author: schraut $
 *                      $Date: 1995/01/30 07:24:54 $
 *                      $Revision: 1.2 $
 */

#define STORAGE_CLASS register
#define TYPE  int
#define LENGTH 16

#ifdef __cplusplus
extern "C" {
#endif
#include<stdio.h>
	TYPE convolution( TYPE *px, TYPE*ph){

	TYPE y = 0;
	TYPE i;

	for (i = 0; i < LENGTH; ++i)
		y += *px++ * *ph--;
    
    return y;
}
#ifdef __cplusplus
}
#endif

void fill_input(TYPE * px, TYPE * ph)
{
	STORAGE_CLASS TYPE    i;

	for (i = 0; i < LENGTH; ++i) {
		*px++ = 1;
		*ph++ = 1;
	}

}


TYPE main()
{

	static TYPE     x[LENGTH];
	static TYPE     h[LENGTH];

	STORAGE_CLASS TYPE y;
	//STORAGE_CLASS TYPE i;
	STORAGE_CLASS TYPE *px = x;
	STORAGE_CLASS TYPE *ph = &h[LENGTH - 1];
        

	fill_input(&x[0], &h[0]);

	y = convolution(px,ph);
  
  printf("y: %d\n",y);

	return (0);

}
