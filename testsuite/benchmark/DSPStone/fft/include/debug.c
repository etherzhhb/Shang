/*
 * convert float to fract (format 1.NUMBER_OF_BITS)
 *
 * author : Juan Martinez Velarde
 *  
 * 07-02-94
 *
 */

FILE           *FFT_FILE ; 

void pin_down_debug()
{
  
  /* open dump file */
  
  char            fname[20];
  char           *pfname = fname;
  
  fname[0] = (char) NULL ;
  
#ifdef __DSP56000__
    strcat(pfname, "inpsca_16.dat") ; 
#elif __DSP56156__
    strcat(pfname, "inpsca_16.dat") ; 
#else
    strcat(pfname, "unix_inpsca_16.dat") ; 
#endif
  
  if ((FFT_FILE = fopen(fname, "w")) == 0) 
    {
      printf("%s \n", "Cannot open output file %s\n correctly", fname);
      exit(1);
      }
}


void
dump_to_file(TYPE input_data[])
     // TYPE input_data[] ; 
{
  
  double m, exponent = NUMBER_OF_BITS ; 
  STORAGE_CLASS TYPE f  ;
  STORAGE_CLASS TYPE *ps; 
  
  /* write the real values to the dump file */
  
  ps = &input_data[0];
  
#ifdef __DSP56000__
  m = pow(exponent,2)  ;
#elif __DSP56156__
  m = pow(exponent,2) ;  
#else
  m = exp2(exponent)  ; 
#endif
  
  for (f = 0; f < N_FFT; f++) 
    {
      fprintf(FFT_FILE, "%i %f\n",f, ((float) (*ps)) / m ) ; 
      ps += 2 ; 
    }
  
  fclose(FFT_FILE);
  
}
