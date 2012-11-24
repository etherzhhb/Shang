FUs.ClkEnSelLatency = 1.535 / PERIOD --1.535
FUs.MaxLutSize = 4
FUs.LutLatency = 0.635 / PERIOD
-- Latency table for EP2C35F672C6
FUs.AddSub = { Latencies = { 1.430 / PERIOD, 2.615 / PERIOD, 3.260 / PERIOD, 4.556 / PERIOD, 7.099 / PERIOD }, --Add 
               Costs = {2 * 64, 10 * 64, 18 * 64, 34 * 64, 66 * 64}, --Add
               StartInterval=1,
			         ChainingThreshold = ADDSUB_ChainingThreshold}
FUs.Shift = { Latencies = { 1.191 / PERIOD, 3.338 / PERIOD, 4.415 / PERIOD, 5.150 / PERIOD, 6.428 / PERIOD }, --Shift 
	            Costs = {1 * 64, 27 * 64, 70 * 64, 171 * 64, 393 * 64}, --Shift
              StartInterval=1,
			        ChainingThreshold = SHIFT_ChainingThreshold}
FUs.Mult = { Latencies = { 1.195 / PERIOD, 4.237 / PERIOD, 4.661 / PERIOD, 9.519 / PERIOD, 12.616 / PERIOD }, --Mul 
             Costs = {1 * 64, 0 * 64, 0 * 64, 28 * 64, 168 * 64}, --Mul
             StartInterval=1,
			       ChainingThreshold = MULT_ChainingThreshold}

FUs.ICmp   = { Latencies = { 1.191 / PERIOD, 2.612 / PERIOD, 3.253 / PERIOD, 4.531 / PERIOD, 7.083 / PERIOD }, --Cmp 
               Costs = {1 * 64, 8 * 64, 16 * 64, 32 * 64, 64 * 64}, --Cmp
               StartInterval=1,
			         ChainingThreshold = ICMP_ChainingThreshold}
FUs.Sel = { Latencies = { 1.376 / PERIOD, 1.596 / PERIOD, 1.828 / PERIOD, 1.821 / PERIOD, 2.839 / PERIOD }, --Sel 
            Costs = {1 * 64, 8 * 64, 16 * 64, 32 * 64, 64 * 64}, --Sel 
            StartInterval=1,
            ChainingThreshold = SEL_ChainingThreshold}
FUs.Reduction = { Latencies = { 0.988 / PERIOD, 1.958 / PERIOD, 2.103 / PERIOD, 2.852 / PERIOD, 3.230 / PERIOD }, --Red 
	                Costs = {0 * 64, 3 * 64, 5 * 64, 11 * 64, 21 * 64}, --Red
                  StartInterval=1,
                  ChainingThreshold = REDUCTION_ChainingThreshold}
FUs.Mux    = { MaxAllowedMuxSize = 32,
               Latencies = {{ 1.636 / PERIOD, 1.811 / PERIOD, 1.892 / PERIOD, 2.281 / PERIOD }, --2-input 
                            { 2.631 / PERIOD, 2.631 / PERIOD, 3.070 / PERIOD, 3.134 / PERIOD }, --3-input 
                            { 3.276 / PERIOD, 3.434 / PERIOD, 3.454 / PERIOD, 3.567 / PERIOD }, --4-input 
                            { 3.257 / PERIOD, 3.345 / PERIOD, 3.377 / PERIOD, 3.675 / PERIOD }, --5-input 
                            { 3.294 / PERIOD, 3.568 / PERIOD, 3.760 / PERIOD, 4.132 / PERIOD }, --6-input 
                            { 3.366 / PERIOD, 3.602 / PERIOD, 3.980 / PERIOD, 4.006 / PERIOD }, --7-input 
                            { 3.736 / PERIOD, 4.245 / PERIOD, 4.284 / PERIOD, 4.754 / PERIOD }, --8-input 
                            { 3.939 / PERIOD, 4.261 / PERIOD, 4.422 / PERIOD, 5.111 / PERIOD }, --9-input 
                            { 4.567 / PERIOD, 4.713 / PERIOD, 4.772 / PERIOD, 4.933 / PERIOD }, --10-input 
                            { 5.354 / PERIOD, 5.244 / PERIOD, 4.908 / PERIOD, 5.218 / PERIOD }, --11-input 
                            { 4.765 / PERIOD, 5.173 / PERIOD, 5.220 / PERIOD, 5.412 / PERIOD }, --12-input 
                            { 5.051 / PERIOD, 5.295 / PERIOD, 5.353 / PERIOD, 5.970 / PERIOD }, --13-input 
                            { 5.181 / PERIOD, 5.300 / PERIOD, 5.679 / PERIOD, 6.130 / PERIOD }, --14-input 
                            { 5.210 / PERIOD, 5.844 / PERIOD, 5.811 / PERIOD, 6.044 / PERIOD }, --15-input 
                            { 5.771 / PERIOD, 5.748 / PERIOD, 5.934 / PERIOD, 6.448 / PERIOD }, --16-input 
                            { 5.901 / PERIOD, 5.930 / PERIOD, 6.483 / PERIOD, 6.820 / PERIOD }, --17-input 
                            { 5.590 / PERIOD, 6.235 / PERIOD, 7.051 / PERIOD, 6.741 / PERIOD }, --18-input 
                            { 6.354 / PERIOD, 6.564 / PERIOD, 6.957 / PERIOD, 7.106 / PERIOD }, --19-input 
                            { 6.692 / PERIOD, 6.589 / PERIOD, 7.336 / PERIOD, 7.241 / PERIOD }, --20-input 
                            { 7.327 / PERIOD, 6.743 / PERIOD, 7.345 / PERIOD, 7.779 / PERIOD }, --21-input 
                            { 6.723 / PERIOD, 7.217 / PERIOD, 7.155 / PERIOD, 7.515 / PERIOD }, --22-input 
                            { 7.559 / PERIOD, 7.878 / PERIOD, 7.438 / PERIOD, 7.807 / PERIOD }, --23-input 
                            { 7.308 / PERIOD, 7.265 / PERIOD, 8.437 / PERIOD, 8.491 / PERIOD }, --24-input 
                            { 7.673 / PERIOD, 7.718 / PERIOD, 7.692 / PERIOD, 8.972 / PERIOD }, --25-input 
                            { 7.652 / PERIOD, 7.936 / PERIOD, 7.913 / PERIOD, 8.529 / PERIOD }, --26-input 
                            { 7.916 / PERIOD, 8.349 / PERIOD, 8.549 / PERIOD, 9.189 / PERIOD }, --27-input 
                            { 7.885 / PERIOD, 9.021 / PERIOD, 8.517 / PERIOD, 9.411 / PERIOD }, --28-input 
                            { 8.508 / PERIOD, 8.435 / PERIOD, 8.746 / PERIOD, 9.303 / PERIOD }, --29-input 
                            { 8.424 / PERIOD, 8.826 / PERIOD, 9.318 / PERIOD, 9.773 / PERIOD }, --30-input 
                            { 8.232 / PERIOD, 8.897 / PERIOD, 9.807 / PERIOD, 9.970 / PERIOD }, --31-input 
                            { 8.572 / PERIOD, 8.886 / PERIOD, 9.536 / PERIOD, 10.196 / PERIOD } --32-input 
                           },
               Costs = {{64 , 512 , 1024 , 2048 , 4096},--2-input 
                        {128 , 128 , 128 , 128 , 128},--3-input 
                        {192 , 1024 , 2048 , 4096 , 8192},--4-input 
                        {256 , 2176 , 4224 , 8320 , 16512},--5-input 
                        {320 , 2560 , 5120 , 10368 , 20352},--6-input 
                        {384 , 3200 , 6272 , 12416 , 24704},--7-input 
                        {512 , 3456 , 6592 , 12800 , 25088},--8-input 
                        {512 , 3648 , 7552 , 12864 , 28736},--9-input 
                        {576 , 4672 , 9152 , 17152 , 34816},--10-input 
                        {832 , 5248 , 9344 , 18880 , 35136},--11-input 
                        {768 , 4992 , 9408 , 17216 , 34176},--12-input 
                        {1088 , 6016 , 11136 , 21376 , 42880},--13-input 
                        {1216 , 6528 , 12096 , 23488 , 43008},--14-input 
                        {1216 , 6272 , 12416 , 21696 , 42176},--15-input 
                        {1472 , 7232 , 14400 , 26112 , 51072},--16-input 
                        {1280 , 8384 , 15616 , 28096 , 54912},--17-input 
                        {1664 , 7552 , 15936 , 30528 , 50560},--18-input 
                        {1536 , 9408 , 16640 , 32448 , 63616},--19-input 
                        {1600 , 9600 , 17856 , 34624 , 67968},--20-input 
                        {1984 , 9792 , 16000 , 34880 , 68160},--21-input 
                        {1664 , 10368 , 19008 , 36864 , 72320},--22-input 
                        {1856 , 10880 , 20032 , 39104 , 76672},--23-input 
                        {1856 , 11136 , 20480 , 39296 , 76480},--24-input 
                        {2112 , 11712 , 21440 , 41216 , 80832},--25-input 
                        {2048 , 12160 , 22528 , 43008 , 84032},--26-input 
                        {2176 , 12352 , 22656 , 43392 , 84800},--27-input 
                        {2368 , 12864 , 23616 , 45312 , 88576},--28-input 
                        {2560 , 13440 , 24832 , 47488 , 92800},--29-input 
                        {2432 , 14016 , 24896 , 49408 , 92736},--30-input 
                        {2688 , 14592 , 25984 , 51776 , 97152},--31-input 
                        {2688 , 14784 , 28224 , 51840 , 106048}--32-input 
                       }, StartInterval=1,
			         ChainingThreshold = MUX_ChainingThreshold}
