FUs.ClkEnSelLatency = 1.535 / PERIOD --1.535
FUs.MaxLutSize = 4
FUs.MaxMuxPerLUT = 2
FUs.LutLatency = 0.635 / PERIOD
-- Latency table for EP2C35F672C6
FUs.AddSub = { Latencies = { 2.113 / PERIOD, 2.700 / PERIOD, 4.052 / PERIOD, 6.608 / PERIOD },
               Costs = {128, 576, 1088, 2112, 4160}, StartInterval=1,
			         ChainingThreshold = ADDSUB_ChainingThreshold}
FUs.Shift  = { Latencies = { 3.073 / PERIOD, 3.311 / PERIOD, 4.792 / PERIOD, 5.829 / PERIOD },
               Costs = {64, 1792, 4544, 11072, 26752}, StartInterval=1,
			         ChainingThreshold = SHIFT_ChainingThreshold}
FUs.Mult   = { Latencies = { 2.181 / PERIOD, 2.504 / PERIOD, 6.503 / PERIOD, 9.221 / PERIOD },
               Costs = {64, 7424, 14784, 46016, 158016}, StartInterval=1,
			         ChainingThreshold = MULT_ChainingThreshold}
FUs.ICmp   = { Latencies = { 1.909 / PERIOD, 2.752 / PERIOD, 4.669 / PERIOD, 7.342 / PERIOD },
               Costs = {64, 512, 1024, 2048, 4096}, StartInterval=1,
			         ChainingThreshold = ICMP_ChainingThreshold}
FUs.Sel    = { Latencies = { 0.835 / PERIOD, 1.026 / PERIOD, 1.209 / PERIOD, 2.769 / PERIOD },
               Costs = {64, 512, 1024, 2048, 4096}, StartInterval=1,
			         ChainingThreshold = ICMP_ChainingThreshold}
