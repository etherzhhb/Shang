#! /usr/bin/python

import json
from xlwt import Workbook
import re
import sys

def geomean(nums):
    return (reduce(lambda x, y: x*y, nums))**(1.0/len(nums))

def getFreq(data):
    return float(re.match(r"(^[1-9]\d*\.\d*|0\.\d*[1-9]\d*$|^\d*)",data).group(0))

def getOthers(data):
    data=data.replace(',','')
    return float(re.match(r"(^\d*)",data).group(1))

def append(list, data):
    if data != 0 :
        list.append(data)
    return
#Ready to write a excel file  		
book = Workbook()
#Get the input and output file name from the shell commend 
InFile = sys.argv[1]
OutFile = sys.argv[2]
#Open a Json file
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)
#write the excel and give name to the first sheet "Sheet 1" 
sheet1 = book.add_sheet('Sheet 1')
sheet1.write(0,0,"name")
#write the row 1 column 0 and give the name "restricted_fmax"
sheet1.write(1,0,"restricted_fmax") 
sheet1.write(2,0,"cycles")
sheet1.write(3,0,"run_time")
sheet1.write(4,0,"les")
sheet1.write(5,0,"les_wo_reg")
sheet1.write(6,0,"les_w_reg_only")
sheet1.write(7,0,"les_and_reg")
sheet1.write(8,0,"lut4")
sheet1.write(9,0,"lut3")
sheet1.write(10,0,"lut2")
sheet1.write(11,0,"regs")
sheet1.write(12,0,"mult9")
sheet1.write(13,0,"equ_le")
all_freq = []
all_cycles = []
all_run_time = []
all_les = []
all_les_wo_reg = []
all_les_w_reg_only = []
all_les_and_reg = []
all_lut4 = []
all_lut3 = []
all_lut2 = []
all_regs = []
all_mult9 = []
all_equ_le = []
ColNumber = 1
for data in json_read:	
      sheet1.write(0,ColNumber,data["name"])
      #Frequency
      freq = getOthers(data["restricted_fmax"])
      sheet1.write(1,ColNumber,freq)
      #Cycles
      cycles = getOthers(data["cycles"])
      sheet1.write(2,ColNumber,cycles)
      # Run time.
      run_time = cycles/freq
      sheet1.write(3,ColNumber,run_time)
      #Logic elements.
      les = getOthers(data["les"])
      sheet1.write(4,ColNumber,les)
      #les_wo_reg
      les_wo_reg = getOthers(data["les_wo_reg"])
      sheet1.write(5,ColNumber,les_wo_reg)
      #les_w_reg_only
      les_w_reg_only = getOthers(data["les_w_reg_only"])
      sheet1.write(6,ColNumber,les_w_reg_only)
      #les_and_reg
      les_and_reg = getOthers(data["les_and_reg"])
      sheet1.write(7,ColNumber,les_and_reg)
      #lut4
      lut4 = getOthers(data["lut4"])
      sheet1.write(8,ColNumber,lut4)
      #lut3
      lut3 = getOthers(data["lut3"])
      sheet1.write(9,ColNumber,lut3)
      #lut2
      lut2 = getOthers(data["lut2"])
      sheet1.write(10,ColNumber,lut2)
      #regs
      regs = getOthers(data["regs"])
      sheet1.write(11,ColNumber,regs)
      #mult9
      mult9 = getOthers(data["mult9"])
      sheet1.write(12,ColNumber,mult9)
      #equ LEs
      equ_le = mult9 * 115 + les
      sheet1.write(13,ColNumber,equ_le)
      append(all_freq, freq)
      append(all_cycles, cycles)
      append(all_run_time, run_time)
      append(all_les, les)
      append(all_les_wo_reg, les_wo_reg)
      append(all_les_w_reg_only, les_w_reg_only)
      append(all_les_and_reg, les_and_reg)
      append(all_lut4, lut4)
      append(all_lut3, lut3)
      append(all_lut2, lut2)
      append(all_regs, regs)
      append(all_mult9, mult9)
      append(all_equ_le, equ_le)
			#the column is changed after a loop
      ColNumber = ColNumber + 1
#write the last column with the geomean
sheet1.write(0,ColNumber,"geomean")
freq_mean = geomean(all_freq)
sheet1.write(1,ColNumber, freq_mean)
sheet1.write(2,ColNumber,geomean(all_cycles))
sheet1.write(3,ColNumber,geomean(all_run_time))
sheet1.write(4,ColNumber,geomean(all_les))
sheet1.write(5,ColNumber,geomean(all_les_wo_reg))
sheet1.write(6,ColNumber,geomean(all_les_w_reg_only))
sheet1.write(7,ColNumber,geomean(all_les_and_reg))
sheet1.write(8,ColNumber,geomean(all_lut4))
sheet1.write(9,ColNumber,geomean(all_lut3))
sheet1.write(10,ColNumber,geomean(all_lut2))
sheet1.write(11,ColNumber,geomean(all_regs))
sheet1.write(12,ColNumber,geomean(all_mult9))
sheet1.write(13,ColNumber,geomean(all_equ_le))
#save the excel with the name which given by user
ColNumber = ColNumber + 1
sheet1.write(0,ColNumber,"sum")
sheet1.write(1,ColNumber,'n/a')
cycles_sum = sum(all_cycles)
sheet1.write(2,ColNumber,cycles_sum)
delay_sum = sum(all_run_time)
sheet1.write(3,ColNumber,delay_sum)
les_sum = sum(all_les)
sheet1.write(4,ColNumber,les_sum)
les_wo_reg_sum = sum(all_les_wo_reg)
sheet1.write(5,ColNumber,les_wo_reg_sum)
all_les_w_reg_only_sum = sum(all_les_w_reg_only)
sheet1.write(6,ColNumber,all_les_w_reg_only_sum)
all_les_and_reg_sum = sum(all_les_and_reg) 
sheet1.write(7,ColNumber,all_les_and_reg_sum)
all_lut4_sum = sum(all_lut4)
sheet1.write(8,ColNumber,all_lut4_sum)
all_lut3_sum = sum(all_lut3)
sheet1.write(9,ColNumber,all_lut3_sum)
all_lut2_sum = sum(all_lut2)
sheet1.write(10,ColNumber,all_lut2_sum)
all_regs_sum = sum(all_regs)
sheet1.write(11,ColNumber,all_regs_sum)
all_mult9_sum = sum(all_mult9)
sheet1.write(12,ColNumber,all_mult9_sum)
all_equ_le_sum = sum(all_equ_le)
sheet1.write(13,ColNumber,all_equ_le_sum)
print("Test results { freq_mean: %d total_cycles: %d total_time: %d total_les: %d total_mults: %d total_equ_les: %d delay-area: %d }" %(freq_mean,cycles_sum,delay_sum, les_sum, all_mult9_sum, all_equ_le_sum, all_equ_le_sum * delay_sum))
book.save(OutFile)	
