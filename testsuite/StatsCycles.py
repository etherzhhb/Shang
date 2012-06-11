#! /usr/bin/python

import json
from xlwt import Workbook
import re
import sys

def geomean(nums):
    return (reduce(lambda x, y: x*y, nums))**(1.0/len(nums))
def append(list, data):
    if data != 0 :
        list.append(data)
    return
book = Workbook()
InFile = sys.argv[1]
OutFile = sys.argv[2]
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
readdate = json.loads(read_data)
sheet1 = book.add_sheet('Sheet 1')
sheet1.write(0,0,"name")
sheet1.write(1,0,"total")
sheet1.write(2,0,"wait")
sheet1.write(3,0,"wait_ratio")
all_total = []
all_waits = []
wait_ratios = []
ColNumber = 1
for data in readdate:	
      sheet1.write(0,ColNumber,data["name"])
      #Total
      total = data["total"]
      sheet1.write(1,ColNumber,total)
      #Wait
      wait = data["wait"]
      sheet1.write(2,ColNumber,wait)
      append(all_total, total)
      append(all_waits, wait)
      wait_ratio = float(wait) / float(total)
      sheet1.write(3, ColNumber, wait_ratio)
      append(wait_ratios, wait_ratio)
      ColNumber = ColNumber + 1
sheet1.write(0,ColNumber,"geomean")
sheet1.write(1,ColNumber,geomean(all_total))
sheet1.write(2,ColNumber,geomean(all_waits))
sheet1.write(3,ColNumber,geomean(wait_ratios))

ColNumber = ColNumber + 1
sheet1.write(0,ColNumber,"sum")
total_sum = sum(all_total)
sheet1.write(1,ColNumber,total_sum)
total_wait = sum(all_waits)
sheet1.write(2,ColNumber,total_wait)
total_wait_ratio = float(total_wait) / float(total_sum)
sheet1.write(3,ColNumber,total_wait_ratio)
if  total_wait <= 12:
  print("Test result { total_cycles: %d}" %(total_sum))
else:
  print("Test result { total_cycles: %d total_wait: %d total_wait_ratio: %f }" %(total_sum,total_wait,total_wait_ratio))
book.save(OutFile)
