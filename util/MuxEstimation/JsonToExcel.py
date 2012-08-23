import json
from xlwt import Workbook
import re

def getOthers(data):
    data=data.replace(',','')
    return float(re.match(r"^\d*",data).group(0))

book = Workbook()
InFile =  "report.json"
OutFile = "Timing.xls"
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)
sheet1 = book.add_sheet('Sheet 1')
sheet1.write(0,0,"name")
sheet1.write(0,1,"Input_Num") 
sheet1.write(0,2,"Bit_Width")
sheet1.write(0,3,"Total_LEs")
sheet1.write(0,4,"Delay")
RowNumber = 1
for data in json_read:
  sheet1.write(RowNumber,0,data["name"])
  sheet1.write(RowNumber,1,getOthers(data["Input_Num "]))
  sheet1.write(RowNumber,2,getOthers(data["Bit_Width "]))
  sheet1.write(RowNumber,3,getOthers(data["Total_LEs "]))
  sheet1.write(RowNumber,4,float(data["delay"]))
  RowNumber = RowNumber + 1
book.save(OutFile)  

book = Workbook()
InFile =  "Tmp.json"
OutFile = "LEs.xls"
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)
sheet1 = book.add_sheet('Sheet 1')
sheet1.write(0,0,"name")
sheet1.write(0,1,"Input_Num") 
sheet1.write(0,2,"Bit_Width")
sheet1.write(0,3,"Total_LEs")
RowNumber = 1
for data in json_read:
  sheet1.write(RowNumber,0,data["Name"])
  sheet1.write(RowNumber,1,getOthers(data["Input_Num"]))
  sheet1.write(RowNumber,2,getOthers(data["Width"]))
  sheet1.write(RowNumber,3,getOthers(data["Total_LEs"]))
  RowNumber = RowNumber + 1
book.save(OutFile)  
