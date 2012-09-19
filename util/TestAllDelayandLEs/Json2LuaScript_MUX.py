import json
InFile =  "report_timing_MUX.json"
OutFile = "MUX_latency.lua"
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)

Writefile = open(OutFile,'w')
Width8 = 0
Width16 = 0
Width32 = 0
Width64 = 0
inputnum = 2
while inputnum <= 256:
 for data in json_read:
   if int(data["Input_Num "]) == inputnum :
     if data["Bit_Width "] == "8":
       Width8 = data["delay"]
     if data["Bit_Width "] == "16":
       Width16 = data["delay"]
     if data["Bit_Width "] == "32":
       Width32 = data["delay"]
     if data["Bit_Width "] == "64":
       Width64 = data["delay"]
 Writefile.write("{ %s / PERIOD, %s / PERIOD, %s / PERIOD, %s / PERIOD }, --%d-input \n"%(Width8,Width16,Width32,Width64,inputnum))
 inputnum = inputnum + 1
###-----------------------------------------------------------------------------------------------------------###
import re
def getOthers(data):
    data=data.replace(',','')
    return int(re.match(r"^\d*",data).group(0))
  
InFile =  "report_LEs_MUX.json"
with open(InFile,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)
Width8 = 0
Width16 = 0
Width32 = 0
Width64 = 0
Width1 = 0
inputnum = 2
while inputnum <= 256:
 for data in json_read:
   if int(data["Input_Num"]) == inputnum :
     if data["Width"] == "1":
       Width1 = getOthers(data["Total_LEs"])
     if data["Width"] == "8":
       Width8 = getOthers(data["Total_LEs"])
     if data["Width"] == "16":
       Width16 = getOthers(data["Total_LEs"])
     if data["Width"] == "32":
       Width32 = getOthers(data["Total_LEs"])
     if data["Width"] == "64":
       Width64 = getOthers(data["Total_LEs"])
 Writefile.write("{%s , %s , %s , %s , %s},--%d-input \n"%(Width1*64,Width8*64,Width16*64,Width32*64,Width64*64,inputnum))
 inputnum = inputnum + 1

Writefile.close() 
