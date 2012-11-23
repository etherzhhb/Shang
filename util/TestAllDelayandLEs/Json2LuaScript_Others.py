import json
from optparse import OptionParser

parser = OptionParser()
parser.add_option("-t", "--timing_report", dest="timing_report",
                  help="Timing report to read", metavar="FILE")
parser.add_option("-l", "--logic_element_report", dest="logic_element_report",
                  help="Logic element usage report to read", metavar="FILE")
parser.add_option("-o", "--output", dest="output",
                  help="The output LUAScript file", metavar="FILE")

(options, args) = parser.parse_args()

with open(options.timing_report,"r") as f:
    read_data = '['+f.read()[1:]+']'
f.closed
json_read = json.loads(read_data)

Writefile = open(options.output,'w')

# The FU dictionary
FUs = {}

Width8 = 0
Width16 = 0
Width32 = 0
Width64 = 0

for data in json_read:
  CurFUs = { data["Bit_Width"] : { "LEs" : data["Total_LEs"], "Delay" : data["delay"] } }
  CurFUs.update(FUs.get(data["name"], {}))
  FUs[data["name"]] = CurFUs
  print FUs

  if data["Bit_Width"] == "8":
    Width8 = data["delay"]
  if data["Bit_Width"] == "16":
    Width16 = data["delay"]
  if data["Bit_Width"] == "32":
    Width32 = data["delay"]
  if data["Bit_Width"] == "64":
    Width64 = data["delay"]

Writefile.write("{ %s / PERIOD, %s / PERIOD, %s / PERIOD, %s / PERIOD },  \n"%(Width8,Width16,Width32,Width64))

#Writefile.write("{%s , %s , %s , %s , %s}, \n"%(Width1*64,Width8*64,Width16*64,Width32*64,Width64*64))
###-----------------------------------------------------------------------------------------------------------###

Writefile.close()

