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

for data in json_read:
  CurFUs = { data["Bit_Width"] : { "LEs" : data["Total_LEs"], "Delay" : data["delay"] } }
  CurFUs.update(FUs.get(data["name"], {}))
  FUs[data["name"]] = CurFUs


for name, bits in FUs.iteritems() :
  print "FUs.%s = {" % name
  print "\tLatencies = { %s / PERIOD, %s / PERIOD, %s / PERIOD, %s / PERIOD, %s / PERIOD }, --%s " % (bits['1']['Delay'],bits['8']['Delay'],bits['16']['Delay'],bits['32']['Delay'],bits['64']['Delay'], name)
  print "\tCosts = {%s * 64, %s * 64, %s * 64, %s * 64, %s * 64}, --%s " % (bits['1']['LEs'] , bits['8']['LEs'], bits['16']['LEs'], bits['32']['LEs'], bits['64']['LEs'], name)


Writefile.close()

