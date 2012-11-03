#! /usr/bin/python

import re
import sys

KeyWord = sys.argv[1]

StatsSum = 0

RegexKeyWord = re.compile(KeyWord)
ResultStr = KeyWord

for line in sys.stdin:
    #Get the line that we are interested in.
    if RegexKeyWord.search(line) :
        # Extract the number from the Stats line.
        StatsNum = int(re.match(r".*\d+", line).group())
  #      print '+',  StatsNum
        StatsSum += StatsNum
#        print StatsSum
        
        ResultStr += '\t' + str(StatsNum)
 #   elif re.search(r"High-levle Synthesising",  line)  :
  #      print line

# print StatsSum,  KeyWord
print ResultStr
