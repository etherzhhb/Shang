#!/bin/bash
ExpectFailListPath=$1
CurrentFail=$2
TargetFailListPath=$3

while read line
do
  #If the fail is expected, simply ignore it.
  [ "$line" = "$CurrentFail" ] && exit 0
done < $ExpectFailListPath

# Append the fail test to the fail list.
echo "$CurrentFail"_diff_output `find $PWD -name $CurrentFail*.bc` >> $TargetFailListPath

