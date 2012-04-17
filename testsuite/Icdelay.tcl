set fileid [open "@IcDelayTmp@" a+]
set tmpslack 0

foreach_in_collection path [get_timing_paths -setup  -npath 8 -detail path_only] {
  set pathSlack [get_path_info $path -slack]
  #Only record the paths with a negative slack
  if {$pathSlack < $tmpslack} {
		set tmpslack $pathSlack
		set icdelay 0
    foreach_in_collection pt [ get_path_info $path -arrival_points ] {
      set total     [get_point_info $pt -total]
      set incr      [get_point_info $pt -incr]
      set type      [get_point_info $pt -type]
			if { $type == "ic" } {
				set icdelay [expr $icdelay + $incr]
			}
    }
  }
}

if {$tmpslack < 0} {
  set rate [expr $icdelay/$total]
  #Write in json file
  puts $fileid "\{\"netdelay\":"
  puts $fileid $icdelay
  puts $fileid ","
  puts $fileid "\"totaldelay\": "
  puts $fileid ","
  puts $fileid "\"net_total_rate\": "
  puts $fileid $rate
  puts $fileid "\}"
} else {
  puts $fileid "There is no fail path"
}

close $fileid
