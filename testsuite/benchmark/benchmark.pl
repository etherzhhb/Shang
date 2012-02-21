#!/usr/bin/perl
#
# USAGE: benchmark.pl --family <family> --key <key> --prefix <prefix>
#

use warnings;
use strict;
use Cwd;
use Data::Dumper;
use Getopt::Long;
use File::Basename;

# --family <family>
my $family = 'CycloneII';

# --key <key>
# This is passed by buildbot to indicate the type of graph we are collecting
# results for.
# ie. ./benchmark.pl LEs
# Will print out lines:
#   *RESULT all: dhrystone= 5165
#   *RESULT dhrystone: LEs= 5165
# The * indicates the result is 'important' and should be displayed on the buildbot waterfall
# The line must be formatted as:
#     <*>RESULT <graph_name>: <trace_name>= <value> <units>
my $performance_key = '';

# This is used to append a prefix to all result names:
# ie. ./benchmark.pl LEs tiger_
#   *RESULT tiger_all: tiger_dhrystone= 5165
#   *RESULT tiger_dhrystone: LEs= 5165
my $prefix = '';

# The list of benchmarks to parse.
my $benchmarkslist = '';

GetOptions ("family=s" => \$family,
            "key=s" => \$performance_key,
            "prefix=s" => \$prefix,
            "benchmarkslist=s" => \$benchmarkslist
           ) || die;

my $stratix = 0;
$stratix = 1 if ($family =~ /stratix/i);

my @benchmarks = split(' ',"$benchmarkslist");

#die unless scalar(@benchmarks) == 12;
print @benchmarks;

my $pwd = &Cwd::cwd();

my @metric_keys = qw(time cycles waitcycles Fmax LEs regs comb mults membits logicUtil);


my %metric_units = (
    'time' => 'us',
    'Fmax' => 'MHz',
);

my %allmetrics;
my %quartusWarnings;

foreach my $name (@benchmarks) {
    chdir $name or die "$!";

    $quartusWarnings{$name} = checkForWarnings($name);

    $allmetrics{$name} = parse($name);

    chdir $pwd or die "$!";
}

# calculate geomean
my %geomean;
foreach my $key (@metric_keys) {
    $geomean{$key} = 1;
}
my $count = 0;
foreach my $name (@benchmarks) {
    if ($allmetrics{$name}->{Fmax} ne "N/A") {
        $count++;
        foreach my $key (@metric_keys) {
            my $value = $allmetrics{$name}->{$key};
            # for 0 entries just assume 1. otherwise geomean will just equal 0.
            if ($value != 0) {
                $geomean{$key} *= $value;
            }
        }
    }
}
$count = 1 if ($count == 0); # avoid division by zero
foreach my $key (@metric_keys) {
    $geomean{$key} **= 1.0/$count;
}

foreach my $key (@metric_keys) {
    if (defined $performance_key && $key eq $performance_key) {
        my $value = $geomean{$key};
        if (defined $metric_units{$key}) {
            $value .= " $metric_units{$key}";
        }
        print "RESULT ".$prefix."all: ".$prefix."geomean= $value\n";
        print "RESULT ".$prefix."geomean: $prefix$key= $value\n";
    }
}


open(CSVFile, '>benchmark.csv') || die "Error: $!\n";
print CSVFile "name ";
foreach my $key (@metric_keys) {
    print CSVFile "$key ";
}
print CSVFile "\n";

open(JSONFile, '>benchmark.json') || die "Error: $!\n";

# JSON begin, an array containing the metrics.
print JSONFile "[\n";
my $isFirstObj = 1;

foreach my $name (@benchmarks) {
    # JSON object:
    # JSON object Name:
    print JSONFile ',' if ($isFirstObj == 0);
    $isFirstObj = 1;
    print JSONFile '{"'.$name.'" : {';
    print CSVFile "$name ";
    foreach my $key (@metric_keys) {
        print JSONFile ',' if ($isFirstObj == 0);
        print JSONFile '"'.$key.'" : "'.$allmetrics{$name}->{$key}."\"\n";
        print CSVFile $allmetrics{$name}->{$key}." ";
        $isFirstObj = 0;
    }
    print CSVFile "\n";
    # End object
    print JSONFile "} }\n";
    $isFirstObj = 0;
}
# JSON end.
print JSONFile "]\n";
close(JSONFile);
close(CSVFile);

print "Latex table (note: for Stratix LEs=ALMs, comb=ALUTs)\n";
printf "%-20s", "benchmark";
foreach my $key (@metric_keys) {
    printf " & %-10s", $key;
}
print " \\\\\n";

foreach my $name (@benchmarks) {
    printf "%-20s", $name;
    foreach my $key (@metric_keys) {
        printf " & %-10s", $allmetrics{$name}->{$key};
    }
    print " \\\\\n";
}
print "\n";

#print Dumper(\%allmetrics);
#print Dumper(\%geomean);
print "\nCycle geomean: $geomean{cycles}\nFmax geomean: $geomean{Fmax}\nLatency geomean: $geomean{time}\n";

print "\nContents of benchmark.csv:\n\n";
system("cat benchmark.csv");

# fail if quartus warnings were seen
my $fail = 0;
print "Checking for Quartus Warnings\n";
foreach my $key (%quartusWarnings) {
    if ($quartusWarnings{$key}) {
#        print "WARNING: $key\n";
#        print "Quartus Warnings:\n$quartusWarnings{$key}\n";
#        $fail = 0;
    }
}

die "Found Quartus warnings!\n" if ($fail);


sub round {
    my($number) = shift;
    return int($number + .5);
}

# parse the .rpt files for warnings
sub checkForWarnings {
    my $name = shift;
    my $testname = basename($name);
    $testname = $testname . '_DUT_RTL';
    my $warnings = '';

    # synthesis warnings:
    $warnings .= qx/grep -i "Inferred latch for" $testname.map.rpt/;
    $warnings .= qx/grep -i "truncated value with size" $testname.map.rpt/;
    $warnings .= qx/grep -i "assigned a value but never read" $testname.map.rpt/;

    # timing analysis warnings:
    $warnings .= qx/grep -i "Found combinational loop" $testname.sta.rpt/;

    return $warnings;
}

sub parse {
    my $name = shift;
    my $testname = basename($name);
    $testname = $testname . '_DUT_RTL';

    print STDERR "Can't find simulation cycles report $testname.txt for $name\n" unless (-f "$testname.txt");
    my $transcript = qx/grep "$testname " $testname.txt/;

    my $fmaxAll;
    if ($stratix) {
        # Stratix IV
        $fmaxAll = qx/grep --after-context=6 -i "Slow 900mV 0C Model Fmax Summary" $testname.sta.rpt|grep MHz/;
    } else {
        # Cyclone II
        $fmaxAll = qx/grep --after-context=6 -i "Slow Model Fmax Summary" $testname.sta.rpt|grep MHz/;
    }

    my $lines = qx/wc -l $testname.v|grep -v tb|grep -v total/;
    my $resources = qx/cat $testname.fit.summary/;

    # debugging
    if (0) {
        print $transcript;
        print $fmaxAll;
        print $lines;
        print $resources;
    }

    my %metrics;

    print "$name\n"; 
    print "------------\n"; 
    if ($transcript =~ /$testname hardware run cycles (\d+) wait cycles (\d+)/) {
        $metrics{cycles} = $1;
        $metrics{waitcycles} = $2;
    } else {
        $metrics{cycles} = 0;
        $metrics{waitcycles} = 0;
    }

    $metrics{Fmax} = 0;
    my $counter = 0;
    $counter++ while ($fmaxAll =~ /([\.\d]+) MHz.*/g);
    if ($counter > 1) {
        die "Found more than one Fmax!\nParsing:\n$fmaxAll";
    } elsif ($counter == 1) {
        $metrics{Fmax} = $1;
        print "Fmax: $metrics{Fmax} MHz\n";
    } else {
        print "Fmax: N/A\n";
    }

    print "Latency: $metrics{cycles} cycles\n";
    print "Wait cycles: $metrics{waitcycles} cycles\n";
    $metrics{'time'} = 0;
    if ($metrics{Fmax} > 0) {
        $metrics{'time'} = round($metrics{cycles} / $metrics{Fmax} );
        print "Latency: $metrics{time} us\n";
    } else {
        $metrics{Fmax} = "N/A";
        print "Latency: N/A\n";
    }

    if ($lines =~ /(\d+)/) {
        print "Verilog: $1 LOC\n";
    } else {
        print "Verilog: 0 LOC\n";
    }

    if ($resources =~ /(Family.*?)Total (GXB|PLL)/s) {
        print $1;
    }


    $resources =~ /Total registers : ([\d,]+)/;
    $metrics{regs} = $1;

    $metrics{LEs} = 0;

    # logic utilization metric meaningless for CycloneII
    $metrics{logicUtil} = 0;

    # for stratix LE = ALM, comb = ALUT
    if ($stratix) {
        $resources =~ /ALUTs : ([\d,]+)/;
        $metrics{comb} = $1;

        my $alm_raw = qx/grep "ALMs:" $testname.fit.rpt/;
        if ($alm_raw =~ /ALMs:\s+partially or completely used\s+; ([\d,]+)/) {
            $metrics{LEs} = $1;
        } 

        my $lu_raw = qx/grep "Logic utilization" $testname.fit.rpt/;
        if ($lu_raw =~ /Logic utilization\s+; ([\d,]+) \//) {
            $metrics{logicUtil} = $1;
        }

        $resources =~ /DSP block 18-bit elements : ([\d,]+)/;
        $metrics{mults} = $1;
    } else {
        # cyclone
        $resources =~ /Total combinational functions : ([\d,]+)/;
        $metrics{comb} = $1;

        $resources =~ /Total logic elements : ([\d,]+)/;
        $metrics{LEs} = $1;

        $resources =~ /Embedded Multiplier 9-bit elements : ([\d,]+)/;
        $metrics{mults} = $1;
    }


    $resources =~ /memory bits : ([\d,]+)/;
    $metrics{membits} = $1;

    print "\n";


    foreach my $key (@metric_keys) {
        $metrics{$key} =~ s/,//g;

        # Performance graphs
        if (defined $performance_key && $key eq $performance_key) {
            my $value = $metrics{$key};
            if (defined $metric_units{$key}) {
                $value .= " $metric_units{$key}";
            }
            # can't have a '/' in name:
            my $stripped = $name;
            $stripped =~ s/chstone\///;
            print "RESULT ".$prefix."all: $prefix$stripped= $value\n";
            print "RESULT $prefix$stripped: $prefix$key= $value\n";
        }
    }

    return \%metrics;
}
