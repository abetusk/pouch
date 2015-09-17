#!/usr/bin/perl
#

my @a = split(/\n/, `cat /home/abram/bin/motd_faces.txt`);
my $l = scalar(@a);
my $k = rand($l);
print $a[$k], "\n";
