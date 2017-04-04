#!/usr/bin/perl
#

my $user = $ENV{'USER'};

my @a = split(/\n/, `cat /home/$user/bin/motd_faces.txt`);
my $l = scalar(@a);
my $k = rand($l);
print $a[$k], "\n";
