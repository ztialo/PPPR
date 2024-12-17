sudo apt-get install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Package libpng12-dev:armhf is not available, but is referred to by another package.
This may mean that the package is missing, has been obsoleted, or
is only available from another source

E: Package 'libpng12-dev:armhf' has no installation candidate


Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 apt : Depends: libapt-pkg6.0 (>= 2.6.1) but it is not going to be installed
       Depends: libstdc++6 (>= 11) but it is not installable
 libgcc-s1:armhf : Breaks: libgcc-s1 (!= 12.2.0-14+rpi1) but 12.2.0-14 is to be installed
 libgcc-s1 : Depends: gcc-12-base (= 12.2.0-14) but it is not installable
             Breaks: libgcc-s1:armhf (!= 12.2.0-14) but 12.2.0-14+rpi1 is to be installed
E: Error, pkgProblemResolver::Resolve generated breaks, this may be caused by held packages.

