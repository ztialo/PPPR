sudo apt-get install build-essential cmake pkg-config
Reading package lists... Done
Building dependency tree... Done
Reading state information... Done
build-essential is already the newest version (12.9).
pkg-config is already the newest version (1.8.1-1).
Some packages could not be installed. This may mean that you have
requested an impossible situation or if you are using the unstable
distribution that some required packages have not yet been created
or been moved out of Incoming.
The following information may help to resolve the situation:

The following packages have unmet dependencies:
 apt : Depends: adduser but it is not going to be installed
       Depends: gpgv or
                gpgv2 but it is not going to be installed or
                gpgv1
       Depends: libapt-pkg6.0 (>= 2.6.1) but it is not going to be installed
       Depends: libgnutls30 (>= 3.7.5) but it is not installable
       Depends: libsystemd0 but it is not going to be installed
 bsdutils : PreDepends: libsystemd0 but it is not going to be installed
 cmake:armhf : Depends: procps:armhf
               Depends: libgcc-s1:armhf (>= 3.5) but it is not installable
               Depends: libstdc++6:armhf (>= 9) but it is not installable
               Recommends: gcc:armhf but it is not installable
               Recommends: make:armhf
 coreutils : PreDepends: libacl1 (>= 2.2.23) but it is not installable
 dpkg : PreDepends: libbz2-1.0 but it is not installable
 dpkg-dev : Depends: perl:any
            Recommends: gnupg but it is not going to be installed or
                        sq but it is not installable or
                        sqop but it is not installable or
                        pgpainless-cli but it is not installable or
                        sequoia-chameleon-gnupg but it is not installable
            Recommends: gpgv or
                        sq but it is not installable or
                        sqop but it is not installable or
                        pgpainless-cli but it is not installable or
                        sequoia-chameleon-gnupg but it is not installable
            Recommends: libalgorithm-merge-perl but it is not going to be installed
 init : PreDepends: systemd-sysv or
                    sysvinit-core but it is not installable
 libarchive13:armhf : Depends: liblzma5:armhf (>= 5.2.2) but it is not installable
                      Depends: libzstd1:armhf (>= 1.4.0) but it is not installable
 libc6:armhf : Depends: libgcc-s1:armhf but it is not installable
 libcurl4:armhf : Depends: libgssapi-krb5-2:armhf (>= 1.17) but it is not installable
 libdpkg-perl : Depends: perl:any
                Recommends: libfile-fcntllock-perl but it is not going to be installed
 libexpat1:armhf : Depends: libgcc-s1:armhf (>= 3.5) but it is not installable
 libffi7:armhf : Depends: libgcc-s1:armhf (>= 3.5) but it is not installable
 libgl1 : Depends: libglx0 (= 1.6.0-1) but it is not going to be installed
 libglx-mesa0 : Depends: libexpat1 (>= 2.0.1) but it is not installable
                Depends: libgl1-mesa-dri but it is not going to be installed
 libgnutls30:armhf : Depends: libgmp10:armhf (>= 2:6.0.0) but it is not installable
 libhogweed6:armhf : Depends: libgmp10:armhf (>= 2:6.1.0) but it is not installable
 libicu67:armhf : Depends: libgcc-s1:armhf (>= 3.5) but it is not installable
                  Depends: libstdc++6:armhf (>= 5.2) but it is not installable
 libjsoncpp24:armhf : Depends: libgcc-s1:armhf (>= 3.5) but it is not installable
                      Depends: libstdc++6:armhf (>= 5.2) but it is not installable
 librtmp1:armhf : Depends: libgmp10:armhf but it is not installable
 libxml2:armhf : Depends: liblzma5:armhf (>= 5.1.1alpha+20120614) but it is not installable
 login : PreDepends: libpam-runtime but it is not going to be installed
         PreDepends: libpam-modules but it is not going to be installed
 python3 : PreDepends: python3-minimal (= 3.11.2-1+b1) but it is not going to be installed
           Depends: python3.11 (>= 3.11.2-1~) but it is not going to be installed
           Depends: libpython3-stdlib (= 3.11.2-1+b1) but it is not going to be installed
 sed : PreDepends: libacl1 (>= 2.2.23) but it is not installable
 tar : PreDepends: libacl1 (>= 2.2.23) but it is not installable
 util-linux : PreDepends: libsystemd0 but it is not going to be installed
E: Error, pkgProblemResolver::Resolve generated breaks, this may be caused by held packages.
