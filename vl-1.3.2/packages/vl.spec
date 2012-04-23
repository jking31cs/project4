Name: vl
Version: 1.3.2
Release: 1
Summary: Vector Library
Source: ftp.cs.cmu.edu:/afs/cs/user/ajw/public/dist/vl-%{version}.tar.gz
Copyright: BSD-like
Group: Libraries
BuildRoot: /var/tmp/%{name}-root
Prefix: /usr
URL: http://www.cs.cmu.edu/~ajw/software/

%description 
VL is a C++ vector library, oriented towards graphics use, having optimized
2-, 3- and 4-vectors as well as generic vectors and matrices, sparse vectors
and matrices, inline mathematic operations, and lots of other stuff.

%prep

%setup -n vl-%{version}
make linux_RH

%build
make

%install
rm -rf $RPM_BUILD_ROOT
make DEST=$RPM_BUILD_ROOT/usr install

%clean
rm -rf $RPM_BUILD_ROOT

%files
%doc README LICENSE doc/vl.html
/usr/include/vl/*
/usr/include/cl/*
/usr/lib/*
