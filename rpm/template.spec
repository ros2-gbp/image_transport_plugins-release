%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-compressed-image-transport
Version:        2.5.3
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS compressed_image_transport package

License:        BSD
URL:            http://www.ros.org/wiki/image_transport_plugins
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-humble-cv-bridge
Requires:       ros-humble-image-transport
Requires:       ros-humble-ros-workspace
BuildRequires:  ros-humble-ament-cmake
BuildRequires:  ros-humble-cv-bridge
BuildRequires:  ros-humble-image-transport
BuildRequires:  ros-humble-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%description
Compressed_image_transport provides a plugin to image_transport for
transparently sending images encoded as JPEG or PNG.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
mkdir -p .obj-%{_target_platform} && cd .obj-%{_target_platform}
%cmake3 \
    -UINCLUDE_INSTALL_DIR \
    -ULIB_INSTALL_DIR \
    -USYSCONF_INSTALL_DIR \
    -USHARE_INSTALL_PREFIX \
    -ULIB_SUFFIX \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/humble" \
    -DAMENT_PREFIX_PATH="/opt/ros/humble" \
    -DCMAKE_PREFIX_PATH="/opt/ros/humble" \
    -DSETUPTOOLS_DEB_LAYOUT=OFF \
%if !0%{?with_tests}
    -DBUILD_TESTING=OFF \
%endif
    ..

%make_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%make_install -C .obj-%{_target_platform}

%if 0%{?with_tests}
%check
# Look for a Makefile target with a name indicating that it runs tests
TEST_TARGET=$(%__make -qp -C .obj-%{_target_platform} | sed "s/^\(test\|check\):.*/\\1/;t f;d;:f;q0")
if [ -n "$TEST_TARGET" ]; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
CTEST_OUTPUT_ON_FAILURE=1 \
    %make_build -C .obj-%{_target_platform} $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Mon Feb 10 2025 Kenji Brameld <kenjibrameld@gmail.com> - 2.5.3-1
- Autogenerated by Bloom

* Mon Feb 10 2025 Kenji Brameld <kenjibrameld@gmail.com> - 2.5.2-2
- Autogenerated by Bloom

* Mon Jul 22 2024 Kenji Brameld <kenjibrameld@gmail.com> - 2.5.2-1
- Autogenerated by Bloom

* Sat Apr 13 2024 Kenji Brameld <kenjibrameld@gmail.com> - 2.5.1-1
- Autogenerated by Bloom

* Tue Apr 19 2022 David Gossow <dgossow@willowgarage.com> - 2.5.0-2
- Autogenerated by Bloom

* Mon Apr 18 2022 David Gossow <dgossow@willowgarage.com> - 2.5.0-1
- Autogenerated by Bloom

* Fri Apr 08 2022 David Gossow <dgossow@willowgarage.com> - 2.4.0-1
- Autogenerated by Bloom

* Fri Feb 18 2022 David Gossow <dgossow@willowgarage.com> - 2.3.2-1
- Autogenerated by Bloom

* Tue Feb 08 2022 David Gossow <dgossow@willowgarage.com> - 2.3.1-2
- Autogenerated by Bloom

