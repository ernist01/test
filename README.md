Preparing metadata (pyproject.toml) ... error
  error: subprocess-exited-with-error
  
  × Preparing metadata (pyproject.toml) did not run successfully.
  │ exit code: 1
  ╰─> [61 lines of output]
      + /home/pi/rov/myenv1/bin/python3 /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/vendored-meson/meson/meson.py setup /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765 /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/.mesonpy-wpqzmlmu -Dbuildtype=release -Db_ndebug=if-release -Db_vscrt=md --native-file=/tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/.mesonpy-wpqzmlmu/meson-python-native-file.ini
      The Meson build system
      Version: 1.5.2
      Source dir: /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765
      Build dir: /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/.mesonpy-wpqzmlmu
      Build type: native build
      Project name: NumPy
      Project version: 2.2.6
      C compiler for the host machine: cc (gcc 12.2.0 "cc (Raspbian 12.2.0-14+rpi1+deb12u1) 12.2.0")
      C linker for the host machine: cc ld.bfd 2.40
      C++ compiler for the host machine: c++ (gcc 12.2.0 "c++ (Raspbian 12.2.0-14+rpi1+deb12u1) 12.2.0")
      C++ linker for the host machine: c++ ld.bfd 2.40
      Cython compiler for the host machine: cython (cython 3.1.2)
      Host machine cpu family: arm
      Host machine cpu: arm
      Program python found: YES (/home/pi/rov/myenv1/bin/python3)
      Found pkg-config: YES (/usr/bin/pkg-config) 1.8.1
      Run-time dependency python found: YES 3.11
      Has header "Python.h" with dependency python-3.11: YES
      Compiler for C supports arguments -fno-strict-aliasing: YES
      Message: During parsing cpu-dispatch: The following CPU features were ignored due to platform incompatibility or lack of support:
      "XOP FMA4"
      Test features "NEON" : Supported
      Test features "NEON_FP16" : Supported
      Test features "NEON_VFPV4" : Supported
      Test features "ASIMD" : Supported
      Test features "ASIMDHP" : Supported
      Test features "ASIMDFHM" : Supported
      Test features "SVE" : Unsupported due to Arguments "-mfp16-format=ieee, -mfpu=neon-fp-armv8, -march=armv8.2-a+sve+fp16+simd" are not supported
      Configuring npy_cpu_dispatch_config.h using configuration
      Message:
      CPU Optimization Options
        baseline:
          Requested : min
          Enabled   :
        dispatch:
          Requested : max -xop -fma4
          Enabled   : NEON NEON_FP16 NEON_VFPV4 ASIMD ASIMDHP ASIMDFHM
      
      Library m found: YES
      Run-time dependency scipy-openblas found: NO (tried pkgconfig)
      Found CMake: /usr/bin/cmake (3.25.1)
      WARNING: CMake Toolchain: Failed to determine CMake compilers state
      Run-time dependency openblas found: NO (tried pkgconfig, pkgconfig, pkgconfig, system and cmake)
      Run-time dependency flexiblas found: NO (tried pkgconfig and cmake)
      Run-time dependency blis found: NO (tried pkgconfig and cmake)
      Run-time dependency blas found: YES 3.10.3
      ../numpy/meson.build:135: WARNING: Project targets '>=1.2.99' but uses feature introduced in '1.3.0': dep 'blas' custom lookup.
      Message: BLAS symbol suffix:
      Run-time dependency openblas found: NO (tried pkgconfig, pkgconfig, pkgconfig, system and cmake)
      Run-time dependency flexiblas found: NO (tried pkgconfig and cmake)
      Run-time dependency lapack found: YES 3.10.3
      ../numpy/meson.build:198: WARNING: Project targets '>=1.2.99' but uses feature introduced in '1.3.0': dep 'lapack' custom lookup.
      Checking if "Check atomic builtins without -latomic" : links: YES
      Program _build_utils/process_src_template.py found: YES (/home/pi/rov/myenv1/bin/python3 /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/numpy/_build_utils/process_src_template.py)
      Program _build_utils/tempita.py found: YES (/home/pi/rov/myenv1/bin/python3 /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/numpy/_build_utils/tempita.py)
      Configuring __config__.py using configuration
      
      ../numpy/_core/meson.build:145:31: ERROR: Can not run test applications in this cross environment.
      
      A full log can be found at /tmp/pip-install-s4dk7wbb/numpy_f6feb3d3f71d42ccaa5a2488d5ff7765/.mesonpy-wpqzmlmu/meson-logs/meson-log.txt
      [end of output]
  
  note: This error originates from a subprocess, and is likely not a problem with pip.



