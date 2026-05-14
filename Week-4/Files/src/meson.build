# meson build alternative to the Makefile.
#
# Usage (on target):
#   cd src && meson setup build && meson compile -C build 
#
# Produces: src/build/kyocera_pmd_init

project('kyocera_pmd_init', 'c',       
    version: '0.1',
    default_options: ['c_std=c11', 'warning_level=2', 'optimization=3'])

dpdk_dep = dependency('libdpdk', version: '>=22.11', required: true)

executable('kyocera_pmd_init',
    sources: ['main.c'],
    dependencies: [dpdk_dep],
    install: false)
  
