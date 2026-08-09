# This directory holds several independent programs, so it can't use ns-3's
# default "one program per scratch subdir" rule (which globs every .cc together
# and rejects more than one main()). Instead we build each program explicitly.
# Layout (files grouped by role, this CMake reaches into each subfolder):
#   harness/  : the thesis testbed (per-layer files linked into one `main`)
#   gates/    : standalone validation experiments, each its own main()
#   nsga/     : NSGA-II backbone (added as it's built)
# Run as: ./ns3 run "mesh-slicing/main ..."  /  "mesh-slicing/formula-gate ..."

set(mesh_slicing_out ${CMAKE_OUTPUT_DIRECTORY}/scratch/mesh-slicing/)

# The thesis harness: main.cc + the per-layer source files, one executable.
build_exec(
  EXECNAME main
  EXECNAME_PREFIX scratch_mesh-slicing_
  SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/topology.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/warmup.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/internet.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/applications.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/traffic-control.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
)

# NSGA-II backbone: pure-C++ GA (graph + decoder/fitness + machinery) + driver.
build_exec(
  EXECNAME nsga
  EXECNAME_PREFIX scratch_mesh-slicing_
  SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/graph.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/fitness.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/nsga2.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
)

# Standalone gate tests: each a single-file program with its own main().
foreach(gate adhoc-ax-test formula-gate slice-shape-gate encoding-gate)
  build_exec(
    EXECNAME ${gate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gates/${gate}.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()
