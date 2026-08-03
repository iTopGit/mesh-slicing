# This directory holds several independent programs, so it can't use ns-3's
# default "one program per scratch subdir" rule (which globs every .cc together
# and rejects more than one main()). Instead we build each program explicitly:
#   - main         : the thesis harness (layer files linked together)
#   - the gate/*   : standalone validation experiments, each its own main()
# Run as: ./ns3 run "mesh-slicing/main ..."  /  "mesh-slicing/formula-gate ..."

set(mesh_slicing_out ${CMAKE_OUTPUT_DIRECTORY}/scratch/mesh-slicing/)

# The thesis harness: main.cc + the per-layer source files, one executable.
build_exec(
  EXECNAME main
  EXECNAME_PREFIX scratch_mesh-slicing_
  SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/topology.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/warmup.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/internet.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/applications.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/traffic-control.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
)

# Standalone gate tests: each a single-file program with its own main().
foreach(gate adhoc-ax-test formula-gate slice-shape-gate)
  build_exec(
    EXECNAME ${gate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${gate}.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()
