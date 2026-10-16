# This directory holds several independent programs, so it can't use ns-3's
# default "one program per scratch subdir" rule (which globs every .cc together
# and rejects more than one main()). Instead we build each program explicitly.
# Layout (files grouped by role, this CMake reaches into each subfolder):
#   harness/  : the thesis testbed (per-layer files linked into one `main`)
#   gates/    : standalone validation experiments, each its own main()
#   fail/     : archived dead ends (dual-channel/dual-role, superseded allocation), still buildable
#   nsga/     : NSGA-II backbone (added as it's built)
# Run as: ./ns3 run "mesh-slicing/main ..."  /  "mesh-slicing/formula-gate ..."

set(mesh_slicing_out ${CMAKE_OUTPUT_DIRECTORY}/scratch/mesh-slicing/)

# The thesis harness: main.cc + the per-layer source files, one executable.
build_exec(
  EXECNAME main
  EXECNAME_PREFIX scratch_mesh-slicing_
  SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/main.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/run-mode.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/topology.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/loss-ledger.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/warmup.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/internet.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/traffic-control.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/controller.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/harness/applications.cc
    # NSGA-II core (no nsga/main.cc — that's a second main()): the harness runs the
    # GA on the warm-up table, then injects its routes (BuildRoutes -> InjectStaticRoutes).
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/graph.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/fitness.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/seng.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/nsga2.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/routing.cc
    # OFDMA: the allocator turns measured capacity + declared demands into per-slice shares.
    ${CMAKE_CURRENT_SOURCE_DIR}/ofdma/allocator.cc
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
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/seng.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/nsga2.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/nsga/routing.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
)

# OFDMA gates: need the ofdma allocator linked in, so build them explicitly (not single-file).
foreach(ogate ofdma-gate)
  build_exec(
    EXECNAME ${ogate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/gates/${ogate}.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/ofdma/allocator.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()

# Gates that drive the harness's own L2.5 queue discs: they link harness/traffic-control.cc
# (InstallSliceQdiscs, SetSliceRates, SliceBandFilter) as well as the allocator.
foreach(qgate capacity-feedback-gate slice-qdisc-gate alloc-gate)
  build_exec(
    EXECNAME ${qgate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES
      ${CMAKE_CURRENT_SOURCE_DIR}/gates/${qgate}.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/harness/traffic-control.cc
      ${CMAKE_CURRENT_SOURCE_DIR}/ofdma/allocator.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()

# Standalone gate tests: each a single-file program with its own main().
foreach(gate formula-gate)
  build_exec(
    EXECNAME ${gate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/gates/${gate}.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()

# fail/ : the dual-channel / dual-role dead end (fail/DualChannel.md). Kept building so
# the measurements stay reproducible, but nothing in harness/, nsga/ or ofdma/ uses them.
# fail/ofdma-dynamic-gate: superseded rather than merely unwired. It senses DEMAND changing and
# retunes the SOURCE rate with R as a given constant; harness/controller.cc senses radio backlog
# and retunes the per-node BUCKET rates, which is the quantity that actually had to change. Needs
# the allocator linked, so it gets its own block.
build_exec(
  EXECNAME ofdma-dynamic-gate
  EXECNAME_PREFIX scratch_mesh-slicing_
  SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/fail/ofdma-dynamic-gate.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/ofdma/allocator.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
  EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
)

foreach(gate dual-role-gate hop-delay-gate star-ofdma-gate ring-bss-gate grid-dualrole-gate)
  build_exec(
    EXECNAME ${gate}
    EXECNAME_PREFIX scratch_mesh-slicing_
    SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/fail/${gate}.cc
    LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}"
    EXECUTABLE_DIRECTORY_PATH ${mesh_slicing_out}
  )
endforeach()
