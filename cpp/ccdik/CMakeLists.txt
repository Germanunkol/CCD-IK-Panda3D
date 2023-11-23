
set(P3CCDIK_HEADERS
  ikChain.h
  ikJoint.h
  debug_utils.h
  vec_utils.h
)

set(P3CCDIK_SOURCES
  ikChain.cxx
  ikJoint.cxx
  debug_utils.cxx
  vec_utils.cxx
)

composite_sources(p3ccdik P3CCDIK_SOURCES)
add_library(p3ccdik ${P3CCDIK_HEADERS} ${P3CCDIK_SOURCES})
set_target_properties(p3ccdik PROPERTIES DEFINE_SYMBOL BUILDING_CCDIK)
target_link_libraries(p3ccdik panda
  PKG::OPENCV PKG::ARTOOLKIT)
set_target_properties(p3ccdik PROPERTIES CXX_EXCEPTIONS ON)
target_interrogate(p3ccdik ALL)

install(TARGETS p3ccdik
	EXPORT Ccdik COMPONENT Ccdik
  DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/panda3d
  ARCHIVE COMPONENT CcdikDevel)
install(FILES ${P3CCDIK_HEADERS} COMPONENT CcdikDevel DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/panda3d)

export_targets(Ccdik COMPONENT CcdikDevel)

