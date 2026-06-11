include(imstkFind)
#-----------------------------------------------------------------------------
# Find All Headers and Libraries for g3log
#-----------------------------------------------------------------------------

if(WIN32)
  set(postfix "d")
else()
  set(postfix " ") # Linux is ignoring the postfix request :(
endif()

if(NOT USE_SYSTEM_g3log)
  unset(g3log_INCLUDE_DIR CACHE)
  unset(g3log_LIBRARIES)
  unset(g3log_RELEASE_LIBRARIES)
  unset(g3log_DEBUG_LIBRARIES)
  unset(g3log_LIBRARY_g3logger-RELEASE CACHE)
  unset(g3log_LIBRARY_g3logger-DEBUG CACHE)
  unset(g3log_LIBRARY_g3log-RELEASE CACHE)
  unset(g3log_LIBRARY_g3log-DEBUG CACHE)
endif()

imstk_find_header(g3log g3log/g3log.hpp)
imstk_find_libary(g3log g3logger ${postfix})
if(NOT g3log_LIBRARIES)
  imstk_find_libary(g3log g3log ${postfix})
endif()
imstk_find_package(g3log g3log::g3log)

find_package(Threads QUIET)
if(Threads_FOUND)
  target_link_libraries(g3log::g3log INTERFACE Threads::Threads)
endif()

if(WIN32)
  target_link_libraries(g3log::g3log INTERFACE general dbghelp)
endif()

#message(STATUS "g3log include : ${g3log_INCLUDE_DIRS}")
#message(STATUS "g3log libraries : ${g3log_LIBRARIES}")
