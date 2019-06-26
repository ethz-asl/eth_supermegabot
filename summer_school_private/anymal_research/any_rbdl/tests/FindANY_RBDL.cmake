# Searches for ANY_RBDL includes and library files, including Addons.
#
# Sets the variables
#   ANY_RBDL_FOUND
#   ANY_RBDL_INCLUDE_DIR
#   ANY_RBDL_LIBRARY
#
# You can use the following components:
#   LuaModel
#   URDFReader
# and then link to them e.g. using ANY_RBDL_LuaModel_LIBRARY.

SET (ANY_RBDL_FOUND FALSE)
SET (ANY_RBDL_LuaModel_FOUND FALSE)
SET (ANY_RBDL_URDFReader_FOUND FALSE)

FIND_PATH (ANY_RBDL_INCLUDE_DIR any_rbdl/rbdl.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	)

FIND_LIBRARY (ANY_RBDL_LIBRARY NAMES any_rbdl
	PATHS
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}/lib
	$ENV{RBDL_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	)

FIND_PATH (ANY_RBDL_LuaModel_INCLUDE_DIR any_rbdl/addons/luamodel/luamodel.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	)

FIND_LIBRARY (ANY_RBDL_LuaModel_LIBRARY NAMES any_rbdl_luamodel
	PATHS
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	)

FIND_PATH (ANY_RBDL_URDFReader_INCLUDE_DIR any_rbdl/addons/urdfreader/urdfreader.h
	HINTS
	$ENV{HOME}/local/include
	$ENV{RBDL_PATH}/src
	$ENV{RBDL_PATH}/include
	$ENV{RBDL_INCLUDE_PATH}
	/usr/local/include
	/usr/include
	)

FIND_LIBRARY (ANY_RBDL_URDFReader_LIBRARY NAMES any_rbdl_urdfreader
	PATHS
	$ENV{HOME}/local/lib
	$ENV{HOME}/local/lib/x86_64-linux-gnu
	$ENV{RBDL_PATH}
	$ENV{RBDL_LIBRARY_PATH}
	/usr/local/lib
	/usr/local/lib/x86_64-linux-gnu
	/usr/lib
	/usr/lib/x86_64-linux-gnu
	)

IF (NOT ANY_RBDL_LIBRARY)
	MESSAGE (ERROR "Could not find ANY_RBDL")
ENDIF (NOT ANY_RBDL_LIBRARY)

IF (ANY_RBDL_INCLUDE_DIR AND ANY_RBDL_LIBRARY)
	SET (ANY_RBDL_FOUND TRUE)
ENDIF (ANY_RBDL_INCLUDE_DIR AND ANY_RBDL_LIBRARY)

IF (ANY_RBDL_LuaModel_INCLUDE_DIR AND ANY_RBDL_LuaModel_LIBRARY)
	SET (ANY_RBDL_LuaModel_FOUND TRUE)
ENDIF (ANY_RBDL_LuaModel_INCLUDE_DIR AND ANY_RBDL_LuaModel_LIBRARY)

IF (ANY_RBDL_URDFReader_INCLUDE_DIR AND ANY_RBDL_URDFReader_LIBRARY)
	SET (ANY_RBDL_URDFReader_FOUND TRUE)
ENDIF (ANY_RBDL_URDFReader_INCLUDE_DIR AND ANY_RBDL_URDFReader_LIBRARY)

IF (ANY_RBDL_FOUND)
   IF (NOT ANY_RBDL_FIND_QUIETLY)
	  MESSAGE(STATUS "Found ANY_RBDL: ${ANY_RBDL_LIBRARY}")
   ENDIF (NOT ANY_RBDL_FIND_QUIETLY)

	 foreach ( COMPONENT ${ANY_RBDL_FIND_COMPONENTS} )
		 IF (ANY_RBDL_${COMPONENT}_FOUND)
			 IF (NOT ANY_RBDL_FIND_QUIETLY)
				 MESSAGE(STATUS "Found ANY_RBDL ${COMPONENT}: ${ANY_RBDL_${COMPONENT}_LIBRARY}")
			 ENDIF (NOT ANY_RBDL_FIND_QUIETLY)
		 ELSE (ANY_RBDL_${COMPONENT}_FOUND)
			 MESSAGE(SEND_ERROR "Could not find ANY_RBDL ${COMPONENT}")
		 ENDIF (ANY_RBDL_${COMPONENT}_FOUND)
	 endforeach ( COMPONENT )
ELSE (ANY_RBDL_FOUND)
   IF (ANY_RBDL_FIND_REQUIRED)
		 MESSAGE(SEND_ERROR "Could not find ANY_RBDL")
   ENDIF (ANY_RBDL_FIND_REQUIRED)
ENDIF (ANY_RBDL_FOUND)

MARK_AS_ADVANCED (
	ANY_RBDL_INCLUDE_DIR
	ANY_RBDL_LIBRARY
	ANY_RBDL_LuaModel_INCLUDE_DIR
	ANY_RBDL_LuaModel_LIBRARY
	ANY_RBDL_URDFReader_INCLUDE_DIR
	ANY_RBDL_URDFReader_LIBRARY
	)
