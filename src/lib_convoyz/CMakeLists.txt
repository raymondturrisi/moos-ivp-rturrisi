#--------------------------------------------------------
# The CMakeLists.txt for:                      lib_convoy
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# enable C++11 (see top-level CMakeLists.txt for macro definition)
#use_cxx11()

SET(SRC
  ConvoyRecap.cpp
  ConvoyStatRecap.cpp
  ConvoySpdPolicy.cpp
  EvalConvoyEngine.cpp
  ConvoyOrderDetector.cpp
)

SET(HEADERS
  ConvoyRecap.h
  ConvoyStatRecap.h
  ConvoySpdPolicy.h
  EvalConvoyEngine.h
  ConvoyOrderDetector.h
)

# Build Library
ADD_LIBRARY(convoyz ${SRC})

