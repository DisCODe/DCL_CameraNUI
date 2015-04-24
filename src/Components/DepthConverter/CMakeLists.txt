# Include the directory itself as a path to include directories
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# Create a variable containing all .cpp files:
FILE(GLOB files *.cpp)

# Find required packages
FIND_PACKAGE( OpenCV REQUIRED )

DISCODE_FIND_DCL(CvCoreTypes)

# Create an executable file from sources:
ADD_LIBRARY(DepthConverter SHARED ${files})

# Add aother libraries if necessary, for example:
TARGET_LINK_LIBRARIES(DepthConverter ${DisCODe_LIBRARIES} ${OpenCV_LIBS} )

INSTALL_COMPONENT(DepthConverter)
 
