TEMPLATE = app
CONFIG += console
CONFIG -= app_bundle
#CONFIG -= qt

QMAKE_CXXFLAGS += -std=gnu++14

QMAKE_CXXFLAGS += -fopenmp
LIBS += -fopenmp

DESTDIR += ./bin
OBJECTS_DIR += ./lib

HEADERS += \
    getalignresults.h \
    naive.h \
    settings.h \
    rayint/acc/acceleration.h \
    rayint/acc/bvh_tree.h \
    rayint/acc/defines.h \
    rayint/acc/kd_tree.h \
    rayint/acc/primitives.h \
    rayint/math/algo.h \
    rayint/math/defines.h \
    rayint/math/vector.h \
    shader.hpp \
    texture.hpp \
    texture.hpp \

SOURCES += main.cpp \
    getalignresults.cpp \
    mainNaive.cpp \
    Naive.cpp \
    shader.cpp \
    texture.cpp \
    TextureFragmentShader.fragmentshader \
    TransformVertexShader.vertexshader \

INCLUDEPATH += ./rayint

INCLUDEPATH += lib
LIBS += ~/ivan/git/EAGLE-TextureMapping_modifying/lib/libEagle_Utils.so

INCLUDEPATH += /usr/include/eigen3/Eigen
LIBS += -lboost_filesystem -lboost_system

INCLUDEPATH += /usr/local/include/opencv2
#-lopencv_world
LIBS += -L"/usr/local/lib" -lopencv_core -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc

INCLUDEPATH += /usr/include/vtk-7.1 /usr/include/pcl
LIBS += -L"/usr/lib" -lpcl_common -lpcl_io -lpcl_io_ply

INCLUDEPATH += /usr/local/include/ImageMagick-7
