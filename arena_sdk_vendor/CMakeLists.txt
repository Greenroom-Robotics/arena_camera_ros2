cmake_minimum_required(VERSION 3.5)

project(arena_sdk_vendor)

find_package(ament_cmake REQUIRED)

set(lib64_libs
  libsavec.so.0
  libsavec.so.0.1.59
  libarenacd.so.0
  libarenacd.so.0.1.59
  libarenac.so
  libarenac.so.0.1.59
  libgentld.so
  libgentld.so.0.1.59
  libgentl.so
  libgentl.so.0.1.59
  liblucidlog.so
  liblucidlog.so.0.1.59
  libsavecd.so
  libsavecd.so.0.1.59
  libsave.so
  libsave.so.0.1.59
  libarenacd.so
  libarenad.so.0
  libarenad.so.0.1.59
  libsavecd.so.0
  libsaved.so.0
  libsaved.so.0.1.59
  libarenac.so.0
  libgentld.so.0
  libgentl.so.0
  libsaved.so
  liblucidlog.so.0
  libsavec.so
  libarena.so.0
  libarena.so.0.1.59
  libarena.so
  liblucidlogd.so.0.1.59
  libsave.so.0
  liblucidlogd.so
  libarenad.so
  liblucidlogd.so.0
)

set(genicam_libs
  libMathParser_gcc54_v3_3_LUCID.so
  libGenApi_gcc54_v3_3_LUCID.so
  libNodeMapData_gcc54_v3_3_LUCID.so
  libXmlParser_gcc54_v3_3_LUCID.so
  liblog4cpp_gcc54_v3_3_LUCID.so
  libLog_gcc54_v3_3_LUCID.so
  libGCBase_gcc54_v3_3_LUCID.so
)

macro(unpack_arenasdk)
  include(ExternalProject)

  # download the SDK from the Lucid website # URL https://thinklucid.com/product/arena-sdk-linux-64-bit-44-mb/

  externalproject_add(arenasdk-bin-dist
    URL file://${CMAKE_CURRENT_SOURCE_DIR}/ArenaSDK_v0.1.59_Linux_x64.tar.gz
    URL_HASH SHA256=e268e487a7ea3e7aa3cfe6241ab881ded7dc1ec9ab87e880b76372c7e52b660e
    DOWNLOAD_NAME ArenaSDK_v0.1.59_Linux_x64.tar.gz
    DOWNLOAD_DIR ${CMAKE_CURRENT_SOURCE_DIR}
    BUILD_IN_SOURCE True
    SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/src/
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src/ArenaSDK_Linux_x64/include/
    DESTINATION include
  )

  install(
    DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/src/ArenaSDK_Linux_x64/GenICam/library/CPP/include/
    DESTINATION include
  )

  foreach(L ${lib64_libs} )
    set(arenasdk_libs ${arenasdk_libs} ${CMAKE_CURRENT_BINARY_DIR}/src/ArenaSDK_Linux_x64/lib64/${L})
  endforeach(L)

  foreach(L ${genicam_libs} )
    set(arenasdk_libs ${arenasdk_libs} ${CMAKE_CURRENT_BINARY_DIR}/src/ArenaSDK_Linux_x64/GenICam/library/lib/Linux64_x64/${L})
  endforeach(L)

  install(
    FILES ${arenasdk_libs}
    DESTINATION lib
  )

  install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/src/ArenaSDK_Linux_x64/OutputDirectory/Linux/x64Release/IpConfigUtility
    DESTINATION bin
  )

endmacro()

unpack_arenasdk()

ament_export_include_directories(include)
ament_export_libraries(${lib64_libs} ${genicam_libs})
# this ensures that the package has an environment hook setting the PATH
ament_package()
