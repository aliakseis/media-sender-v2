cmake_minimum_required(VERSION 3.7)


find_package(PkgConfig)

pkg_check_modules(GSTREAMER REQUIRED 
  glib-2.0
  json-glib-1.0
  libsoup-2.4
  gstreamer-1.0 
  gstreamer-sdp-1.0
  gstreamer-rtp-1.0
  gstreamer-app-1.0
)

message("GSTREAMER_INCLUDE_DIRS: ${GSTREAMER_INCLUDE_DIRS}")

message("GSTREAMER_LIBRARY_DIRS: ${GSTREAMER_LIBRARY_DIRS}")


link_directories(${GSTREAMER_LIBRARY_DIRS})



find_path(PDC_INCLUDES curses.h)
find_library(PDC_LIBS pdcurses REQUIRED)
#find_library(pdcurses pdcurses REQUIRED)

set(MEDIA_UWP_RESOURCES
	uwp/Logo.png
	uwp/package.appxManifest
	uwp/SmallLogo.png
	uwp/SmallLogo44x44.png
	uwp/SplashScreen.png
	uwp/StoreLogo.png
	uwp/Windows_TemporaryKey.pfx
)

if(CMAKE_SYSTEM_NAME STREQUAL "WindowsStore")
	add_executable(datachannel-media-sender-v2 main.cpp cameraman.cpp ${MEDIA_UWP_RESOURCES})
else()
	add_executable(datachannel-media-sender-v2 main.cpp cameraman.cpp)
endif()

set_target_properties(datachannel-media-sender-v2 PROPERTIES
    CXX_STANDARD 17
    OUTPUT_NAME media-sender-v2)

set_target_properties(datachannel-media-sender-v2 PROPERTIES
	XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER com.github.paullouisageneau.libdatachannel.examples.media-sender-v2)

target_include_directories(datachannel-media-sender-v2  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include ${GSTREAMER_INCLUDE_DIRS}  ${PDC_INCLUDES})


find_package(Threads REQUIRED)
find_package(LibDataChannel REQUIRED)
target_link_libraries(datachannel-media-sender-v2 LibDataChannel::LibDataChannel Threads::Threads nlohmann_json  ${GSTREAMER_LIBRARIES} ${PDC_LIBS} )
#target_link_libraries(datachannel-media-sender-v2 LibDataChannel::LibDataChannel Threads::Threads nlohmann_json  ${GSTREAMER_LIBRARIES} pdcurses)

target_compile_options(datachannel-media-sender-v2  PUBLIC ${GSTREAMER_CFLAGS_OTHER})


if(MSVC)
	add_custom_command(TARGET datachannel-media-sender-v2 POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"$<TARGET_FILE_DIR:datachannel>/datachannel.dll"
		$<TARGET_FILE_DIR:datachannel-media-sender-v2>
	)
endif()
