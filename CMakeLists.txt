cmake_minimum_required(VERSION 3.22)
project(nvg_ins VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_library(inslib STATIC
  third_party/INSLIB/src/ins.c
  third_party/INSLIB/src/geodetic_toolbox.c
  third_party/INSLIB/src/magnetic_model.c
  third_party/INSLIB/src/ahrs.c
  third_party/INSLIB/src/baro_alt.c
  third_party/INSLIB/src/nav_suite.c
  third_party/INSLIB/src/log.c
  third_party/INSLIB/KFCore/c/linalg.c
  third_party/INSLIB/KFCore/c/kalman_udu.c
  third_party/INSLIB/KFCore/c/miniblas.c
)
target_include_directories(inslib PUBLIC
  third_party/INSLIB/src
  third_party/INSLIB/KFCore/c
)
target_link_libraries(inslib PUBLIC m)

add_library(ins_protocol STATIC
  generated/ins.pb.c
  third_party/nanopb/pb_common.c
  third_party/nanopb/pb_decode.c
  third_party/nanopb/pb_encode.c
)
target_include_directories(ins_protocol PUBLIC
  generated
  third_party/nanopb
)

add_library(nvg_ins_estimator STATIC src/estimator.cpp)
target_include_directories(nvg_ins_estimator PUBLIC src)
target_link_libraries(nvg_ins_estimator PUBLIC inslib ins_protocol)
target_compile_options(nvg_ins_estimator PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(nvg-ins src/main.cpp)
target_link_libraries(nvg-ins PRIVATE nvg_ins_estimator)
target_compile_options(nvg-ins PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)
if(BUILD_TESTING)
  add_library(ins_capi_oracle STATIC third_party/INSLIB/python/csrc/ins_capi.c)
  target_include_directories(ins_capi_oracle PUBLIC third_party/INSLIB/python/csrc)
  target_link_libraries(ins_capi_oracle PUBLIC inslib)

  add_executable(estimator-replay tests/estimator_replay.cpp)
  add_executable(covariance-oracle tests/covariance_oracle.cpp)
  add_executable(socket-smoke tests/socket_smoke.cpp)
  target_link_libraries(socket-smoke PRIVATE ins_protocol)
  target_compile_options(socket-smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(estimator-replay PRIVATE nvg_ins_estimator)
  target_compile_options(estimator-replay PRIVATE -Wall -Wextra -Wpedantic -Werror)
  target_link_libraries(covariance-oracle PRIVATE nvg_ins_estimator ins_capi_oracle)
  target_compile_options(covariance-oracle PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME estimator-replay COMMAND estimator-replay)
  add_test(NAME covariance-oracle COMMAND covariance-oracle)
endif()

install(TARGETS nvg-ins RUNTIME DESTINATION bin)
install(FILES packaging/nvg-ins.service DESTINATION lib/systemd/system)
