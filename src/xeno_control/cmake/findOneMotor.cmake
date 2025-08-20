
include(FetchContent)
find_package(OneMotor QUIET)

if (NOT OneMotor_FOUND)
    message(STATUS "本地未找到 OneMotor 包，尝试从 GitHub 获取...")
    FetchContent_Declare(
            OneMotor_fetched
            GIT_REPOSITORY "https://github.com/RoboMaster-DLMU-CONE/OneMotor"
            GIT_TAG "main"
    )
    FetchContent_MakeAvailable(OneMotor_fetched)
else ()
    message(STATUS "已找到 OneMotor 版本 ${OneMotor_VERSION}")
endif ()