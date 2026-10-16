cmake_minimum_required(VERSION 3.21)

project(Incessus VERSION 0.4.0 LANGUAGES CXX)

find_package(CommonLibSSE CONFIG REQUIRED)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_commonlibsse_plugin(${PROJECT_NAME} SOURCES
    src/main.cpp

    src/Hooks.cpp
    src/Hooks.h

    src/SyntheticMotionInjector.cpp
    src/SyntheticMotionInjector.h

    src/SteeringIntentController.cpp
    src/SteeringIntentController.h

    src/LocomotionSDK.cpp
    src/LocomotionSDK.h

    src/AnimationMotionDatabase.cpp
    src/AnimationMotionDatabase.h

    src/AnimationPriority.cpp
    src/AnimationPriority.h

    src/CapsuleSlerp.cpp
    src/CapsuleSlerp.h

    src/MotionProfileDatabase.cpp
    src/MotionProfileDatabase.h

    src/MotionTrajectoryController.cpp
    src/MotionTrajectoryController.h

    src/RotationIntentTranslator.cpp
    src/RotationIntentTranslator.h

    src/SpeedIntentTranslator.cpp
    src/SpeedIntentTranslator.h

    src/TranslationAuthority.h
    src/NPCLockOnBreaker.h

    src/PCH.cpp
    src/PCH.h
)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_23)

target_precompile_headers(${PROJECT_NAME} PRIVATE
    src/PCH.h
)

target_include_directories(${PROJECT_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    ALR_SDK_EXPORTS=1
)

