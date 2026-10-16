# Incessus

Incessus is a C++23 SKSE plugin project. Its CMake configuration uses CommonLibSSE and the vcpkg manifest declares the project dependencies.

## Source and SDK

- Plugin implementation: `src/`
- Public headers: `include/` and `sdk/include/`
- SDK integration notes: [`sdk/README.md`](sdk/README.md)

## Build

Requirements and dependencies are declared in `CMakeLists.txt`, `CMakePresets.json`, and the vcpkg manifest files. Configure with the appropriate vcpkg toolchain and build the selected CMake preset for your Skyrim runtime.

## License

No license has been selected for this repository yet. Public visibility alone does not grant permission to reuse or redistribute the code.
