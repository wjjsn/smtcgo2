安装依赖：`conan.exe install . --build=missing --output-folder=build -c tools.cmake.cmaketoolchain:generator=Ninja`

配置：`pwsh.exe -c './env.ps1 cmake --preset conan-release'`

编译：`pwsh.exe -c './env.ps1 cmake --build --preset conan-release --parallel'`