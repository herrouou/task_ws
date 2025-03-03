# check and correct python codes
autopep8 --in-place --aggressive --aggressive -r src/sine_wave_py src/sine_wave_cpp
ament_pycodestyle src/sine_wave_py src/sine_wave_cpp

# clang format for C++
find src/sine_wave_cpp src/sine_wave_py -type f \( -name "*.cpp" -o -name "*.hpp" -o -name "*.c" -o -name "*.h" \) -exec clang-format -i {} +
