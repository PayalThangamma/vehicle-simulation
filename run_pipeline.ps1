Write-Host "========================================="
Write-Host "Vehicle Simulation Factory Pipeline"
Write-Host "========================================="

$python = "C:\Users\payal\AppData\Local\Programs\Python\Python312\python.exe"
$cmake = "C:\msys64\ucrt64\bin\cmake.exe"
$ctest = "C:\msys64\ucrt64\bin\ctest.exe"


Write-Host ""
Write-Host "Step 1: Generating Euler/RK4 sweep scenarios..."

& $python .\analysis\generate_sweep.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Sweep generation failed."
    exit 1
}


Write-Host ""
Write-Host "Step 2: Configuring CMake..."

& $cmake `
    -S . `
    -B build-cmake `
    -G "MinGW Makefiles" `
    -DCMAKE_CXX_COMPILER=C:/msys64/ucrt64/bin/g++.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "CMake configuration failed."
    exit 1
}


Write-Host ""
Write-Host "Step 3: Building project..."

& $cmake --build build-cmake

if ($LASTEXITCODE -ne 0) {
    Write-Host "Build failed."
    exit 1
}


Write-Host ""
Write-Host "Step 4: Running C++ unit tests..."

& $ctest `
    --test-dir build-cmake `
    --output-on-failure

if ($LASTEXITCODE -ne 0) {
    Write-Host "C++ unit tests failed."
    exit 1
}


Write-Host ""
Write-Host "Step 5: Running simulations..."

.\build-cmake\main.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "Simulation execution failed."
    exit 1
}


Write-Host ""
Write-Host "Step 6: Running standard scenario validation..."

& $python .\analysis\analyze.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Standard scenario validation failed."
    exit 1
}


Write-Host ""
Write-Host "Step 7: Running cruise-control validation..."

& $python .\analysis\analyze_cruise_control.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Cruise-control validation failed."
    exit 1
}


Write-Host ""
Write-Host "Step 8: Generating Euler/RK4 sweep report..."

& $python .\analysis\sweep_report.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Sweep report failed."
    exit 1
}


Write-Host ""
Write-Host "Step 9: Running regression suite..."

& $python .\analysis\regression_test.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Regression tests failed."
    exit 1
}


Write-Host ""
Write-Host "Step 10: Running standalone Euler vs RK4 comparison..."

.\build-cmake\integration_compare.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "Euler vs RK4 comparison failed."
    exit 1
}


Write-Host ""
Write-Host "Step 11: Running timestep sensitivity experiment..."

.\build-cmake\timestep_sensitivity.exe

if ($LASTEXITCODE -ne 0) {
    Write-Host "Timestep sensitivity experiment failed."
    exit 1
}


Write-Host ""
Write-Host "Step 12: Generating timestep convergence plot..."

& $python .\analysis\plot_convergence.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Convergence plot generation failed."
    exit 1
}


Write-Host ""
Write-Host "Step 13: Generating Euler/RK4 sweep plots..."

& $python .\analysis\plot_integration_comparison.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Integration comparison plotting failed."
    exit 1
}


Write-Host ""
Write-Host "Step 14: Generating final simulation report..."

& $python .\analysis\generate_report.py

if ($LASTEXITCODE -ne 0) {
    Write-Host "Report generation failed."
    exit 1
}


Write-Host ""
Write-Host "========================================="
Write-Host "PIPELINE COMPLETED SUCCESSFULLY"
Write-Host "========================================="

Write-Host ""
Write-Host "Generated outputs:"
Write-Host "  Simulation CSVs:"
Write-Host "    results\"

Write-Host ""
Write-Host "  Trajectory plots:"
Write-Host "    results\*_trajectory.png"

Write-Host ""
Write-Host "  Cruise-control tracking plot:"
Write-Host "    results\cruise_control_tracking.png"

Write-Host ""
Write-Host "  Sweep summary:"
Write-Host "    results\turn_sweep_summary.csv"

Write-Host ""
Write-Host "  Euler/RK4 sweep comparison:"
Write-Host "    results\integration_method_comparison.csv"

Write-Host ""
Write-Host "  Euler/RK4 position plot:"
Write-Host "    results\euler_rk4_position_difference.png"

Write-Host ""
Write-Host "  Euler/RK4 radius-error plot:"
Write-Host "    results\euler_rk4_radius_error.png"

Write-Host ""
Write-Host "  Timestep results:"
Write-Host "    results\timestep_sensitivity.csv"

Write-Host ""
Write-Host "  Convergence plot:"
Write-Host "    results\timestep_convergence.png"

Write-Host ""
Write-Host "  Final report:"
Write-Host "    results\simulation_report.md"

Write-Host ""