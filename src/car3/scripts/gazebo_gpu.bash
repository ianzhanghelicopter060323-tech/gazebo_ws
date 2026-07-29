# Gazebo/RViz GPU helpers for this WSL2/WSLg workspace.
#
# Usage:
#   gazebo_gpu
#   gazebo_gpu gazebo_nav gazebo_nav.launch
#   gazebo_gpu_check

gazebo_gpu_check() (
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    export LIBGL_ALWAYS_SOFTWARE=0

    if ! command -v glxinfo >/dev/null 2>&1; then
        echo "glxinfo 未安装；请执行: sudo apt install mesa-utils" >&2
        return 127
    fi

    glxinfo -B | grep -E 'Device|Accelerated|OpenGL renderer'
)

gazebo_gpu() (
    export GALLIUM_DRIVER=d3d12
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    export LIBGL_ALWAYS_SOFTWARE=0

    if [ -r "$HOME/gazebo_ws/devel/setup.bash" ]; then
        . "$HOME/gazebo_ws/devel/setup.bash"
    else
        echo "找不到 $HOME/gazebo_ws/devel/setup.bash；请先编译工作区。" >&2
        return 1
    fi

    if [ "$#" -eq 0 ]; then
        set -- car3 gazebo.launch gui:=true
    fi

    exec roslaunch "$@"
)
