-- Setup the extension.
local ext = get_current_extension_info()
project_ext(ext)

-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
}

-- Build the C++ plugin that will be loaded by the extension.
-- The plugin must implement the omni::ext::IExt interface to
-- be automatically loaded by the extension system at startup.
project_ext_plugin(ext, "aerosim.omniverse.extension.plugin")
    add_files("include", "include/aerosim/omniverse/extension")
    add_files("source", "plugins/aerosim.omniverse.extension")
    includedirs {
        "include",
        "plugins/aerosim.omniverse.extension",
        "%{target_deps}/nv_usd/release/include",
    }
    libdirs { "%{target_deps}/nv_usd/release/lib"}
    links { "arch", "gf", "sdf", "tf", "usd", "usdGeom", "usdUtils", "aerosim_world_link" }
    defines { "NOMINMAX", "NDEBUG" }
    runtime "Release"
    rtti "On"

    filter { "system:linux" }
        exceptionhandling "On"
        staticruntime "Off"
        cppdialect "C++17"
        includedirs { "%{target_deps}/python/include/python3.10" }
        buildoptions { "-D_GLIBCXX_USE_CXX11_ABI=0 -Wno-deprecated-declarations -Wno-deprecated -Wno-unused-variable -pthread -lstdc++fs -Wno-undef" }
        linkoptions { "-Wl,--disable-new-dtags -Wl,-rpath,%{target_deps}/nv_usd/release/lib:%{target_deps}/python/lib:" }
    filter { "system:windows" }
        buildoptions { "/wd4244 /wd4305 /EHsc" }
    filter {}

    -- Add the aerosim_world_link library to the project using a separate file to set its path so it can still be built inside a docker container
    local file = io.open("aerosim_world_link_lib_path.txt", "r")
    if file then
        local aerosim_world_link_lib_path = file:read("*line")
        file:close()
        includedirs { aerosim_world_link_lib_path }
        libdirs { aerosim_world_link_lib_path }
        filter { "system:windows" }
            postbuildcommands{
                "{COPY} " .. aerosim_world_link_lib_path .. "/aerosim_world_link.dll " .. ext.target_dir .. "/bin"
            }
        filter { "system:linux" }
            postbuildcommands{
                "{COPY} " .. aerosim_world_link_lib_path .. "/libaerosim_world_link.so " .. ext.target_dir .. "/bin"
            }
        filter {}
    else
        error("Could not read aerosim_world_link_lib_path.txt. Please run build.bat/sh script to regenerate it.")
    end

-- Build Python bindings that will be loaded by the extension.
project_ext_bindings {
    ext = ext,
    project_name = "aerosim.omniverse.extension.python",
    module = "_aerosim_connector_bindings",
    src = "bindings/python/aerosim.omniverse.extension",
    target_subdir = "aerosim/omniverse/extension"
}
    includedirs { "include" }
    repo_build.prebuild_link {
        { "python/impl", ext.target_dir.."/aerosim/omniverse/extension/impl" },
        { "python/tests", ext.target_dir.."/aerosim/omniverse/extension/tests" },
    }