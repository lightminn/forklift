# fw_vehicle_header(<target> <out_dir>): generate <out_dir>/vehicle_config.h from
# config/real/vehicle.yaml with tools/real_vehicle_header.py (plan P0-6: the vehicle values are
# written once, in that YAML) and make <target> depend on it. Needs FW_PYTHON: a Python that can
# import forklift_core. FW_VEHICLE_HEADER=<file> uses an already generated header instead (a build
# outside the repository, e.g. the identity test's temporary copy) -- except on the open normal
# (operating) build, which the board CMake refuses with it (operating plan 2026-11-26 2.4).
function(fw_vehicle_header target out_dir)
    if(FW_VEHICLE_HEADER)
        configure_file(${FW_VEHICLE_HEADER} ${out_dir}/vehicle_config.h COPYONLY)
        target_include_directories(${target} PRIVATE ${out_dir})
        return()
    endif()
    if(NOT FW_PYTHON)
        message(FATAL_ERROR "${target} needs -DFW_PYTHON=<python with forklift_core importable>")
    endif()
    get_filename_component(repo ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../.. ABSOLUTE)
    set(yaml ${repo}/config/real/vehicle.yaml)
    set(header ${out_dir}/vehicle_config.h)
    # Generated at every build, replaced only when its content changes (gen_vehicle_header.cmake): a
    # header left by an earlier configuration of this build directory is never taken (Codex U3d P2-1).
    # FW_VEHICLE_COMMITTED (the operating build): its inputs must be the commit's (Codex U3d P2-2).
    if(FW_VEHICLE_COMMITTED)
        set(committed 1)
    else()
        set(committed 0)
    endif()
    set(gen ${CMAKE_COMMAND} -DREPO=${repo} -DPYTHON=${FW_PYTHON} -DYAML=${yaml} -DOUT=${header}
        -DCOMMITTED=${committed} -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/gen_vehicle_header.cmake)
    if(committed)  # the operating build fails at configure already; the others at their build, as before
        execute_process(COMMAND ${gen} RESULT_VARIABLE rc ERROR_VARIABLE err)
        if(NOT rc EQUAL 0)
            message(FATAL_ERROR "${err}")
        endif()
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${yaml})
    if(committed)
        # at the build: the identity first, and the inputs checked against the commit it names (Codex U3d r3 P2-2)
        set(gen_build ${CMAKE_COMMAND} -DREPO=${repo} -DPYTHON=${FW_PYTHON} -DYAML=${yaml} -DOUT=${header}
            -DCOMMITTED=1 -DBUILD_INFO=${FW_VEHICLE_BUILD_INFO}
            -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/gen_vehicle_header.cmake)
    else()
        set(gen_build ${gen})
    endif()
    add_custom_target(${target}_vehicle_header COMMAND ${gen_build} BYPRODUCTS ${header}
                      COMMENT "Generating ${header} from ${yaml}" VERBATIM)
    if(committed)
        add_dependencies(${target}_vehicle_header build_info)
    endif()
    add_dependencies(${target} ${target}_vehicle_header)
    target_include_directories(${target} PRIVATE ${out_dir})
    # the include names this header by its absolute path: a stray vehicle_config.h beside a source file or
    # in another include directory is never the one compiled (Codex U3d round 2 P2-2)
    target_compile_definitions(${target} PRIVATE "FW_VEHICLE_HEADER_PATH=\"${header}\"")
endfunction()
