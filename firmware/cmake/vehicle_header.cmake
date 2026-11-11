# fw_vehicle_header(<target> <out_dir>): generate <out_dir>/vehicle_config.h from
# config/real/vehicle.yaml with tools/real_vehicle_header.py (plan P0-6: the vehicle values are
# written once, in that YAML) and make <target> depend on it. Needs FW_PYTHON: a Python that can
# import forklift_core. FW_VEHICLE_HEADER=<file> uses an already generated header instead (a build
# outside the repository, e.g. the identity test's temporary copy).
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
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${repo}/src
                ${FW_PYTHON} ${repo}/tools/real_vehicle_header.py --config ${yaml} --print-chassis
        OUTPUT_VARIABLE chassis OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc)
    if(NOT rc EQUAL 0 OR NOT EXISTS ${chassis})
        message(FATAL_ERROR "could not read the chassis model named by ${yaml}")
    endif()
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${yaml})
    add_custom_command(
        OUTPUT ${header}
        COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${repo}/src
                ${FW_PYTHON} ${repo}/tools/real_vehicle_header.py --config ${yaml} --out ${header}
        DEPENDS ${yaml} ${chassis} ${repo}/tools/real_vehicle_header.py
                ${repo}/src/forklift_core/plant/real_vehicle.py
        COMMENT "Generating ${header} from ${yaml}"
        VERBATIM)
    add_custom_target(${target}_vehicle_header DEPENDS ${header})
    add_dependencies(${target} ${target}_vehicle_header)
    target_include_directories(${target} PRIVATE ${out_dir})
endfunction()
