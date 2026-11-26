# Generate vehicle_config.h at every build (cmake -P), replacing the header only when its content
# changes: a header left in the build directory by an earlier configuration (a FW_VEHICLE_HEADER
# copy, another commit's values) is never taken as up to date (operating plan 2026-11-26 U3d, Codex
# round 1 P2-1).
#   -DREPO=<repository root> -DPYTHON=<python> -DYAML=<vehicle.yaml> -DOUT=<header>
#   -DCOMMITTED=1 (the open normal, operating build): the config and the chassis model it names must
#   be regular files in that repository whose work-tree bytes are their HEAD blobs, so the header
#   is the commit's (C12; the same test as check_protection_record.cmake, Codex round 1 P2-2).

function(refuse why)
    message(FATAL_ERROR "vehicle header: ${why}. The operating build's vehicle values must be the "
        "commit's config/real/vehicle.yaml and the chassis model it names (operating plan 2026-11-26 2.4, C12).")
endfunction()

function(check_committed top path)
    # no symbolic link anywhere on the path: the path itself is the file the generator reads (its
    # resolve() then changes nothing), so the bytes checked are the bytes used (Codex U3d round 2 P2-1)
    get_filename_component(real "${path}" REALPATH)
    if(NOT real STREQUAL path)
        refuse("'${path}' goes through a symbolic link")
    endif()
    file(RELATIVE_PATH rel "${top}" "${path}")
    if(rel MATCHES "^\\.\\./" OR rel STREQUAL ".." OR IS_ABSOLUTE "${rel}")
        refuse("'${path}' is outside the repository")
    endif()
    if(IS_SYMLINK "${path}" OR NOT EXISTS "${path}" OR IS_DIRECTORY "${path}")
        refuse("'${rel}' is not a regular file")
    endif()
    execute_process(COMMAND git diff --cached --quiet HEAD -- "${rel}" WORKING_DIRECTORY ${top}
                    RESULT_VARIABLE index_rc OUTPUT_QUIET ERROR_QUIET)
    # the work-tree bytes are hashed directly: git diff skips assume-unchanged / skip-worktree files
    execute_process(COMMAND git rev-parse "HEAD:${rel}" WORKING_DIRECTORY ${top}
                    OUTPUT_VARIABLE head_id OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    execute_process(COMMAND git hash-object --no-filters -- "${path}" WORKING_DIRECTORY ${top}
                    OUTPUT_VARIABLE tree_id OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT index_rc EQUAL 0 OR NOT head_id OR NOT tree_id STREQUAL head_id)
        refuse("'${rel}' is not committed or differs from its committed version")
    endif()
endfunction()

execute_process(
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
            ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${YAML} --print-chassis
    OUTPUT_VARIABLE chassis OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc)
if(NOT rc EQUAL 0 OR NOT EXISTS "${chassis}")
    message(FATAL_ERROR "could not read the chassis model named by ${YAML}")
endif()
macro(check_inputs)
    if(COMMITTED)
        execute_process(COMMAND git rev-parse --show-toplevel WORKING_DIRECTORY ${REPO}
                        OUTPUT_VARIABLE top RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT rc EQUAL 0 OR NOT top)
            refuse("the repository is not a git repository")
        endif()
        get_filename_component(top "${top}" REALPATH)
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
                    ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${YAML} --print-chassis-as-written
            OUTPUT_VARIABLE written OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc)
        if(rc EQUAL 3)
            refuse("the chassis model path ${YAML} names goes through a symbolic link")
        endif()
        if(NOT rc EQUAL 0 OR NOT written)
            refuse("could not read the chassis model path ${YAML} names")
        endif()
        check_committed("${top}" "${YAML}")
        check_committed("${top}" "${written}")
        if(NOT written STREQUAL chassis)
            refuse("the chassis model resolves to '${chassis}', not the path '${written}' the config names")
        endif()
        # the commit the build identity names (build_info, run first) is the one these inputs are checked against
        # (Codex U3d round 3 P2-2: HEAD moving between the two targets); BUILD_INFO is absent at configure
        if(BUILD_INFO)
            file(READ "${BUILD_INFO}" info)
            if(NOT info MATCHES "#define FW_GIT_DESCRIBE \"([^\"]*)\"")
                refuse("no git description in ${BUILD_INFO}")
            endif()
            set(desc "${CMAKE_MATCH_1}")
            # a -dirty identity is the C1 board check's to refuse (dirty_image); here only the commit it names
            if(NOT desc MATCHES "^(.+-[0-9]+-g)?([0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f]+)(-dirty)?$")
                refuse("the build identity '${desc}' names no commit")
            endif()
            set(abbrev "${CMAKE_MATCH_2}")
            execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${REPO}
                            OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
            string(FIND "${head}" "${abbrev}" at)
            if(NOT at EQUAL 0)
                refuse("HEAD (${head}) is not the commit the build identity names (${abbrev})")
            endif()
        endif()
    endif()
endmacro()
check_inputs()
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
            ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${YAML} --out ${OUT}.new
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "could not generate ${OUT} from ${YAML}")
endif()
check_inputs()  # again after the generation: the bytes it read are still the committed ones
file(READ "${OUT}.new" new)
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL new)
    file(WRITE "${OUT}" "${new}")
endif()
file(REMOVE "${OUT}.new")
