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

function(check_committed top sha path)
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
    execute_process(COMMAND git diff --cached --quiet ${sha} -- "${rel}" WORKING_DIRECTORY ${top}
                    RESULT_VARIABLE index_rc OUTPUT_QUIET ERROR_QUIET)
    # the work-tree bytes are hashed directly: git diff skips assume-unchanged / skip-worktree files
    execute_process(COMMAND git rev-parse "${sha}:${rel}" WORKING_DIRECTORY ${top}
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
        # one commit for every check and blob read of this run (Codex U3d round 5 P2-2): fixed at the first check,
        # and HEAD must still be it at the second
        execute_process(COMMAND git rev-parse HEAD WORKING_DIRECTORY ${top}
                        OUTPUT_VARIABLE head OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
        if(NOT sha)
            set(sha "${head}")
        endif()
        if(NOT head OR NOT head STREQUAL sha)
            refuse("HEAD moved during the header's generation (${sha} -> ${head})")
        endif()
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
                    ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${YAML} --print-chassis-as-written
            OUTPUT_VARIABLE written OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE rc)
        if(rc EQUAL 3)
            refuse("the chassis model path ${YAML} names goes through a symbolic link")
        endif()
        if(rc EQUAL 4)
            refuse("the chassis model path ${YAML} names is absolute (it must be relative to the config)")
        endif()
        if(NOT rc EQUAL 0 OR NOT written)
            refuse("could not read the chassis model path ${YAML} names")
        endif()
        check_committed("${top}" "${sha}" "${YAML}")
        check_committed("${top}" "${sha}" "${written}")
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
            string(FIND "${sha}" "${abbrev}" at)
            if(NOT at EQUAL 0)
                refuse("HEAD (${sha}) is not the commit the build identity names (${abbrev})")
            endif()
        endif()
    endif()
endmacro()
check_inputs()
# The operating build generates from the committed blobs themselves (the commit the identity names), staged and
# both handed to the generator by their staged paths -- never work-tree bytes that could change between the checks
# (Codex U3d round 4 P2-2), nor a chassis path in the config that could lead out of the stage (round 6 P2).
set(gen_yaml "${YAML}")
set(gen_chassis "")
if(COMMITTED)
    set(stage "${OUT}.stage")
    file(REMOVE_RECURSE "${stage}")
    foreach(path "${YAML}" "${written}")
        file(RELATIVE_PATH rel "${top}" "${path}")
        get_filename_component(dir "${stage}/${rel}" DIRECTORY)
        file(MAKE_DIRECTORY "${dir}")
        execute_process(COMMAND git cat-file blob "${sha}:${rel}" WORKING_DIRECTORY ${top}
                        OUTPUT_FILE "${stage}/${rel}" RESULT_VARIABLE rc ERROR_QUIET)
        if(NOT rc EQUAL 0)
            refuse("'${rel}' could not be read from ${sha}")
        endif()
    endforeach()
    file(RELATIVE_PATH rel_yaml "${top}" "${YAML}")
    file(RELATIVE_PATH rel_chassis "${top}" "${written}")
    set(gen_yaml "${stage}/${rel_yaml}")
    set(gen_chassis --chassis "${stage}/${rel_chassis}")
endif()
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
            ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${gen_yaml} ${gen_chassis} --out ${OUT}.new
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "could not generate ${OUT} from ${YAML}")
endif()
check_inputs()  # again after the generation: HEAD is still the identity's and the work tree still its blobs
if(COMMITTED)
    file(REMOVE_RECURSE "${stage}")
endif()
file(READ "${OUT}.new" new)
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL new)
    file(WRITE "${OUT}" "${new}")
endif()
file(REMOVE "${OUT}.new")
