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
if(COMMITTED)
    execute_process(COMMAND git rev-parse --show-toplevel WORKING_DIRECTORY ${REPO}
                    OUTPUT_VARIABLE top RESULT_VARIABLE rc OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
    if(NOT rc EQUAL 0 OR NOT top)
        refuse("the repository is not a git repository")
    endif()
    check_committed("${top}" "${YAML}")
    check_committed("${top}" "${chassis}")
endif()
execute_process(
    COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${REPO}/src
            ${PYTHON} ${REPO}/tools/real_vehicle_header.py --config ${YAML} --out ${OUT}.new
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "could not generate ${OUT} from ${YAML}")
endif()
file(READ "${OUT}.new" new)
if(EXISTS "${OUT}")
    file(READ "${OUT}" old)
endif()
if(NOT old STREQUAL new)
    file(WRITE "${OUT}" "${new}")
endif()
file(REMOVE "${OUT}.new")
