# Run at every build (cmake -P): regenerate build_info.h from the current git state. configure_file
# rewrites the header only when its content changes, so an unchanged tree does not recompile, and
# a new commit or a dirty tree always reaches the ELF (board design 3.7; Codex B1 review).
execute_process(COMMAND git describe --always --dirty --abbrev=12
                WORKING_DIRECTORY ${SRC_DIR}
                OUTPUT_VARIABLE FW_GIT_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT FW_GIT_DESCRIBE)
    set(FW_GIT_DESCRIBE unknown)
endif()
# The USB product string is the identity cut at 126 characters (board design 5). In the waived
# build the host reads the last token, waiver=<8 hex>, so it must fit whole (plan 2026-11-18,
# Codex): refuse a build whose identity would be cut.
if(FW_ESTOP_WAIVED)
    set(ID "forklift-mcu ${FW_BUILD} ${FW_OUTPUTS_WORD} ${FW_GIT_DESCRIBE} rec=00000000 waiver=00000000")
    string(LENGTH "${ID}" ID_LEN)
    if(ID_LEN GREATER 126)
        message(FATAL_ERROR "build identity is ${ID_LEN} characters; the USB product string keeps 126 "
            "and would cut the waiver= token. Shorten the git description (tag name).")
    endif()
endif()
configure_file(${IN} ${OUT} @ONLY)
