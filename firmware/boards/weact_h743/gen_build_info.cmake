# Run at every build (cmake -P): regenerate build_info.h from the current git state. configure_file
# rewrites the header only when its content changes, so an unchanged tree does not recompile, and
# a new commit or a dirty tree always reaches the ELF (board design 3.7; Codex B1 review).
execute_process(COMMAND git describe --always --dirty --abbrev=12
                WORKING_DIRECTORY ${SRC_DIR}
                OUTPUT_VARIABLE FW_GIT_DESCRIBE OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET)
if(NOT FW_GIT_DESCRIBE)
    set(FW_GIT_DESCRIBE unknown)
endif()
configure_file(${IN} ${OUT} @ONLY)
