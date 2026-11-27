# Run at every build (cmake -P): regenerate build_info.h from the current git state. configure_file
# rewrites the header only when its content changes, so an unchanged tree does not recompile, and
# a new commit or a dirty tree always reaches the ELF (board design 3.7; Codex B1 review).
# --long: a tagged build is always <tag>-<n>-g<abbrev>, never the bare tag, so the commit
# abbreviation is in every identity whatever the tag is named (operating plan 2026-11-26 U3, Codex
# U1 round 4: a tag that looks like the long form must not be taken for a commit). The host reads
# the abbreviation after the last -g (forklift_core.live.real_preconditions.image_commit).
execute_process(COMMAND git describe --always --dirty --abbrev=12 --long
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
# The open commissioning build carries its limit generation as the last token (" lim=lr1"); the
# bring-up tool reads it whole, so it must not be cut either (design 2026-11-25 1.1).
# fw-single (partial-parts plan 2026-11-25 PR-B) appends " sa1" after it: the tool tells the image
# apart only by that exact suffix, so it must not be cut either.
if(FW_BUILD STREQUAL "commission" AND NOT FW_OUTPUTS_LOCKED)
    if(FW_SINGLE_AXIS)
        set(FW_LIMITS_TAG " lim=lr1 sa1")
    elseif(FW_BENCH) # the bench image and its wire generation (mcu-link contract 8)
        set(FW_LIMITS_TAG " lim=lr1 bench5")
    else()
        set(FW_LIMITS_TAG " lim=lr1")
    endif()
    set(ID "forklift-mcu ${FW_BUILD} ${FW_OUTPUTS_WORD} ${FW_GIT_DESCRIBE} rec=00000000${FW_LIMITS_TAG}")
    string(LENGTH "${ID}" ID_LEN)
    if(ID_LEN GREATER 126)
        message(FATAL_ERROR "build identity is ${ID_LEN} characters; the USB product string keeps 126 "
            "and would cut the lim= token. Shorten the git description (tag name).")
    endif()
else()
    set(FW_LIMITS_TAG "")
endif()
# Every image: the whole identity fits the 126-character USB product string, which the host and the
# board check's staged copy compare whole (--long lengthens a tagged build's description; Codex U3d P3).
set(ID "forklift-mcu ${FW_BUILD} ${FW_OUTPUTS_WORD} ${FW_GIT_DESCRIBE}")
if(NOT FW_OUTPUTS_LOCKED)
    string(APPEND ID " rec=00000000")
endif()
if(FW_ESTOP_WAIVED)
    string(APPEND ID " waiver=00000000")
endif()
string(APPEND ID "${FW_LIMITS_TAG}")
string(LENGTH "${ID}" ID_LEN)
if(ID_LEN GREATER 126)
    message(FATAL_ERROR "build identity is ${ID_LEN} characters; the USB product string keeps 126 "
        "and would cut it. Shorten the git description (tag name).")
endif()
configure_file(${IN} ${OUT} @ONLY)
