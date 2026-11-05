# shellcheck shell=bash
# A launcher's TERM (Slurm's, or a wrapper ending it) passed on to the children it starts in the background, however
# early it comes, and the launcher's wait for their own end -- the pattern of deploy/console/isaac_live.sbatch's
# supervisor start (fix/live-launcher-term, #322's review). Before it, these launchers set their trap after the start:
# a TERM between the start and the trap ended the launcher alone and left its child running; their second wait gave
# up on a second TERM and ended before the child; a probe run in the foreground held a trapped TERM until it ended.
#
# Sourcing defines functions only (no trap, no shell option). Use, with bash (not its POSIX mode: a repeated wait of a
# reaped child is 127 there):
#   source "$ROOT/deploy/lib/term_relay.sh" || exit 2     # in the launcher's preflight, so a dry run checks it too
#   term_relay_arm [HANDLER]                             # immediately before the first background start
#   CMD ... &
#   term_relay_hold NAME                                 # tests only; must sit between `&` and `$!`
#   PID=$!
#   term_relay_add "$PID"                                # a TERM that came before is passed on now
#   term_relay_wait "$PID"; rc=$TERM_RELAY_RC            # copy the code at once: the function itself returns 0
# A TERM or INT the launcher receives is passed on as TERM to every child added and not yet waited for, then calls
# HANDLER (a function name) if one was given -- a launcher with its own cancel policy (reloc_session.sh: end the
# launcher within a time, exit 143). TERM_RELAY_GOT says one came; TERM_RELAY_AT is when the first did, in
# microseconds ($EPOCHREALTIME without its point; term_relay_since_us says how long ago).
# A non-interactive bash starts its background children with INT ignored, so only TERM is passed on to them.

term_relay_arm() {
  TERM_RELAY_PIDS=()
  TERM_RELAY_GOT=0
  TERM_RELAY_AT=""
  TERM_RELAY_HANDLER=${1:-}
  trap term_relay_on TERM INT
}

term_relay_on() {
  TERM_RELAY_GOT=1
  [ -n "$TERM_RELAY_AT" ] || TERM_RELAY_AT=${EPOCHREALTIME//[.,]/}
  local p
  for p in "${TERM_RELAY_PIDS[@]}"; do
    kill -TERM "$p" 2>/dev/null
  done
  if [ -n "$TERM_RELAY_HANDLER" ]; then
    "$TERM_RELAY_HANDLER"
  fi
  return 0
}

term_relay_since_us() {
  echo $(( ${EPOCHREALTIME//[.,]/} - TERM_RELAY_AT ))
}

term_relay_add() {
  TERM_RELAY_PIDS+=("$1")
  if [ "$TERM_RELAY_GOT" = 1 ]; then  # it came before this pid was known
    kill -TERM "$1" 2>/dev/null
  fi
  return 0
}

term_relay_hold() {
  # tests only (TERM_RELAY_TEST_HOLD=<path prefix>): hold the window between a start and its `$!` open -- touch
  # <prefix>.<NAME>.at, then wait (30 s at most) for <prefix>.<NAME>. Starts no background job, so `$!` is kept.
  [ -n "${TERM_RELAY_TEST_HOLD:-}" ] || return 0
  local f="$TERM_RELAY_TEST_HOLD.$1" i
  touch "$f.at"
  for i in $(seq 1 600); do
    [ -e "$f" ] && break
    sleep 0.05
  done
  return 0
}

term_relay_wait() {
  # the child's own end: a trapped TERM cuts a wait short (it returns > 128 with the child still up), so wait again
  # until it is gone, then once more for its code; it leaves the list (its pid may be reused later)
  while kill -0 "$1" 2>/dev/null; do
    wait "$1"
  done
  wait "$1"
  TERM_RELAY_RC=$?
  local keep=() p
  for p in "${TERM_RELAY_PIDS[@]}"; do
    [ "$p" = "$1" ] || keep+=("$p")
  done
  TERM_RELAY_PIDS=("${keep[@]}")
  return 0
}
