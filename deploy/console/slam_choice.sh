# The live session's SLAM mode (docs/plans/2026-10-26-slam-localization-default.md D2; the default switched to the
# registry by user decision 2026-10-27, plan 5d), sourced by isaac_live.sbatch after ENCODER_MODEL is set. Reads ROOT,
# VENV_PY, ENCODER_MODEL and optionally MAP_BUNDLE, MAP_START_POSE, EXPECTED_MAP_ID, SCENE_FILE, SLAM_BUNDLE_ROOT,
# SLAM_BUNDLE_REGISTRY; sets MAP_BUNDLE (empty: online mapping), MAP_START_POSE, EXPECTED_MAP_ID and SLAM_CHOICE (what was
# chosen and why, for the session's log).
#   MAP_BUNDLE unset    the default: the accepted survey bundle of SCENE_FILE (by content) and ENCODER_MODEL from
#                       config/slam_map_bundles.yaml -- its folder under SLAM_BUNDLE_ROOT, its map id (held against the
#                       folder by the supervisor) and its start pose. No row for them, or no SCENE_FILE (the factory
#                       hall): online mapping as a fallback, said in SLAM_CHOICE ("mapping (fallback: ...)") and on
#                       stderr. A row without SLAM_BUNDLE_ROOT stops the launch (localization was meant)
#   MAP_BUNDLE=registry the same bundle, asked for: no SCENE_FILE or no row stops the launch
#   MAP_BUNDLE=none     online mapping, whatever the registry says (comparison arms, survey epochs)
#   MAP_BUNDLE=<folder> that bundle, with MAP_START_POSE (and EXPECTED_MAP_ID if given)
# A registry that does not read stops the launch either way.
EXPECTED_MAP_ID=${EXPECTED_MAP_ID:-}
MAP_START_POSE=${MAP_START_POSE:-}
slam_fallback() {  # $1: why; online mapping, recorded
  MAP_BUNDLE=; EXPECTED_MAP_ID=; MAP_START_POSE=
  SLAM_CHOICE="mapping (fallback: $1)"
  echo "slam: no accepted survey bundle -- $SLAM_CHOICE" >&2
}
SLAM_ASKED=${MAP_BUNDLE:-default}
case "$SLAM_ASKED" in
  none) MAP_BUNDLE=; EXPECTED_MAP_ID=; MAP_START_POSE=; SLAM_CHOICE="mapping (MAP_BUNDLE=none)" ;;
  default|registry)
    if [ -z "${SCENE_FILE:-}" ]; then
      [ "$SLAM_ASKED" = registry ] && { echo "MAP_BUNDLE=registry needs SCENE_FILE" >&2; exit 2; }
      slam_fallback "no SCENE_FILE, the factory hall has no registry row"
    else
      SLAM_ROW=$("$VENV_PY" "$ROOT/tools/slam_bundle_registry.py" lookup \
        "${SLAM_BUNDLE_REGISTRY:-$ROOT/config/slam_map_bundles.yaml}" --scene-file "$SCENE_FILE" \
        --encoder-model "$ENCODER_MODEL")
      SLAM_RC=$?
      if [ "$SLAM_RC" -eq 0 ]; then
        : "${SLAM_BUNDLE_ROOT:?the registry has a bundle for this scene: SLAM_BUNDLE_ROOT (or MAP_BUNDLE=none to map)}"
        read -r SLAM_FOLDER EXPECTED_MAP_ID MAP_START_POSE <<< "$SLAM_ROW"
        MAP_BUNDLE=$SLAM_BUNDLE_ROOT/$SLAM_FOLDER
        SLAM_CHOICE="localization (registry: $SLAM_FOLDER)"
      elif [ "$SLAM_RC" -eq 3 ]; then
        SLAM_WHY="no registry row for $(basename "$SCENE_FILE") and $ENCODER_MODEL"
        [ "$SLAM_ASKED" = registry ] && { echo "MAP_BUNDLE=registry: $SLAM_WHY" >&2; exit 2; }
        slam_fallback "$SLAM_WHY"
      else
        echo "the SLAM bundle registry did not read (exit $SLAM_RC)" >&2
        exit 2
      fi
    fi ;;
  *) SLAM_CHOICE="localization (given: $MAP_BUNDLE)" ;;
esac
