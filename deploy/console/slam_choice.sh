# The live session's SLAM mode (docs/plans/2026-10-26-slam-localization-default.md D2), sourced by isaac_live.sbatch
# after ENCODER_MODEL is set. Reads ROOT, VENV_PY, ENCODER_MODEL and optionally MAP_BUNDLE, MAP_START_POSE,
# EXPECTED_MAP_ID, SCENE_FILE, SLAM_BUNDLE_ROOT, SLAM_BUNDLE_REGISTRY; sets MAP_BUNDLE (empty: online mapping),
# MAP_START_POSE, EXPECTED_MAP_ID and SLAM_CHOICE (what was chosen and why, for the session's log).
#   MAP_BUNDLE unset    online mapping
#   MAP_BUNDLE=none     online mapping, whatever the registry says (comparison arms, survey epochs)
#   MAP_BUNDLE=registry the accepted survey bundle of SCENE_FILE (by content) and ENCODER_MODEL from
#                       config/slam_map_bundles.yaml: its folder under SLAM_BUNDLE_ROOT, its map id (held against the
#                       folder by the supervisor) and its start pose; no row: online mapping; a registry that does not
#                       read stops the launch
#   MAP_BUNDLE=<folder> that bundle, with MAP_START_POSE (and EXPECTED_MAP_ID if given)
EXPECTED_MAP_ID=${EXPECTED_MAP_ID:-}
MAP_START_POSE=${MAP_START_POSE:-}
case "${MAP_BUNDLE:-}" in
  "") MAP_BUNDLE=; SLAM_CHOICE="mapping (MAP_BUNDLE unset)" ;;
  none) MAP_BUNDLE=; EXPECTED_MAP_ID=; MAP_START_POSE=; SLAM_CHOICE="mapping (MAP_BUNDLE=none)" ;;
  registry)
    : "${SCENE_FILE:?MAP_BUNDLE=registry needs SCENE_FILE}" "${SLAM_BUNDLE_ROOT:?MAP_BUNDLE=registry needs SLAM_BUNDLE_ROOT}"
    SLAM_ROW=$("$VENV_PY" "$ROOT/tools/slam_bundle_registry.py" lookup \
      "${SLAM_BUNDLE_REGISTRY:-$ROOT/config/slam_map_bundles.yaml}" --scene-file "$SCENE_FILE" \
      --encoder-model "$ENCODER_MODEL")
    SLAM_RC=$?
    if [ "$SLAM_RC" -eq 0 ]; then
      read -r SLAM_FOLDER EXPECTED_MAP_ID MAP_START_POSE <<< "$SLAM_ROW"
      MAP_BUNDLE=$SLAM_BUNDLE_ROOT/$SLAM_FOLDER
      SLAM_CHOICE="localization (registry: $SLAM_FOLDER)"
    elif [ "$SLAM_RC" -eq 3 ]; then
      MAP_BUNDLE=; EXPECTED_MAP_ID=; MAP_START_POSE=
      SLAM_CHOICE="mapping (no registry row for $(basename "$SCENE_FILE") and $ENCODER_MODEL)"
    else
      echo "the SLAM bundle registry did not read (exit $SLAM_RC)" >&2
      exit 2
    fi ;;
  *) SLAM_CHOICE="localization (given: $MAP_BUNDLE)" ;;
esac
