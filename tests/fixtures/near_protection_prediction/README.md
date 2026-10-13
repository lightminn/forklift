# Protection prediction cases (0.29 m fork spacing)

`cases.json` holds the recorded inputs of two screened 0.6 m correction decisions (S4 development
batch `s4dev`: seed 6 correction lateral +0.02, refused; seed 0 no injection, passed) and the margins
the protection prediction gave them.

Those margins depend on the forks, the near-field bounds and the S4 drive terms the runs used, so the
three files are pinned here as they were at 0.29 m fork spacing (the measured model before the EPAL 6
fork spacing plan of 2026-10-13 moved it to 0.3725 m):

- `forklift_0p29.urdf` — `sim/models/dls08_measured/forklift.urdf` at commit b8a5147.
- `near_field_bounds_0p29.json`, `near_field_drive_terms_0p29.json` — the `config/` files at that commit.

The historical regression tests read these, not the current model and tables, so replacing either
table after a re-measurement does not break them.
