"""Display-only outline of the measured chassis for the console's welcome (contract 3).

dls08_measured (sim/models/dls08_measured/parameters.yaml), measured from the rear axle at
base_link x -0.34 (an estimate kept fixed): fork tip 0.95 = rear extent -0.51 + overall
length 1.46, width 0.63, wheelbase 0.66, both wheels at 15 deg give 0.3665 1/m. The page
draws the truck and its teleop sweep with it only; nothing here decides motion.
"""

DLS08_MEASURED_CHASSIS = {
    "rear_overhang_m": 0.17,
    "nose_from_rear_m": 1.29,
    "half_width_m": 0.315,
    "wheelbase_m": 0.66,
    "max_curvature_inv_m": 0.3665,
    "model_turning_radius_m": 1 / 0.3665,
}
