# Hospital L Corridor

This Gazebo Sim scene is a closed L-shaped corridor designed for visual-inertial and LiDAR-inertial localization tests. It has a 140 m main leg, a 90 m return leg, a 5.2 m clear width, and a 3.3 m ceiling. The walls contain 36 closed doors and 94 flat notices; there are no open doorways or window apertures.

`rich.sdf` and `low.sdf` have the same geometry and lighting. Their wall, floor, and ceiling textures differ. The paired scene check in `preview_qa.json` records six rendered views per condition and bit-identical LiDAR range arrays for 138,240 samples.

## Run

From the repository root, with Gazebo Sim installed:

```bash
export GZ_SIM_RESOURCE_PATH="$PWD/simulation/scenes${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim simulation/scenes/hospital_l_corridor_v1/rich.sdf
```

Use `low.sdf` for the low-texture condition. `model.config` and the `model://hospital_l_corridor_v1/assets/...` URIs keep the scene assets within this repository.

## Rebuild the worlds

The deterministic generator reads its textures, poster inputs, and template from this repository. It writes into a new directory so the checked-in worlds and their QA record remain intact:

```bash
python3 simulation/scenes/hospital_l_corridor_v1/build_hospital_l_corridor.py \
  --output /tmp/hospital_l_corridor_v1
```

The output includes its model metadata, licenses, worlds, and generated textures. To run it, add `/tmp` to `GZ_SIM_RESOURCE_PATH` and launch `/tmp/hospital_l_corridor_v1/rich.sdf`. Python dependencies are `numpy` and `Pillow`.

## Files

- `rich.sdf`, `low.sdf`: scene variants.
- `assets/`: source material crops, derived wall/floor/ceiling textures, and poster inputs.
- `build_hospital_l_corridor.py`: deterministic world and texture generator.
- `render_hospital_l_preview.py`, `export_previews.py`: RGB preview utilities.
- `qa_hospital_l_corridor.py`, `preview_qa.json`: geometry and paired sensor checks.
- `scene_manifest.json`: dimensions, source hashes, and door/poster placement.
- `AWS_HOSPITAL_LICENSE`, `AWS_HOSPITAL_NOTICE`: material attribution and terms.

## Asset sources

Floor, ceiling, wall, and wood material source images are from [AWS RoboMaker Hospital World](https://github.com/aws-robotics/aws-robomaker-hospital-world), distributed under the MIT license. `AWS_HOSPITAL_NOTICE` retains the upstream notice. Poster inputs include the user-supplied simulated notice, two CDC poster images, and four authored directory signs. CDC source pages are [Clean Hands](https://stacks.cdc.gov/view/cdc/39223) and [Superpower](https://stacks.cdc.gov/view/cdc/58753). The images are simulation textures; the displayed text is not maintained as medical guidance.
