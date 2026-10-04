# libmmd

`libmmd` is a C++20 SDK for PMX models, VMD/VPD motion, MMD animation, and optional physics.

```cmake
find_package(libmmd CONFIG REQUIRED)
target_link_libraries(app PRIVATE mmd::core mmd::animation mmd::physics)
```

It is equally usable as a subdirectory. `mmd::core` has no SDL, Vulkan, FFmpeg, or Bullet requirement.

For nested physics bones, mode 2 bodies connected by a usable joint to a mode
1 or mode 2 body on their immediate parent bone use effective mode 1. `bodyMode()` reports this
runtime mode; the source PMX stays unchanged. Standalone mode 2 bones retain
animated translation. Bone results are applied in parent order.

The original constructor retains the default Bullet constraint settings,
10 solver iterations and a maximum substep of 1/120 second. To opt into the
MMD 2.75 constraint compatibility preset:

```cpp
mmd::PhysicsSettings settings;
settings.compatibility = mmd::PhysicsCompatibilityProfile::mmd275Compatible();
// Optional solver tuning after checking the model with the defaults:
settings.solverIterations = 25;
settings.fixedTimeStep = 1.0F / 240.0F;
mmd::MmdPhysics physics(model, settings);
```

The preset uses frame offsets disabled, a frame-A linear-constraint lever arm,
STOP ERP 0.475 on all six axes and solver CFM 0.00001. The lever-arm correction
is local to libmmd constraints, supports system and fetched Bullet, and follows
[babylon-mmd's constraint correction](https://github.com/noname0310/babylon-mmd/blob/main/src/Runtime/Optimized/wasm_src/bullet_src/constraint-fix.patch).
This is a constraint compatibility preset, not a replacement for the entire
Bullet 2.75 engine. Collision masks and the 0.01 margin remain unchanged.

Settings are validated even without Bullet: iterations must be 1–1000,
`fixedTimeStep` must be finite and between 1/10000 and 1/4 second, CFM must be
finite and nonnegative, and STOP ERP must be between 0 and 1. The 2.75 lever-arm
option requires `useFrameOffset == false`. Invalid settings throw
`std::invalid_argument`. Existing stepping divides each update into equal
substeps no larger than `fixedTimeStep`.

The physics compatibility test can also check a locally supplied Vivian model:

```sh
build/libmmd_physics_compatibility_tests /path/to/Vivian.pmx
```

It verifies the 150 main skirt boxes, 15 mode 2 roots and 135 effective mode 1
children, skirt bone/body center agreement and finite mesh positions for 120
simulation updates with each profile. The model is not included in the repo.

`MmdAnimator::setPose()` overrides the VMD local translation and rotation of
bones named in the VPD. Other bone and morph tracks continue to use the motion.
`setPose(nullptr)` restores the motion, and either change resynchronizes physics.

Document relation edits and global validation share a nonrecursive dependency
cycle check covering parent and active inheritance edges. Each check is
O(bones + edges); an incremental editor graph cache is not implemented.
Transaction patch generation uses handle-table indexes for membership checks.

Rigid-body property edits, transaction commits and saves share validation of
shape/mode/group and every floating-point field. Soft-body material, anchor and
pinned-vertex references use the same checks in all three paths. Optional
material and anchor rigid-body references retain the editor's `-1` convention;
anchor and pinned vertices must be valid vertex indices.

Serialization validation also rejects unknown text encodings, index widths
outside 1/2/4 and unknown vertex weight types. Joint types follow the PMX
version (2.0: 0; 2.1: 0–5), and all joint float arrays must be finite. Soft bodies
validate shape 0/1, group 0–15, aero model 0–4 and every float field/array.

`pmx::save()` writes into a private staging directory beside the destination,
checks flush and close, then replaces the destination with the complete file.
Validation, serialization and I/O failures before replacement leave the old
file intact; staging files are cleaned up on exceptions. POSIX mode bits are
preserved when replacing an existing regular file. Replacement uses same-volume
rename on POSIX and `MoveFileExW` on Windows, without a delete/copy fallback.
Flush/close is not a guarantee of persistence through a power failure; file and
directory fsync are not performed.
