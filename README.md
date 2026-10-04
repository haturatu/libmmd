# libmmd

`libmmd` is a C++20 SDK for PMX models, VMD/VPD motion, MMD animation, and optional physics.

```cmake
find_package(libmmd CONFIG REQUIRED)
target_link_libraries(app PRIVATE mmd::core mmd::animation mmd::physics)
```

It is equally usable as a subdirectory. `mmd::core` has no SDL, Vulkan, FFmpeg, or Bullet requirement.

For nested physics bones, mode 2 bodies whose immediate parent bone has a
usable mode 1 or mode 2 body use effective mode 1. `bodyMode()` reports this
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
