# libmmd

`libmmd` is a C++20 SDK for PMX models, VMD/VPD motion, MMD animation, and optional physics.

```cmake
find_package(libmmd CONFIG REQUIRED)
target_link_libraries(app PRIVATE mmd::core mmd::animation mmd::physics)
```

It is equally usable as a subdirectory. `mmd::core` has no SDL, Vulkan, FFmpeg, or Bullet requirement.
