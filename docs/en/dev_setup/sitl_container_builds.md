# Building SITL Containers

The [prebuilt SITL images](../simulation/px4_sitl_prebuilt_packages.md) share the existing `.deb` build and publishing workflow.
`Tools/packaging/docker-bake.hcl` defines the image graph, tags and cache settings; `.github/workflows/build_deb_package.yml` supplies the simulator, architecture and version.

## Build And Publish Flow

1. The package jobs build SIH and Gazebo `.deb`s for Ubuntu Noble and Jammy.
2. Four native container jobs consume the Noble packages: SIH/Gazebo on amd64/arm64.
   Each job runs `Tools/packaging/prepare_container_context.sh` to stage the package context and matching PX4 messages, then builds the runtime image and its ROS child together with Docker Bake.
3. On release tags or a manual run with deployment enabled, the jobs push architecture-specific images to Docker Hub and GHCR.
   The deploy jobs combine those images into versioned and `latest` multi-architecture tags using `docker buildx imagetools create`.

Pull requests build the same graph without publishing.
The ROS image consumes its parent directly through a Bake target context, so neither local builds nor pull requests need an unpublished parent tag in a registry.

## Building Locally

Run these commands from the repository root on Ubuntu 24.04 with [PX4 build dependencies](dev_env_linux_ubuntu.md) and Docker Buildx installed:

```sh
make px4_sitl_sih
(cd build/px4_sitl_sih && cpack -G DEB)
mkdir -p docker-context
cp build/px4_sitl_sih/*.deb docker-context/
bash Tools/packaging/prepare_container_context.sh
SIMULATOR=sih docker buildx bake -f Tools/packaging/docker-bake.hcl --load
```

On macOS or Windows, obtain Noble `.deb`s for the same PX4 revision and Docker architecture instead of building the packages natively, then start at the context-preparation step.
Do not combine packages from different revisions or architectures in one context.
For Gazebo, build `px4_sitl_default` and set `SIMULATOR=gazebo`.

`ARCH` defaults to the Docker builder's local architecture and `VERSION` defaults to `dev`.
For example, an ARM64 SIH build produces `px4io/px4-sitl-ros2:dev-arm64`.
Override `VERSION` and `ARCH` through environment variables when needed.
Replace `--load` with `--print` to inspect the resolved graph without building, or add `--set ros2.tags=px4-ros2:local` to choose a local ROS image tag.

## Cache Boundaries

Every successful `RUN` creates a cacheable layer.
A failed step is not committed as an image layer.
Changing an input invalidates its layer and subsequent layers, not earlier ones.

The ROS Dockerfile declares each source pin immediately before its first use.
The base ROS/rosdep setup, DDS Agent, message-package metadata, interface-library sources, workspace dependency installation, compilation and tests have separate cache boundaries.
Message definitions are copied after dependency installation, so changing them does not repeat `rosdep install`.
Changing only the ROS entrypoint does not rebuild the workspace, and changing only the test command does not recompile it.

The Agent and ROS workspace build in independent Ubuntu 24.04/Jazzy stages, keyed to source pins and the supplied PX4 message definitions rather than the packaged firmware.
A new PX4 binary with unchanged messages reuses those compilation and test layers.
Changed messages invalidate workspace compilation and tests; cached artifacts never substitute messages from another checkout.
The final image still derives from the connected SIH/Gazebo parent and installs its Ubuntu/ROS dependencies using the same helper as the builder.
Only the source-built Agent/logger and complete ROS workspace are copied, preserving the source/build/install layout and absolute symlinks without copying the builder's operating system or package database.
APT cache mounts are held only during installation, not during compilation or tests.
The runtime Dockerfiles extract the package's `Depends` field into a separate stage, so their dependency layers are keyed to that field rather than to every firmware binary change.
Package jobs disable Ubuntu's container cleanup hook so downloaded APT archives survive installation and can be cached between runs.

In Actions, `CACHE_GHA=true` enables one cache per simulator and architecture.
The ROS target exports `mode=max`, covering its entire graph including the runtime parent.
The compiler cache is persisted separately with `actions/cache` and `buildkit-cache-dance`, allowing unchanged compilation units to be reused when workspace inputs change.
Ccache checks compiler contents and source/header inputs; it does not substitute an old ROS workspace for changed PX4 messages.
The container jobs use eight native CPUs, matching the Dockerfile's `BUILD_JOBS` default.
Override that build argument with Bake's `--set ros2.args.BUILD_JOBS=4` when building on a smaller machine.
Local builds use the builder's regular cache by default; standard Bake `--set` options can select other cache backends.
Cached installs do not automatically refresh upstream packages: rebuild without cache when intentionally refreshing dependencies.

## Container SBOMs

The publishing workflow enables BuildKit's standard SBOM attestations for both runtime and ROS images.
These SPDX inventories describe discoverable packages in the image filesystem, including Ubuntu and ROS dependencies.
They complement, rather than replace, PX4's [source and firmware SBOM](../contribute/sbom.md).

Each architecture is published with its attestation.
The final `imagetools create` step preserves those attestations when assembling multi-architecture indexes.
Inspect a published image with:

```sh
docker buildx imagetools inspect px4io/px4-sitl-ros2:<tag> --format '{{json .SBOM}}'
```

Non-publishing pull-request builds do not publish an SBOM for inspection.
Local `--load` builds are intended for running examples; use a registry or an OCI export when retaining attestations.
