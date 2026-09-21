# Reproducibility provenance

- Jev model: `jev-1.13.0` through the TypeSafe `/v1/systemone` API.
- TypeSafe Python SDK: `0.6.0`.
- MuJoCo: `3.3.7`.
- Panda geometry: Franka Emika Panda asset from the [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/tree/main/franka_emika_panda).
- Public mechanism reference: [Dmytro Hrybov's llm-robotics-playground](https://github.com/dimentary/llm-robotics-playground) and the [Jev as Policy post](https://x.com/dimentary/status/2101018760371171420).
- The repository contains a simulation harness reconstructed from the public description. It does not claim to contain the original author's private prompt, scene, controller, or source code.

The repository includes the Panda mesh assets and license notice needed to render the included scene. The Jev API key is always supplied at runtime and is never stored in this repository.
