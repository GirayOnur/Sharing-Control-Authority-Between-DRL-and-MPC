# Sharing the Control Authority Between Deep Reinforcement Learning and Model Predictive Control: Application to Multi-Class Transportation Networks

This repository contains the source code for the case study presented in the paper. The proposed DRL-MPC framework divides control inputs between DRL and MPC. MPC operates at the high level and computes the low-frequency vehicle splitting rate, whereas DRL operates at the low level and computes the high-frequency ramp metering rates. This combines MPC's built-in optimization and constraint-handling capabilities with DRL's fast online computation and model independence.

## Case study

The framework is evaluated on a benchmark multi-class freeway network with two vehicle classes. The network is simulated using the multi-class METANET model, and the MPC problems are solved in MATLAB using `fmincon`.

The vehicle splitting rate is updated every 300 s, the two ramp metering rates every 60 s, and the network state every 10 s. Four scenarios combine nominal or noisy traffic demands with a matched or mismatched MPC prediction model. The DDPG-MPC and SAC-MPC variants are compared with a hierarchical MPC controller, a state-feedback-MPC controller using PI-ALINEA, and the no-control case.

## Repository structure

```text
DDPG_MPC/         DDPG-MPC experiments
SAC_MPC/          SAC-MPC experiments
PI_ALIENA_MPC/    SF-MPC experiments
plots/comparison/ evaluation and plotting scripts
```

Folder suffixes identify the scenarios: no suffix for Scenario 1, `_nd` for Scenario 2, `_mm` for Scenario 3, and `_mm_nd` for Scenario 4. The trained agents, tuned PI-ALINEA parameters, and initial network states are included.

## Running the evaluations

MATLAB R2025a is recommended with the Optimization, Reinforcement Learning, Statistics and Machine Learning, Signal Processing, and Parallel Computing toolboxes.

Enter an experiment folder and run its benchmark script, for example `benchmark_RL_MPC_SR_RM`. Run `run_10_experiments` to perform the ten evaluation simulations. Results are saved in the experiment folder as `<framework>_result_<timestamp>.mat`; the scripts in `plots/comparison` generate the evaluation figures and tables.
