# Sharing the Control Authority Between Deep Reinforcement Learning and Model Predictive Control: Application to Multi-Class Transportation Networks

Source code for the case study of the paper. The DRL-MPC framework divides the
control inputs between a DRL agent and an MPC controller: the MPC controller
sits at the high level and computes the low-frequency control inputs, while the
DRL agent sits at the low level and computes the high-frequency ones. That way
MPC keeps its built-in optimization and constraint handling on the inputs, and 
DRL covers the inputs that have to be updated at high frequency.

The freeway network is simulated with the multi-class METANET model, and the MPC 
problems are solved with `fmincon` function of MATLAB.

## The case study

```
                                     O2
                                      v
                  +--> [2_1] --> [3_1 3_2] --+
O1 --> [1_1 1_2 1_3]                         +--> D1
                  +--> [4_1] --> [5_1 5_2] --+
                                      ^
                                     O3
```

A mainstream link of three four-lane segments fed by mainstream origin O1,
splitting into a primary route (on-ramp O2) and a secondary route (on-ramp O3)
of three two-lane segments each, both ending at destination D1. Two vehicle
classes, 75 states in one vector `x`. The state order is in the header of
`fun_benchmark_RM.m`.

Two control measures, both applied to both classes and bounded in [0, 1]:

- the **vehicle split rate** at the node, set through a variable message sign
  for route guidance, updated every `T_h` = 300 s
- the two **ramp metering rates**, updated every `T_l` = 60 s

The network sampling time step is `T` = 10 s and a simulation covers 2.5 h.
Before that, the network runs for 10 min under the no-control setting so every
controller starts from a congested network. The queue length limits are 200
vehicles at O1 and 100 at each on-ramp.

## Control frameworks

| framework | ramp metering rates | vehicle split rate |
| --- | --- | --- |
| No control | fixed at 1 | fixed at 0.5 |
| SF-MPC | PI-ALINEA state feedback | MPC |
| Hierarchical MPC | low-level MPC | high-level MPC |
| DDPG-MPC | DDPG agent | MPC |
| SAC-MPC | SAC agent, stochastic | MPC |
| SACD-MPC | SAC agent, deterministic | MPC |

In DDPG-MPC, SAC-MPC, SACD-MPC and SF-MPC, the high-level MPC controller evaluates the low-level
controller inside its own prediction, so the predicted ramp metering rates are
the ones that will actually be applied. The hierarchical MPC controller instead
reuses the most recent low-level trajectory, shifted by one step, because
solving the low-level MPC problem inside the high-level one is too expensive.

SACD-MPC uses the same trained agents as SAC-MPC, deployed with the mean of the
policy instead of a sample from it.

## Scenarios

Four scenarios combine demand uncertainty and prediction model mismatch. The
simulator always uses the real parameter values; what changes is the demand and
what the MPC prediction model assumes. The folder suffixes follow this:

| suffix | scenario | demand | MPC prediction model |
| --- | --- | --- | --- |
| *(none)* | 1 | nominal | real parameters, nominal demands |
| `_nd` | 2 | noisy | real parameters, noisy demands |
| `_mm` | 3 | nominal | perturbed parameters, estimated demands |
| `_mm_nd` | 4 | noisy | perturbed parameters, estimated demands |

The noisy profiles are the nominal ones plus zero-mean Gaussian noise, smoothed
with a third-order low-pass Butterworth filter (`calc_noisy_demands.m`). The
perturbed parameters are in `param_get_mm.m` and the estimated demands in
`demando1mm.m`, `demando2mm.m` and `demando3mm.m`.

## Layout

```
DDPG_MPC/         DDPGpi_main_s5_<run><suffix>
SAC_MPC/          SACpi_main_s5_<run><suffix>
SACD_MPC/         SACDpi_main_s5_<run><suffix>
PI_ALIENA_MPC/    PI_ALINEA_MPC<suffix>_<run>
plots/comparison/ figures, tables and the scripts that make them
```

`<run>` is 1 to 5: five independently trained agents per algorithm and
scenario, and five independent Bayesian optimization runs for PI-ALINEA. Each
one is evaluated 10 times. The no-control case and the hierarchical MPC
controller do not depend on an agent, so they are only run in the
`DDPGpi_main_s5_1*` folders.

The experiment folders are near-copies of each other, each self-contained so it
can run on its own. Inside one, the files worth reading are
`fun_benchmark_RM.m` (one sampling step of the METANET model) and the
`calc_*.m` equations it is built from, `param_get.m` and `param_MPC_get.m` for
the parameter and controller settings, `demando1/2/3.m` for the demand
profiles, `MPC_solve_*.m` for the MPC problems, `const_RL.m` with
`rlStepFunc.m` and `rlResFunc.m` for the agent and its training environment,
and `benchmark_*.m` for one evaluation simulation.

## Running

Needs MATLAB with the Optimization, Reinforcement Learning, Statistics and
Machine Learning, Signal Processing and Parallel Computing toolboxes.

The trained agents, the tuned PI-ALINEA parameters and the initial states are
all in the repository, so nothing has to be retrained. `cd` into an experiment
folder and run the benchmark script in it, for example
`benchmark_RL_MPC_SR_RM`. For the 10 evaluation simulations with seeds 1 to 10
and a diary of the output, use `run_10_experiments`.

Results are saved next to the script as `<framework>_result_<timestamp>.mat`.
They hold the raw state trajectory `xx` and input trajectory `uu`; the
evaluation metrics are recomputed from those by the comparison scripts.

## Training

`train_RL_MPC` prepares the starting point and then runs `const_RL.m`, which
builds the agent and trains it. During training the MPC
controller uses a soft penalty on the queue length limits rather than hard
constraints, to avoid infeasibility while the policy is still poor.

## Results

The scripts in `plots/comparison` read the result files out of the experiment
folders and produce the figures and LaTeX tables for the evaluation metrics:
TTS, the total and maximum queue length constraint violation, the total input
variation, the soft objective cost and the total computation time.

- `all_agents/` all four control frameworks in all four scenarios

They expect the experiment folders directly in the repository root.

Each experiment folder also has `network_analyzer.m`, which prints the queue
length limit exceedance, `training_analyzer.m` for a
learning curve, and `demand_plotter.m` for a demand profile.
