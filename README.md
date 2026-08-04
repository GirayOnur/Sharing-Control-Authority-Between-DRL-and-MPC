# Sharing the Control Authority Between Deep Reinforcement Learning and Model Predictive Control: Application to Multi-Class Transportation Networks

Freeway traffic control where two inputs are handled by two different
controllers. A reinforcement learning agent does the fast job (ramp metering,
once a minute) and an MPC does the slow one (route guidance, once every five
minutes). The experiments check whether splitting the inputs this way holds up
against doing everything with MPC, and how each option copes when the model or
the demand is not what the controller expects.

All MATLAB. The traffic model is METANET, written out by hand, and the MPC is a
plain `fmincon` problem, not the MPC Toolbox.

## The network

```
                                     o2
                                      v
                  +--> [2_1] --> [3_1 3_2] --> out    route 1
o1 --> [1_1 1_2 1_3]
                  +--> [4_1] --> [5_1 5_2] --> out    route 2
                                      ^
                                     o3
```

Nine segments, three origins, two vehicle classes, 75 states in one vector `x`
(state order is in the header of `fun_benchmark_RM.m`). Three inputs, all
between 0 and 1: the split rate that sends traffic to route 1, set every 5 min,
and the two ramp rates, set every 1 min.

One step is 10 seconds and a run is 900 steps, after a 60 step warm-up so no
controller starts from an empty road. Everything minimises total time spent
(TTS) plus a small penalty for jerking the inputs around, with a length limit
on the queues.

## Controllers and scenarios

| name | ramp metering | route guidance |
| --- | --- | --- |
| No control | ramps open | fixed 50/50 split |
| SF-MPC | PI-ALINEA | MPC |
| Hierarchical MPC | MPC | MPC |
| DDPG-MPC | DDPG agent | MPC |
| SAC-MPC | SAC agent | MPC |
| SACD-MPC | same SAC agent, deterministic | MPC |

The point of the DRL and PI-ALINEA versions is that the route guidance MPC does
not assume the ramp controller is frozen. It steps the agent (or the PI-ALINEA
law) forward inside its own prediction, so it optimises against what the fast
level will actually do. See `MPC_solve_SR_w_RL.m`. The hierarchical MPC
baseline just holds the last ramp rates.

Each controller runs in four scenarios, which is what the folder suffixes mean:
nothing for the nominal case, `_nd` for noisy demand, `_mm` for a controller
model that is wrong on purpose, and `_mm_nd` for both.

## Layout

```
DDPG_MPC/         DDPGpi_main_s5_<run><suffix>
SAC_MPC/          SACpi_main_s5_<run><suffix>
SACD_MPC/         SACDpi_main_s5_<run><suffix>
PI_ALIENA_MPC/    PI_ALINEA_MPC<suffix>_<run>
plots/comparison/ figures, tables and the scripts that make them
```

`<run>` is 1 to 5: five independently trained agents (five separate tuning runs
for PI-ALINEA), each simulated 10 times with seeds 1 to 10. No control and
hierarchical MPC do not need an agent, so they only run in the
`DDPGpi_main_s5_1*` folders.

The experiment folders are near-copies of each other, each self-contained so it
can run on its own. Inside one, the files worth reading are
`fun_benchmark_RM.m` (one step of the traffic model) and the `calc_*.m`
equations it is built from, `param_get.m` and `param_MPC_get.m` for the
settings, `demando1/2/3.m` for the demand, `MPC_solve_*.m` for the MPC
problems, `const_RL.m` with `rlStepFunc.m` and `rlResFunc.m` for the agent and
its environment, and `benchmark_*.m` for one experiment run.

## Running

Needs MATLAB with the Optimization, Reinforcement Learning, Statistics and
Machine Learning, Signal Processing and Parallel Computing toolboxes. Without
the last one everything still runs, only slower. The MPC Toolbox is not needed.

Trained agents, tuned PI-ALINEA gains and the initial states are all in the
repository, so nothing has to be retrained. `cd` into an experiment folder and
run the benchmark script in it, for example `benchmark_RL_MPC_SR_RM`. For ten
runs with seeds 1 to 10 and a diary of the output, use `run_10_experiments`.

It is slow: every MPC step solves a non-linear problem from several starting
points, and there are 900 steps per run.

Results land next to the script as `<controller>_result_<timestamp>.mat`. They
hold the raw state history `xx` and input history `uu`; TTS is recomputed from
those by the plotting scripts, it is not stored.

To train instead of reusing an agent, `train_RL_MPC` sets up the starting point
and then runs `const_RL.m`. This takes hours, mostly because the MPC runs inside
the training environment, so every agent step also solves an optimisation
problem.

## Results

The scripts in `plots/comparison` read the result files out of the experiment
folders and produce the figures and LaTeX tables: `all_agents/` for all six
controllers in all four scenarios, `single_agent/` for one agent per method,
`multi_agents/` for the spread across the five agents, `extra_experiments/` for
agents trained without noise and tested with it.

They expect the experiment folders directly in the repository root, from before
they were grouped into `DDPG_MPC/`, `SAC_MPC/` and so on. Move them back up or
fix the paths in the scripts.

There are also a few small scripts in each experiment folder:
`network_analyzer.m` prints the queue violations of the last run,
`training_analyzer.m` plots a training curve, `demand_plotter.m` plots a demand
profile.
