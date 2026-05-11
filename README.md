# Dividing Control Inputs Between DRL and MPC

MATLAB experiment folders for comparing no control, PI-ALINEA, hierarchical
MPC, DDPG-MPC, SAC-MPC, and deterministic SAC-MPC controllers.

## Repository Layout

- `DDPGpi_main_s5_*`, `SACpi_main_s5_*`, and `SACDpi_main_s5_*`: DRL/MPC
  experiment folders.
- `PI_ALINEA_MPC*`: PI-ALINEA benchmark folders.
- `run_all_experiments.m`: batch runner for all experiment sets.
- `analysis/comparison`: comparison plot/table scripts and existing exported
  figures/tables.

## Running On Another PC

1. Clone the repository.
2. Open MATLAB in the repository root.
3. Make sure the needed MATLAB toolboxes are installed, especially the
   toolboxes used by the scripts for reinforcement learning, optimization,
   MPC, and parallel execution.
4. Run:

   ```matlab
   run_all_experiments
   ```

The benchmark result `.mat` files and diary files are generated locally and
are ignored by git. Required setup files such as `agent_*.mat`,
`base_demands.mat`, `net_init.mat`, and `Bayes_opt_*.mat` are tracked.

## Analyzing Results

After the experiments finish, run the scripts under:

- `analysis/comparison/all_agents`
- `analysis/comparison/single_agent`
- `analysis/comparison/multi_agents`

The comparison scripts resolve paths relative to the cloned repository, so
they do not depend on the original `/home/giray/...` path.
