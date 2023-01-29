# Task-Specific Synthesis and Design of a mobile 6-DoF Hexa Parallel Robot for Weed Control


Authors: Tim Sterneck, Jannik Fettin, and Moritz Schappler

This code is supplemental material to the following publication at the [I4SDG Workshop 2023](https://www.iftommitaly.it/i4sdg2023/):

```bib
@InProceedings{SterneckFetSch2023,
  author       = {Sterneck, Tim and Fettin, Jannik and Schappler, Moritz },
  booktitle    = {Proceedings of the 2nd IFToMM for Sustainable Development Goals Workshop},
  title        = {Task-Specific Synthesis and Design of a mobile 6-DoF Hexa Parallel Robot for Weed Control},
  note         = {submitted for publication},
  organization = {Springer},
  date         = {2023},
}
```

## Contents and Useage

This repository contains Matlab scripts and Latex code to reproduce results and figures of the paper. Beforehand, the steps from the [prerequesites section](#prerequisites) have to be performed.
  
1. To reproduce the results, first the dimensional synthesis has to be performed by running the Matlab script `dimsynth/config_amun.m`.
    * If existing results shall be evaluated, their location has to be set with `i4sdg2023_dimsynth_data_dir.m`. Then this step can be omitted
    * The results that were used in the paper are stored in `data/synthesis_results` and are used by default
2. The synthesis' results have to be post-processed by the scripts from `dimsynth_postprocess` (in this order):
    * `eval_figures_pareto.m`: Assemble all Pareto fronts for the individual robots
    * `robot_names.m`: The names of the robots are assembled (e.g. for the figure captions); only once.
    * `eval_existing_design.m` uses the synthesis output and computes the performance of the actual designed prototype described in **the text of Sec. 4 of the paper**
    * `eval_figures_pareto_groups.m`: Group the robots to a smaller set.
    * `select_eval_robot_examples.m`: Select the specific robot structures from the Pareto front for a detailed view
3. The robot figures and detail information is reproduced with the Matlab scripts from `paper/figures`:
    * `robot_images.m`: Creates one image file for each robot. These are taken in `robots.svg` to create **Fig. 2 the paper**. Run `robots/config_robot_figs.m` for additional formatting.
	* The Pareto diagram of **Fig. 3,a of the paper** is already formatted by `eval_figures_pareto.m`
    * Two redundancy map of **Fig. 3,b of the paper** is formatted by `perfmap_fig.m`.
	* The workspace figures of Fig. 5 are created by a script which is not ready for publication yet.

The steps 2 to 3 can be performed after each iteration of the dimensional synthesis by using `run_evaluation_i4sdg2023.m` in the paper repo's root directory.

## Prerequisites <a name="prerequisites">

For the Matlab scripts to work, the following steps have to be performed:

1. Install Matlab (tested with R2022b)
2. Set up the Matlab mex compiler, if not already done
3. Set up the path to the results for this paper by copying `i4sdg2023_dimsynth_data_dir.m.template` without the template suffix to the same location and by changing the content of the file to point to the stored results of the synthesis.
4. Download Matlab dependencies:
    * [Toolbox for geometry functions](https://github.com/SchapplM/robotics-dep-geometry) (tested with rev. c4af53d; 2021-10-26)
    * [Toolbox for trajectory functions](https://github.com/SchapplM/robotics-dep-traj) (tested with rev. 42c8302; 2022-02-09))
    * [Collection of external dependencies](https://github.com/SchapplM/robotics-dep-ext) (tested with rev. d8d29ff; 2022-08-26))
    * [Toolbox for various Matlab functions](https://github.com/SchapplM/matlab_toolbox) (tested with rev. 0654bba; 2022-04-05))
    * [Robotics toolbox](https://github.com/SchapplM/robotics-toolbox) (tested with rev. 24e9a94; 2023-01-29))
    * [Serial robot model database](https://github.com/SchapplM/robsynth-serroblib) (tested with rev. 36ab3b167; 2023-01-20))
    * [Parallel robot model database](https://github.com/SchapplM/robsynth-parroblib) (tested with rev. 7c31c32; 2023-01-24))
    * [Toolbox for symbolic robot model generation](https://github.com/SchapplM/robsynth-modelgen) (tested with rev. fd37713; 2022-01-20)
    * [Toolbox for transferring jobs to a computing cluster](https://github.com/SchapplM/matlab_pbs_transfer) (tested with rev. a4bb277; 2022-12-07))
    * [Dimensional synthesis toolbox](https://github.com/SchapplM/robsynth-structdimsynth) (tested with rev. 9c24493; 2023-01-29))
    * Update the repositories to the given versions. You should first try the more recent version before downgrading to the given versions. It should still work.
    * Short command to download all repositories (run in Git Bash in Windows or in Linux terminal):
      ```Bash
      git clone git@github.com:SchapplM/robotics-dep-geometry.git
      git clone git@github.com:SchapplM/robotics-dep-traj.git
      git clone git@github.com:SchapplM/robotics-dep-ext.git -b dev
      git clone git@github.com:SchapplM/matlab_toolbox.git -b dev
      git clone git@github.com:SchapplM/robotics-toolbox.git -b dev
      git clone git@github.com:SchapplM/robsynth-serroblib.git -b dev
      git clone git@github.com:SchapplM/robsynth-parroblib.git -b dev
      git clone git@github.com:SchapplM/robsynth-modelgen.git
      git clone git@github.com:SchapplM/matlab_pbs_transfer.git
      git clone git@github.com:SchapplM/robsynth-structdimsynth.git -b dev
      ```
5. Set up the toolboxes by modifying the following files according to the instructions in the respective README files
    * robsynth-serroblib/maplerepo_path.m.template (has to link to robsynth-modelgen)
    * robsynth-structdimsynth/computingcluster_repo_path.m.template (has to link to matlab_pbs_transfer, if a cluster is used for computation)
6. Run the path initialization scripts (`..._path_init.m`) of all downloaded software repos in Matlab. Put the run commands in the startup.m file of Matlab if used frequently.

