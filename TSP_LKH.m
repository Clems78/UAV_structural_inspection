% Define the working directory where your files are located
workingDir = 'C:\Users\cleme\OneDrive - Cranfield University\IRP\Dev\Matlab\';

% Load and prepare the distance matrix
dist_opt = pdist(wp_opt, 'euclidean');
dist_opt = squareform(dist_opt);

% Set up parameters
pars_struct.CostMatrixMulFactor = 1000;
pars_struct.user_comment = "test";

% Define file names and directories
fname_tsp = [workingDir, 'TSP_LKH_SOL.mat'];  % Full path for the TSP solution
LKHdir = 'C:\Users\cleme\OneDrive - Cranfield University\IRP\Dev\Matlab\LKHWin-2.0.10\LKHWin-2.0.10';  % Directory where LKH executable is located
TSPLIBdir = workingDir;  % Directory where TSPLIB files are located

% Run the TSP solver
TSPsolution = LKH_TSP(dist_opt, pars_struct, fname_tsp, LKHdir, TSPLIBdir);
