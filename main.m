% Project:          Monte Carlo Localization with Hilbert Maps as
%                   Likelihood Fields 
%
% Description:      This program simulates runs of Monte Carlo
%                   Localization (MCL) throughout random 2D maps with a
%                   range based sensor. Different from traditional MCL with
%                   grid maps, this method utilizes a continuous map in the
%                   form of a classifier. Specifically, Hilbert maps are
%                   used. These continuous maps are also utilized as 
%                   observation likelihood models, and this model is
%                   compared to using beam likelihood models.
%
% Author:           Alexander Wallen Kiessling
%
% Version:          Version 2.1 (June 2023)
%

% NOTE: Changing parameters is done in src/Simulation/parameters

% Setup (set paths, load parameters, create containers)
setup;

% Main simulation entry point
if parallel_flag 
    results = runParallel(params);
else
    results = runSequential(params);
end

