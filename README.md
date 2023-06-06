# Hilbert_MCL
## Table of contents
* [General info](#general-info)
* [Setup](#setup)
* [Running](#running)

## General info
This project simulates Monte Carlo Localization (MCL) on a continuous map, namely the sparse Hilbert map. The main purpose of this software is testing a new likelihood function with this map representation, as well as experimenting if continuous maps offer robustness against poor sensor accuracy. A more detailed description of the project can be read in this [article]([https://www.google.com](https://www.diva-portal.org/smash/record.jsf?pid=diva2%3A1737469&dswid=-4207)). 
	
## Setup
To run this project, first download the repository. Ensure that Matlab is installed, and that a compatible version of Python is available on a path to Matlab (for reference, this program has been tested to work with Matlab R2021b and Python 3.9.10). The program also depends on the following Matlab tools: Navigation Toolbox and the Statistics Toolbox. These can be installed in Matlab from the Add-on explorer.

The software also requires some python libraries to be installed. One method of installation is through pip:

```
$ pip install numpy scipy sklearn
```
## Running
To run the code, simply execute the main.m script. This script is outside any folder structure, for easy access. To tune/change any parameters, open src/Simulation/parameters. Here are all parameters that can be tuned. While there are rudimentary descriptions of the parameters, these are also described in more depth in the appendix A of the above article.

