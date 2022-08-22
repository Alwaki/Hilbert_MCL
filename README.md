# Hilbert_MCL
## Table of contents
* [General info](#general-info)
* [Technologies](#technologies)
* [Setup](#setup)
* [Troubleshooting](#troubleshooting)

## General info
This project simulates Monte Carlo Localization (MCL) on a continuous map, namely the sparse Hilbert map. The main purpose of this software is testing a new likelihood function with this map representation, as well as a A more detailed description can be read in this [article](https://www.google.com). 
	
## Setup
To run this project, first download the repository. Ensure that Matlab is installed, and that a compatible version of Python is available on a path to Matlab (for reference, this program has been tested to work with Matlab R2021b and Python 3.9.10). The program also depends on the following Matlab tools: Navigation Toolbox and the Statistics Toolbox. These can be installed in Matlab from the Add-on explorer.

The software also requires some python libraries are installed. One method of installation is through pip:

```
$ pip install numpy scipy sklearn
```

