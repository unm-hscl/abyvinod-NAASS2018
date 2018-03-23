# Code for bi-criterion optimization with safety as a criterion (Spacecraft docking problem)

Paper title     | Optimal trade-off analysis for efficiency and safety in the spacecraft rendezvous and docking problem
----------------|------------------------------------------------------------
Conference      | NAASS 2018, Santa Fe, NM, USA
Code author     | Abraham P. Vinod
Code language   | MATLAB
Dependencies    | CVX, MATLAB's Global Optimization toolbox, MPT3

## Instructions

1. Clone this repository or download the zip from https://github.com/unm-hscl/abyvinod-NAASS2018/archive/v1.zip
1. Run `FigureSubSect42.m` and `FigureSubSect43.m` to get the plots used in the paper (Uses data files stored in `data` folder)
1. Run `multicriterion_CWH.m` to generate fresh data for `FigureSubSect42.m` (Requires edits to the filename used in `FigureSubSect42.m`)
1. Run `scan_through_umax.m` to generate fresh data for `FigureSubSect43.m` (Requires edits to the filename used in `FigureSubSect42.m`)

