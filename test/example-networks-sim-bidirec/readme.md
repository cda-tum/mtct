## Explanation Bidirectional Networks

The continuous simulator cannot export solutions back to the original network used to generate its unidirectional representation (from convertToUnidirec.py),
since information about directional edges is lost.
Thus we create a bidirectional version (has all reciprocal edges) of the original with convertToBidirec.py.
We can then export the simulator solution (TrainTrajectorySet) to a SolGeneralPerformanceOptimizationInstance for the bidirectional network using to_vss_solution().
