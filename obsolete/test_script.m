a = DoubleIntegrator('1', [2; 2; 0; 0; 10]);
b = DoubleIntegrator('2', [1; 1; 0; 0; 10]);

p = AirTrafficProblem([a, b], 10);

[x_opt, u_opt, exitval_opt] = p.RunMPC();

p.PlotSolution()


