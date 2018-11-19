a = DoubleIntegrator([2; 2; 0; 0; 10], [0; 0; 0; 0; 1]);
b = DoubleIntegrator([1; 1; 0; 0; 10], [2; 2; 0; 0; 1]);

p = AirTrafficProblem([a, b], 10);

[x_opt, u_opt, exitval_opt] = p.RunMPC();


