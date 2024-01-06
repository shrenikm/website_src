---
title: "Understanding TrajOpt"
date: 2024-01-06T11:11:12-08:00
draft: true
tags: ["control", "implementation", "motion_planning", "numerical_optimization", "robotics", "trajectory_optimization"]
---
<!--
Latex commands
-->
$\newcommand{\st}{\text{s.t.}}$
$\newcommand{\nineq}{n_{ineq}}$
$\newcommand{\neq}{n_{eq}}$
$\newcommand{\x}{\mathrm{x}}$
$\newcommand{\xst}{\mathrm{x}^\ast}$
$\newcommand{\e}{\mathrm{e}}$
$\newcommand{\fc}{\tilde{f}}$
$\newcommand{\fcx}{\tilde{f}(\x)}$
$\newcommand{\gc}{\tilde{g}}$
$\newcommand{\gcx}{\tilde{g}(\x)}$
$\newcommand{\gci}{\tilde{g}_i}$
$\newcommand{\gcxi}{\tilde{g}_i(\x)}$
$\newcommand{\hc}{\tilde{h}}$
$\newcommand{\hcx}{\tilde{h}(\x)}$
$\newcommand{\hci}{\tilde{h}_i}$
$\newcommand{\hcxi}{\tilde{h}_i(\x)}$
<!--
Post
-->
TrajOpt [$[1]$](#references) is an optimization based approach for motion planning. More specifically, it uses a sequential convex optimization procedure along with a formulation for collision constraints to find locally optimal planning trajectories, even for robotic systems that have a large number of degrees of freedom.

This post will go into the details of the optimization part, and give an outline of how it can be implemented. My implementation can be found [here](https://github.com/shrenikm/Atium/tree/main/algorithms/trajopt)


## Theory

Motion planning problems can usually be fully represented as a non-linear, non-convex optimization problem of the form:

\begin{equation}
\begin{aligned}
\min_{\x} f(\x)\\\\
\st \quad g_i(\x) \le 0, \ \ i = 1, 2, \ldots , \nineq\\\\
h_i(\x) = 0, \quad i = 1, 2, \ldots , \nineq\\\\
\end{aligned}
\end{equation}

$f$, $g_i$ and $h_i$ are scalar non-convex functions with $\x$ being the vector of variables to optimize. For trajectory optimization algorithms, $\x$ consists of the sequence of control inputs and/or sequence of states (depending on the method used -- direct, indirect, etc.) that can then be optimized to find the optimal robot trajectory.

Unfortunately, such optimization problems are usually NP-hard and we're almost never guaranteed to find the global optima. Coupled with the fact that for motion planning, we usually require these problems to be solved quickly, most methods to solve such problems tend to gravitate towards using local optimization methods. One such popular method is Sequential Convex Programming (SCP).


### Sequential Convex Programming

The main idea here is to convexify the non-convex problem at the current iterate, and take steps towards the minima found at each convex optimization solution.

\begin{equation}
\begin{aligned}
\min_{\x} \fcx \\\\
\st \quad \gcxi \le 0, \ \ i = 1, 2, \ldots , \nineq\\\\
\hcxi = 0, \ \ i = 1, 2, \ldots , \nineq\\\\
\end{aligned}
\end{equation}

Here, $\fc$, $\gci$ and $\hci$ are the convexified functions at the current value of $\x$ (denoted by $\x^\ast$). The SCP procedure looks something like this:

> **for** iteration = 1, 2, $\ldots$ **do**\
$\quad$ $\fc$, $\gc$, $\hc$ = ConvexifyProblem($f$, $g$, $h$) around $\x^\ast$\
$\quad$ $\x^\ast \leftarrow \text{argmin}_{\x} \fcx \ \st \  \gcx \le 0, \ \hcx = 0$

Fairly straightforward, but there is still something missing. When we convexify the problem, we make a local approximation of the (potentially highly) non-convex function. We rely on the fact that this local approximation is accurate enough and can help drive us to a minima upon repeating the procedure. But irrespective of what approximation method we use, as we move away from the current $\x$ around which the functions are convexified, it will start deviating from true values of the function.

What this means is that when we find the $\x$ that minimizes a particular convexified problem, the actual value of $\x$ that minimizes the problem could be far away from the current value of $\x$. This can lead to the algorithm getting to an updated value of $\x$ that is optimal according to the current convexified problem, but is not optimal according to the true non-convex problem.

To avoid this, SCP methods usually employ an additional trust region constraint, where we restrict the optimal value of $\x$ for a convexified step to be within a certain distance to the current value of $\x$. The distance metric is some sort of norm (usually the $l^2$-norm) in most cases. If we define the trust region size as $s$, we now get:

> **for** iteration = 1, 2, $\ldots$ **do**\
$\quad$ $\fc$, $\gc$, $\hc$ = ConvexifyProblem($f$, $g$, $h$) around $\x^\ast$\
$\quad$ $\x^\ast \leftarrow \text{argmin}_{\x} \fcx \ \st \  \gcx \le 0, \ \hcx = 0, \ \lVert \x - \x^\ast \rVert \le s$

Instead of the $l^2$ ball constraint, we could also use an ellipsoidal constraint of the form $(\x - \x^\ast)^TP(\x - \x^\ast) \le s$ where $P \succ 0$.

### Dynamic Trust Region Size

One strategy is to keep the trust region size small and fixed. This would guarantee that we'd be able to solve any convexified step and reliably get to some sort of local minima. But there are merits to using larger values of trust region size as well.

Consider a convexified step, where the convexified functions are almost an exact representation of the actual non-convex functions. In such a scenario, we could potentially take a large step while minimizing $\x$. Let's say that in this situation a trust region size of $s = 1$ can get us to a new value of $\x$ that also minimizes the original non-convex problem. Compared to us using a smaller $s$ (Like around $1\e{-2}$, $1\e{-3}$), we could get to a similar $\x$ with much fewer iterations.

Dynamically varying the trust region size helps us get the best of both worlds. We can start of using a small trust region size and can progressively increase the size depending on how confident we are with the current convexified approximation. If we think that the convexified approximation is an accurate representation of the original problem around a larger region of $\x$, we can increase the trust region size and vice versa.

How do we know if the current problem approximation is a good one? We can use the new optimized value of $\x$ from the convexified problem and check how much it improves the original cost function as compared to the convexified cost function.

> TrueImprove = $f(\x) - f(\xst)$\
ModelImprove = $\fcx - \fc(\xst)$

As we're minimizing the cost function, we expect the cost at the newly minimized value at $\xst$ to be lower, and so the difference tells us how much of an improvement there is.

As we're minimizing the convexified cost function, it is a given that **_ModelImprove >= TrueImprove_**, but by computing the ratio of these terms, we can get an estimate of how much the original cost function improves upon solving the convexified problem.

> IsImprovement\
$\quad$ TrueImprove = $f(\x) - f(\xst)$\
$\quad$ ModelImprove = $\fcx - \fc(\xst)$\
$\quad$ return TrueImprove/ModelImprove > c

Let's consider two extreme cases:
1. The convexified problem is an exact representation of the non-convex problem (Maybe the original problem was even convex). In this case, $\xst$ will improve the original problem by the same amount as it improves the convexified problem. The ratio is 1 in this case.
2. The convexified problem is an extremely poor representation of the non-convex problem. In this case $\xst$ will not improve the original problem by nearly as much. The ratio is close to 0 in this case.

We define some $c \le 1$ (Referred to as step acceptance parameter in the paper) and make an assumption that if the ratio is $> c$, the approximation is good and we can increase the trust region size. If not, we decrease the trust region size and don't use the new $\xst$.


## References

1. **Schulman, J., Duan, Y., Ho, J., Lee, A., Awwal, I., Bradlow, H., ... & Abbeel, P. (2014). Motion planning with sequential convex optimization and convex collision checking. The International Journal of Robotics Research, 33(9), 1251-1270.** [[Link]](https://journals.sagepub.com/doi/abs/10.1177/0278364914528132)
