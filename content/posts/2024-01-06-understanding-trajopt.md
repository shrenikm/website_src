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
$\newcommand{\fx}{f(\x)}$
$\newcommand{\fc}{\tilde{f}}$
$\newcommand{\fcx}{\tilde{f}(\x)}$
$\newcommand{\gx}{g(\x)}$
$\newcommand{\gc}{\tilde{g}}$
$\newcommand{\gcx}{\tilde{g}(\x)}$
$\newcommand{\gxi}{g_i(\x)}$
$\newcommand{\gci}{\tilde{g}_i}$
$\newcommand{\gcxi}{\tilde{g}_i(\x)}$
$\newcommand{\hx}{h(\x)}$
$\newcommand{\hc}{\tilde{h}}$
$\newcommand{\hcx}{\tilde{h}(\x)}$
$\newcommand{\hxi}{h_i(\x)}$
$\newcommand{\hci}{\tilde{h}_i}$
$\newcommand{\hcxi}{\tilde{h}_i(\x)}$
$\newcommand{\tp}{\tau^{+}}$
$\newcommand{\tm}{\tau^{-}}$
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
$\quad$ $\x^\ast \leftarrow \text{argmin}_{\x} \fcx \ \st \  \gcx \le 0, \ \hcx = 0, \ \lVert \x - \x^\ast \rVert_2 \le s$

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
$\quad$ **return** TrueImprove/ModelImprove > c

Let's consider two extreme cases:
1. The convexified problem is an exact representation of the non-convex problem (Maybe the original problem was even convex). In this case, $\xst$ will improve the original problem by the same amount as it improves the convexified problem. The ratio is 1 in this case.
2. The convexified problem is an extremely poor representation of the non-convex problem. In this case $\xst$ will not improve the original problem by nearly as much. The ratio is close to 0 in this case.

We define some $c \le 1$ (Referred to as step acceptance parameter in the paper) and make an assumption that if the ratio is $> c$, the approximation is good and we can increase the trust region size. If not, we decrease the trust region size and don't use the new $\xst$.

This now brings us to the something similar to trust region loop as defined in the paper (more details on this later):

> TrustRegionLoop\
**for** TrustRegionIteration = 1, 2, $\ldots$ **do**\
$\quad$ $\x^\ast \leftarrow \text{argmin}_{\x} \fcx \ \st \  \gcx \le 0, \ \hcx = 0, \ \lVert \x - \x^\ast \rVert_2 \le s$\
$\quad$ **if** IsImprovement **then**\
$\qquad$ $s \leftarrow \tp \ast s$\
$\qquad$ **break**\
$\quad$ **else**\
$\qquad$ $s \leftarrow \tm \ast s$\
$\quad$ **if** $s < xtol$ **then**\
$\qquad$ **break**

Where we define $\tp \ge 1$ and $\tm \le 1$ as trust region expansion and shrinkage factors that decide how much the trust region expands or shrinks at each step.

If the trust region becomes too small, the problem will be numerically be stationary at a certain point. $xtol$ defines this smallest trust region size beyond which we break out of the loop. It also serves as the convergence threshold which we will get to next.


### Convergence

The SCP also requires some criteria to stop the trust region loop. Usually this is defined as some threshold between the successive solutions of $\x$. Usually when we're at a local/global minima, the gradients are zero and differences between successive solutions are minimal.

Depending on the function, we could also end up in a situation where a region around the minima corresponds to the same or similar cost values. So it's generally a good idea to also have a convergence criteria on the value of the cost function as these would also stop moving when we're at a minima.

> IsConverged\
$\quad$ **return** $\lVert \xst - \x \rVert_2 < xtol$ **or** $\lVert f(\xst) - f(\x) \rVert_2 < ftol$

Where $xtol$ and $ftol$ are the corresponding convergence thresholds.

### Constraints

Something that still needs to be addressed is the fact that we have non-convex constraints in the original problem. Solving the convexified problem with convexified constraints can still be tricky.

In the paper, these are added into the cost function as penalties rather than as constraints to the problem directly. Doing it this way simplifies things a lot as we can restrict the actual constraints to being linear and have the other terms as penalties in the cost function. We'll go over this in detail in the [implementation](#implementation) section, so for now we focus on how the penalties are formulated.

Consider the non-convex inequality constraints of the form $\gxi \le 0$.\
In order to add this as a penalty in the cost function, we need to map it to a function that has a minimum at $0$ whenever the constraints are satisfied, and is positive everywhere else.\
The paper uses $|\gxi|^+$ Where
\begin{equation}
|\gxi|^+ = \max(\x, 0)
\end{equation}
Whenever the constraints are satisfied, $\gxi \le 0$ which means that $|\gxi|^+ = 0$. So if we multiply this term by a an extremeley large penalty factor and add it to the cost function, it would force this term to go to zero (and hence satisfy the constraints) when the problem is solved. The minimizer of the penalized problem would be equal to the minimizer of the original problem with constraints.

Similary for the non-convex equality functions $\hxi = 0$, we have a similar penalty function:
\begin{equation}
|\hxi|
\end{equation}

Where $|\hxi|$ is just the absolute value of $\hxi$. Similar to the inequality function, this penalty function also has a minima at $0$ when the constraints are satisfied and is positive everywhere else. These constraints can also be multiplied by a penalty factor and incorporated into the cost function.

The cost function of the convexified problem which initially was

\begin{equation}
\begin{aligned}
\min_{\x} \fcx \\\\
\st \quad \gcxi \le 0, \ \ i = 1, 2, \ldots , \nineq\\\\
\hcxi = 0, \ \ i = 1, 2, \ldots , \nineq\\\\
\end{aligned}
\end{equation}

Now has a more numerically tractable form:

\begin{equation}
\begin{aligned}
\min_{\x} \fcx + \mu \sum_i^{\nineq} |\gcxi|^+ + \mu\sum_i^{\neq}|\hcxi|\\\\
\st \quad \lVert \x - \xst \rVert_2 \le s\\\\
\end{aligned}
\end{equation}

Given the current trust region size $s$ and the penalty factor $\mu$. Larger the value of $\mu$, the more likely it is for the optimization to prioritize satisfying the constraints over minimizing the pure cost $\fx$. When $\mu$ is very large (theoretically close to $\infty$), solving the problem would attempt to satisfy the constraints and drive this down at any cost.

In order to check if the given constraints are satisfied, we have another procedure using a constraint satisfaction threshold $ctol$ (Where $0 < ctol \ll 1$). The threshold helps us relax the satisfaction requirements a bit when we actually implement it numerically. The check in practice becomes $\gxi \le ctol$ and $\lVert \hxi \rVert_2 \le ctol$

> AreConstraintsSatisfied\
$\quad$ ***for*** $i=1, 2, \ldots, \nineq$\
$\qquad$ **if** $\gxi > ctol$ **then**\
$\qquad$ $\quad$ **return** False\
$\quad$ ***for*** $i=1, 2, \ldots, \neq$\
$\qquad$ **if** $\lVert \hxi \rVert_2 > ctol$ **then**\
$\qquad$ $\quad$ **return** False\
$\quad$ **return** True

It is important to note that the actual versions of the constraint functions are used here, and not the convexified versions.

### Putting It All Together

Finally, we can take a look at how the paper integrates the penalty check, convexification and trust region loops.

1. The first step is to convexify the problem at the given $\xst$ and start the SCP trust region loop.
2. Depending on whether the new solution to the convexified problem is considered an improvement to the original cost function or not, we dynamically expand or shrink the trust region size. Ideally we continue this process until convergence, but what exactly do we need to do post trust region size modification?
    - **Expansion**: When the $\x$ minimizer solution is accepted, we want to increase the trust region size and re-convexify the problem at the new $\xst$. We would keep the new expanded trust region size and continue the SCP process.
    - **Shrinkage**: When the solution is not accepted, the trust region size is reduced and we don't update $\xst$. There's no need to convexify the problem as $\x$ hasn't changed so we can reduce the trust region and try solving the convexified problem again. We keep repeating this until:
        - The trust region size $s < xtol$ after which we need to break out of the loop and check for constraint satisfaction
        - We reach the convergence criteria and we break out of the loop

        All in all, we repeat the loop for a convexified version of the problem until a solution is accepted or the trust region becomes too small. Once we have a new solution, the problem is convexified again and the process is repeated.

3. Where do the penalties fit into all this? The convergence check only ensures that we have found a local optima for the current convexified version of the problem. It does not imply that $\xst$ satisfies the required constraints due to the nature of how the constraints are treated as cost function penalties. After each SCP iteration has converged, we check if the constraints are satisfied. There are only two scenarios:
    - **Satisfied**: If the SCP has converged and the constraints are satisfied, we have satisfied the requirements of the problem. We have minimized the object (To the best of our numerical abilities, around a local convex estimate) and we have satisfied the required constraints. We can return the solution $\xst$ as the final minimizer of the optimization problem.
    - **Unsatisfied**: If the SCP has converged and the constraints are not satisfied, we obviously need to start over, but what do we do differently? This is where the penalty factor comes into play. In this case, we can increase the penalty factor $\mu$ by some $k > 1$ by setting $\mu \leftarrow k \ast \mu$. We don't have to start over from the initial value of $\x$ as the last SCP iteration converged and the only issues are the constraints not being satisfied. We can resume the SCP using the last updated values of $\xst$ and $s$, but with the new penalty factor $\mu$.


The entire algorithm now looks something like:

> **for** PenaltyIteration = 1, 2, $\ldots$ **do**\
$\quad$ **for** ConvexifyIteration = 1, 2, $\ldots$ **do**\
$\qquad$ $\fc$, $\gc$, $\hc$ = ConvexifyProblem($f$, $g$, $h$)\
$\qquad$ **execute** TrustRegionLoop\
$\qquad$ **if** IsConverged **then**\
$\qquad$ $\quad$ **break**\
$\quad$ **if** AreConstraintsSatisfied **then**\
$\qquad$ **return** $\xst$\
$\quad$ **else**\
$\qquad$ $\mu \leftarrow k \ast \mu$


## Implementation

Now we get to the fun part of implementing it. In this section, we'll go over how all of this can be implemented, along with details that were skipped in the paper. For reference, my python version of the algorithm can be found [here](https://github.com/shrenikm/Atium/tree/main/algorithms/trajopt).

### Convexification

How do we actually go about convexifying the problem? In the paper, each of the functions are expanded about the current $\x$ value using the Taylor Series. After doing this for each function, we can convert the non-convex optimization problem in to a Quadratic Program (QP) at each step.

Expanding the cost function $\fx$ around $\xst$, we get

\begin{equation}
\end{equation}



## Evaluation


## References

1. **Schulman, J., Duan, Y., Ho, J., Lee, A., Awwal, I., Bradlow, H., ... & Abbeel, P. (2014). Motion planning with sequential convex optimization and convex collision checking. The International Journal of Robotics Research, 33(9), 1251-1270.** [[Link]](https://journals.sagepub.com/doi/abs/10.1177/0278364914528132)
