---
title: "Dimension of a proper subspace"
date: 2021-08-29T10:20:38-07:00
draft: false
tags: ["linear_algebra", "math"]
---
<!-- Post 1 -->
Let $V$ be a finite dimensional vector space over the field $F$, with $\dim{V} = n$.  
We also define the ordered basis for $V$ to be $B = \set{\beta_1, \ldots, \beta_n}$.

Consider a subspace $U$ of the vector space $V \subseteq U$. Naturally, we have $\dim{U} \leq \dim{V}$.  

This is because for any $\alpha \in U$, we have $\alpha \in V$, which can then be expressed as a linear combination of the basis vectors in $B$. Hence, we don't require more than $n$ linearly independent vectors to represent any vector in $U$.

Now what if $U$ is a proper subspace, $U \subsetneq V$?

At first glance, it might seem like we still have $\dim{U} \leq \dim{V}$, but in fact we have a strict inequality $\dim{U} \lt \dim{V}$.

To see this, we let $\dim{U} = m$. Let the ordered basis of $U$ be $A = \set{\alpha_1, \ldots, \alpha_m}$.  
As $U$ is a proper subspace, we can choose an element $\gamma \in V$ such that $\gamma \not\in U$. Intuitively, the vectors $\set{\alpha_1, \ldots, \alpha_m, \gamma}$ are linearly independent as $\gamma$ cannot be expressed as a linear combination of the vectors in $A$. To see this mathematically, consider the following linear combination of the basis vectors in $A$ and $\gamma$ with scalars $c_i$ taken from the field $F$:

\begin{equation}
c_1\alpha_1 + \ldots + c_m\alpha_m + c_{m+1}\gamma = 0
\end{equation}

For this to be satisfied, we must have all $c_i = 0$. To prove this, we first assume that $c_{m+1} \neq 0$. We can now multiply both sides by $\frac{1}{c_{m+1}}$ (Which is possible because $\frac{1}{c_{m+1}}$ is also an element of the field, for $c_{m+1} \neq 0$).

\begin{equation}
\frac{c_1}{c_{m+1}}\alpha_1 + \ldots + \frac{c_m}{c_{m+1}}\alpha_m + \gamma = 0
\end{equation}

The above equation means that either $\gamma$ can be expressed as a linear combination of the basis vectors of $U$ (If some of the $c_i \neq 0$), or $\gamma = 0$. Both of these would mean that $\gamma \in U$, which is not the case as we have specifically chosen $\gamma \not\in U$. Hence this means that $c_{m+1}$ cannot be non-zero.

\begin{equation}
c_{m+1} = 0 \implies c_1\alpha_1 + \ldots + c_m\alpha_m = 0
\end{equation}

As $\set{\alpha_1, \ldots, \alpha_m}$ is the basis for $U$, they are linearly independent, and we must have $c_i = 0, i \in [1, m]$ in the above equation.

So for the linear combination of the basis vectors in $A$ and $\gamma$ to be equal to $0$, we must have every $c_i = 0, i \in [1, m+1]$, implying that the basis vectors in $A$ along with $\gamma$ are linearly independent.

We now have a set $C = \set{\alpha_1, \ldots, \alpha_m, \gamma}$ of linearly independent vectors within $V$.
Hence $\dim{V} \ge m+1$ which means that $\dim{V} \gt m$, giving us the strict inequality between the dimensions of both vector spaces:
\begin{equation}
\dim{U} \lt \dim{V}
\end{equation}



