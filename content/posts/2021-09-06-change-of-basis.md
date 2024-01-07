---
title: "Change of basis"
date: 2021-09-06T22:32:43-07:00
draft: false
tags: ["linear_algebra", "math"]
---
<!-- Post 2 -->
We look into the problem of expressing a vector $p$, whose coordinates relative to some basis is known, through the coordinates relative to a different basis.

Let's assume that $p$ lies in an $n$ dimensional vector space $V$ defined over the field $F$. Let $A$ and $B$ be two different sets of basis vectors containing the vectors $\set{\alpha_1, \ldots, \alpha_n}$ and $\set{\beta_1, \ldots, \beta_n}$ respectively.

Let $p_A$ represent the coordinates of $p$ with respect to the basis $A$. Now, how do we go about computing $p_B$?

If the coordinates of $p_A$ are $(a_1, \ldots, a_n)$ (With $a_i \in F$), we have:

\begin{equation}
p = a_1\alpha_1 + \ldots + a_n\alpha_n = \sum_{i=1}^{n}a_i\alpha_i
\end{equation}

Assume that the coordinates of $p_B$ are $(b_1, \ldots, b_n)$. It is important to note that while representing vectors through coordinates relative to a basis, the actual vector remains unchanged. Changing the basis merely changes the representation of the vector by changing the scalars that constitute the linear combination of the basis. So when the basis changes, the coordinates also change such that the new linear combination also represents the same vector.

\begin{equation}
p = \sum_{i=1}^{n}a_i\alpha_i = \sum_{i=1}^{n}b_i\beta_i
\end{equation}

Being basis in the same vector space, we can represent the basis vectors of $A$ as linear combinations of the vectors in $B$.

\begin{equation}
\alpha_i = \sum_{j=1}^{n}c_{ij}\beta_j \ \ \ \ \ c_{ij} \in F
\end{equation}

Using this in the previous equation, we get:

\begin{equation}
p = \sum_{i=1}^{n}\sum_{j=1}^{n}a_ic_{ij}\beta_j = \sum_{i=1}^{n}b_i\beta_i
\end{equation}

The above equation makes it clear that the each of new coordinates $b_i$ can be obtained through some linear combination of the known coordinates $a_i$. What this means is that we have a matrix composed of some scalars that can transform the old coordinates to the new ones: $p_B = Pp_A$.

What does this matrix $P$ consist of exactly? We can go ahead and fill up the matrix with the $c_{ij}$ scalars, but this isn't very intuitive and doesn't tell us much about the matrix itself without careful inspection. Let's try something more intuitive.

Imagine that instead of the basis vectors $\alpha_i$, we use $\alpha_{iB}$ which represent the basis vectors as coordinates with respect to $B$.
Now what does $\sum_{i=1}^{n}a_i\alpha_{iB}$ give us? As each of the basis is represented relative to $B$, it should give us the vector $p$ relative to $B$, or $p_B$. The matrix transformation $p_B = Pp_A$ now becomes:

\begin{equation}
\begin{bmatrix}
b_1\\\\
\vdots\\\\
b_n
\end{bmatrix} = \begin{bmatrix}
\alpha_{1B} & \ldots & \alpha_{nB}
\end{bmatrix}\begin{bmatrix}
a_1\\\\
\vdots\\\\
a_n
\end{bmatrix}
\end{equation}

Hence the columns of $P$ consists of the basis vectors $\alpha_i$ represented as coordinates in the frame $B$. With this representation, it is quite easy to see that the matrix $P$ is invertible, as its columns comprise of representations of the basis which are independent. So we also have:

\begin{equation}
p_A = P^{-1}p_B
\end{equation}

To summarize, given a coordinate of a vector $p_A$, to represent it relative to a different basis $B$, all we have to do is represent each of the basis vectors in the original basis $\alpha_i$ relative to the new basis and set these as columns of a matrix $P$. Multiplying $P$ and $p_A$ will give us the required $p_B$.

<br />

### Example

Let's go over a quick example showcasing this change of basis. Consider the two dimensional vector space $\mathbb{R}^2$ over the field of real numbers $\mathbb{R}$. Let the standard basis for this space be $A = \set{\alpha_1, \alpha_2} = \set{[1, 0], [0, 1]}$

Consider a vector $p = [5, 0]$. Using the standard basis, we have $p_A = (5, 0)$.

Introducing a new basis $B = \set{\beta_1, \beta_2} = {[0, -1], [1, 0]}$, we want to find the representation of $p$ relative to $B$. If the standard basis $A$ represents vectors along the $x$ and $y$ axis, $B$ represents vectors along the $-y$ and $x$ axis. Geometrically, we can say that that coordinates with respect to $B$ are rotated clockwise by an angle of $90^\circ$ ($-90^\circ$ using positive angles for anti-clockwise rotations).

Representing $A$ relative to $B$,

\begin{equation}
\alpha_{1B} = (0, 1)
\end{equation}
\begin{equation}
\alpha_{2B} = (-1, 0)
\end{equation}

This is because $0\beta_1 + 1\beta_2 = [1, 0] = \alpha_1$ and $-1\beta_1 + 0\beta_2 = [0, 1] = \alpha_2$. We can construct the matrix P as:

\begin{equation}
P = \begin{bmatrix}
0 & -1 \\\\
1 & 0
\end{bmatrix}
\end{equation}

\begin{equation}
p_B = \begin{bmatrix}
0 & -1\\\\
1 & 0
\end{bmatrix}\begin{bmatrix}
5\\\\
0
\end{bmatrix} = \begin{bmatrix}
0\\\\
5
\end{bmatrix}
\end{equation}

We can verify that this $p_B$ is correct by taking the linear combination of the new basis vectors: $0\beta_1 + 5\beta_2 = [5, 0] = p$. Geometrically, $p_B$ lies in the $y$ direction relative to the new coordinates, which is the $x$ direction with respect to the standard basis.

Rotating the basis by $-90^\circ$ in this case is equivalent to rotating the vector $p$ by $90^\circ$. We can construct the rotation matrix:

\begin{equation}
Rot(90^\circ) = \begin{bmatrix}
\cos{90^\circ} & -\sin{90^\circ}\\\\
\sin{90^\circ} & \cos{90^\circ}
\end{bmatrix} = \begin{bmatrix}
0 & -1\\\\
1 & 0
\end{bmatrix}
\end{equation}

As we can see, the rotation matrix and $P$ are one and the same. In this case, the change of basis can be interpreted as the vector being rotated.


