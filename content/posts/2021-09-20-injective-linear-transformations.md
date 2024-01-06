---
title: "Injective linear transformations"
date: 2021-09-20T01:02:55-07:00
draft: false
tags: ["linear_algebra", "math"]
---

In this post, we take a look at some characteristics of injective linear transformations. Let $T$ be a linear transformation from some vector space $V$ to another vector space $W$, both defined over a field $F$. $T$ being injective, means that no two vectors in $V$ can be mapped to the same vector in $W$. We have a one-to-one mapping between the vectors in $V$ and their images in $W$.


\begin{equation}
T(\alpha) = T(\beta) \implies \alpha = \beta
\end{equation}

What does the null space of $T$ look like?

\begin{equation}
N = \set{\alpha \in V: T\alpha = 0}
\end{equation}

Assuming that $V$ isn't just $\set{0}$, for an $\alpha \in V$ ($\alpha \neq 0$), we can pick another element $\beta \in V$ that is different from $\alpha$. We have $\gamma = \beta - \alpha$, with $\gamma \neq 0$, $\gamma \neq \beta$ and $\alpha = \beta - \gamma$, basically writing $\alpha$ as the difference of two other vectors in $V$.


\begin{equation}
T\alpha = T(\beta - \gamma) = T\beta - T\gamma
\end{equation}

\begin{equation}
T\alpha = 0 \implies T\beta - T\gamma = 0 \implies T\beta = T\gamma
\end{equation}

As $\beta$ and $\gamma$ are different vectors in $V$, they cannot have the same image in $W$ as $T$ is injective. So only the zero vector in $V$ can get mapped to the zero vector in $W$.

\begin{equation}
T\alpha = 0 \implies \alpha = 0
\end{equation}

The null space $N$ of $T$ is just the zero vector space $\set{0}$ over $F$. An injective linear transformation cannot map non-zero vectors to the zero vector. We sometimes say that such a $T$ is non-singular.

Another property of such a transformation, is that it preserves linear independence. Stating more formally, if $A \subset V$ is a set of linearly independent vectors in $V$ consisting of vectors $\set{\alpha_1, \ldots, \alpha_n}$, then $B = \set{T\alpha_1, \ldots, T\alpha_n} \subset W$ is also an independent set of vectors.

Let's look at a simple way of verifying this. We let $\beta_j = T\alpha_j$. Assume that $A$ is a set of independent vectors and $B$ is not. If the vectors in $B$ are not independent, it means that we can express some particular vector as a linear combination of the other vectors.

\begin{equation}
\beta_k = \sum_{j=1, j \neq k}^{n}c_j\beta_j
\end{equation}

As $T$ is a linear transform, we have

\begin{equation}
T\alpha_k = T(\sum_{j=1, j \neq k}^{n}c_j\alpha_j)
\end{equation}

\begin{equation}
T\alpha_k - T(\sum_{j=1, j \neq k}^{n}c_j\alpha_j) = 0
\end{equation}

\begin{equation}
T(\alpha_k - \sum_{j=1, j \neq k}^{n}c_j\alpha_j) = 0
\end{equation}

As the $\alpha_j$'s are independent, the term $\alpha_k - \sum_{j=1, j \neq k}^{n}c_j\alpha_j \neq 0$ as we cannot express any $\alpha_k$ as a linear combination of the other vectors in A.

But as we have seen, if $T$ is injective, it cannot map non-zero vectors to zero. Hence the vectors $\beta_j$ in $B$ must also be independent, meaning that an injective linear transformation preserves the linear independence of vectors.

To close this out, we tie this back to a fundamental result in linear algebra. Let $R_T$ be the range of $T$ and $\dim R_T$ be the rank of $T$. As $T$ is injective, the range has the same dimensionality as $V$ as the basis of $V$ would get mapped to independent vectors in $R_T$. So we have $\dim V = \dim R_T$. For a general linear transformation $T$, we know that:

\begin{equation}
\dim V = \dim N + \dim R_T
\end{equation}

If $T$ is injective, $\dim N = 0$ and so $\dim V = \dim R_T$, which agrees with our conclusion above.
