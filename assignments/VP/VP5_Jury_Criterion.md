# VP5 — Juryjev stabilnostni kriterij (Z-domena)

**Datum:** 22. 4. 2026  
**Tema:** Diskretni ekvivalent Routh-Hurwitzovega kriterija za Z-domeno

---

## Teorija — Primerjava kriterijev

| | **Zvezni sistemi** | **Diskretni sistemi** |
|---|---|---|
| Domena | Laplaceova ($s$) | Z-transformacija ($z$) |
| Meja stabilnosti | Imaginarna os | Enotska krožnica $\|z\|=1$ |
| Stabilnostni kriterij | **Routh-Hurwitz** | **Juryjev kriterij** |
| Pogoj | $\text{Re}(s_i) < 0$ | $\|z_i\| < 1$ |

---

## Teorija — Juryjev kriterij

### Karakteristični polinom

$$P(z) = a_0 z^n + a_1 z^{n-1} + \cdots + a_{n-1} z + a_n, \qquad a_0 > 0$$

### Korak 1 — Nujni pogoji (pred gradnjo tabele)

1. $P(1) > 0$
2. $(-1)^n P(-1) > 0$
3. $|a_n| < a_0$

Če kateri od teh pogojev ni izpolnjen → sistem je **nestabilen** (ni treba graditi tabele).

### Korak 2 — Juryjeva tabela

Tabela se gradi iz koeficientov polinoma in njihovih zrcalnih vrednosti:

| Vrstica | $z^0$   | $z^1$   | $z^2$   | $\cdots$ | $z^n$   |
|---------|---------|---------|---------|----------|---------|
| 1       | $a_n$   | $a_{n-1}$ | $a_{n-2}$ | $\cdots$ | $a_0$ |
| 2       | $a_0$   | $a_1$   | $a_2$   | $\cdots$ | $a_n$ |
| 3       | $b_{n-1}$ | $b_{n-2}$ | $\cdots$ | $b_0$ | |
| 4       | $b_0$   | $b_1$   | $\cdots$ | $b_{n-1}$ | |
| 5       | $c_{n-2}$ | $c_{n-3}$ | $\cdots$ | | |
| $\vdots$ | $\vdots$ | | | | |

Elementi vrstice 3 ($b_k$):

$$b_k = \begin{vmatrix} a_n & a_{n-1-k} \\ a_0 & a_{k+1} \end{vmatrix} = a_n \cdot a_{k+1} - a_0 \cdot a_{n-1-k}, \qquad k = 0, 1, \ldots, n-1$$

Elementi vrstice 5 ($c_k$):

$$c_k = \begin{vmatrix} b_{n-1} & b_{n-2-k} \\ b_0 & b_{k+1} \end{vmatrix} = b_{n-1} \cdot b_{k+1} - b_0 \cdot b_{n-2-k}$$

### Korak 3 — Stabilnostni pogoj

Sistem je stabilen, če so **vsi elementi prvega stolpca pozitivni**:

$$|a_n| < a_0, \quad |b_{n-1}| > |b_0|, \quad |c_{n-2}| > |c_0|, \quad \ldots$$

Oziroma splošno: vsak lihi element $r_{1,1}$ v prvem stolpcu mora imeti **pozitivno absolutno vrednost**, pri čemer mora biti strogo večji od zadnjega elementa iste vrstice.

---

## Rešen primer 1 — Stabilen sistem

Karakteristični polinom:

$$P(z) = z^3 - 0{,}5z^2 + 0{,}3z - 0{,}1$$

Koeficienti: $a_0 = 1,\ a_1 = -0{,}5,\ a_2 = 0{,}3,\ a_3 = -0{,}1$

### Korak 1 — Nujni pogoji

$$P(1) = 1 - 0{,}5 + 0{,}3 - 0{,}1 = 0{,}7 > 0 \checkmark$$

$$(-1)^3 P(-1) = -(-1 - 0{,}5 - 0{,}3 - 0{,}1) = -(-1{,}9) = 1{,}9 > 0 \checkmark$$

$$|a_3| = 0{,}1 < a_0 = 1 \checkmark$$

### Korak 2 — Juryjeva tabela

| Vrstica | $z^0$     | $z^1$    | $z^2$    | $z^3$ |
|---------|-----------|----------|----------|-------|
| 1       | $-0{,}1$  | $0{,}3$  | $-0{,}5$ | $1$   |
| 2       | $1$       | $-0{,}5$ | $0{,}3$  | $-0{,}1$ |
| 3       | $b_2$     | $b_1$    | $b_0$    |       |
| 4       | $b_0$     | $b_1$    | $b_2$    |       |
| 5       | $c_1$     | $c_0$    |          |       |

Izračun $b_k$:

$$b_0 = \begin{vmatrix} a_3 & a_2 \\ a_0 & a_1 \end{vmatrix} = a_3 \cdot a_1 - a_0 \cdot a_2 = (-0{,}1)(-0{,}5) - (1)(0{,}3) = 0{,}05 - 0{,}3 = -0{,}25$$

$$b_1 = \begin{vmatrix} a_3 & a_1 \\ a_0 & a_2 \end{vmatrix} = a_3 \cdot a_2 - a_0 \cdot a_1 = (-0{,}1)(0{,}3) - (1)(-0{,}5) = -0{,}03 + 0{,}5 = 0{,}47$$

$$b_2 = \begin{vmatrix} a_3 & a_0 \\ a_0 & a_3 \end{vmatrix} = a_3^2 - a_0^2 = 0{,}01 - 1 = -0{,}99$$

Izračun $c_k$:

$$c_0 = \begin{vmatrix} b_2 & b_1 \\ b_0 & b_1 \end{vmatrix} = b_2 \cdot b_1 - b_0 \cdot b_1 = (-0{,}99)(0{,}47) - (-0{,}25)(0{,}47) = -0{,}4653 + 0{,}1175 = -0{,}348$$

$$c_1 = \begin{vmatrix} b_2 & b_0 \\ b_0 & b_2 \end{vmatrix} = b_2^2 - b_0^2 = (-0{,}99)^2 - (-0{,}25)^2 = 0{,}9801 - 0{,}0625 = 0{,}918$$

### Korak 3 — Stabilnostni pogoj

| Pogoj | Vrednost | Izpolnjen? |
|-------|----------|------------|
| $\|a_3\| < a_0$ | $0{,}1 < 1$ | ✓ |
| $\|b_2\| > \|b_0\|$ | $0{,}99 > 0{,}25$ | ✓ |
| $\|c_1\| > \|c_0\|$ | $0{,}918 > 0{,}348$ | ✓ |

$$\boxed{\text{Sistem je STABILEN}}$$

---

## Rešen primer 2 — Nestabilen sistem

Karakteristični polinom:

$$P(z) = z^2 - 0{,}5z + 1{,}2$$

Koeficienti: $a_0 = 1,\ a_1 = -0{,}5,\ a_2 = 1{,}2$

### Korak 1 — Nujni pogoji

$$P(1) = 1 - 0{,}5 + 1{,}2 = 1{,}7 > 0 \checkmark$$

$$(-1)^2 P(-1) = 1 + 0{,}5 + 1{,}2 = 2{,}7 > 0 \checkmark$$

$$|a_2| = 1{,}2 < a_0 = 1 \quad \text{NE velja} \; \times$$

Ker tretji nujni pogoj ni izpolnjen, je sistem po Juryjevem kriteriju **nestabilen** (tabele ni treba graditi).

### Korak 2 — Zaključek

Polinom ima produkt polov enak $z_1 z_2 = a_2/a_0 = 1{,}2 > 1$, zato vsaj en pol leži izven enotske krožnice.

### Korak 3 — Stabilnostni pogoj

| Pogoj | Vrednost | Izpolnjen? |
|-------|----------|------------|
| $\|a_2\| < a_0$ | $1{,}2 < 1$ | ✗ |

$$\boxed{\text{Sistem je NESTABILEN}}$$

> **Preverimo z MATLAB:** poli tega polinoma so približno $z_{1,2} = 0{,}25 \pm j\,1{,}067$, zato je $|z| = \sqrt{1{,}2} \approx 1{,}095 > 1$ — izven enotske krožnice ✗

---

## MATLAB — preveritev

```matlab
% Primer 1
p1 = [1, -0.5, 0.3, -0.1];
disp('Poli primera 1:'); roots(p1)
disp('|poli| < 1?'); abs(roots(p1)) < 1

% Primer 2
p2 = [1, -0.5, 1.2];
disp('Poli primera 2:'); roots(p2)
disp('|poli| < 1?'); abs(roots(p2)) < 1
```

---

*VP5 — Diskretni krmilni sistemi*
