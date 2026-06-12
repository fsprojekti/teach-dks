# Izpit — Diskretni krmilni sistemi

## 1. naloga — Diskretna konvolucija (25 točk)

Diskretni sistem: $y[k] - 0{,}6\,y[k-1] = x[k]$, $y[-1] = 0$.

1. **(15 t.)** Z diskretno konvolucijsko vsoto $y[k] = \sum_{m=0}^{k} h[m]\,x_1[k-m]$ izračunajte odziv na rampni vhod $x_1[k] = k$ (za $k\geq 0$, sicer $0$) za $k = 0,\dots,4$.
2. **(10 t.)** Izračunajte $y[\infty]$ pri stopničnem vhodu $u[k] = 1$, $k \geq 0$.

---

## 2. naloga — Inverzna Z transformacija (25 točk)

Dano: $X(z) = \dfrac{z}{(z-1)(z-0{,}8)}$, $k \geq 0$.

1. **(15 t.)** Z metodo delnih ulomkov poiščite zaprtokodno obliko $x[k]$. *(Namig: razvijte $\frac{X(z)}{z}$, nato pomnožite z $z$.)*
2. **(10 t.)** Izračunajte $x[k]$ za $k = 0,\dots,4$ in jih vpišite v tabelo.

---

## 3. naloga — Routh-Hurwitzov kriterij (25 točk)

Zaprtozančni sistem ($K > 0$): $G(s) = \dfrac{K}{s(s+1)(s+3)}$, $P(s) = s^3 + 4s^2 + 3s + K$.

1. **(15 t.)** Sestavite popolno Routhovo tabelo za $P(s)$.
2. **(10 t.)** Iz prvega stolpca določite območje $K$ za stabilnost.

---

## 4. naloga — Realizacija prenosne funkcije (25 točk)

Dano: $H(z) = \dfrac{Y(z)}{U(z)} = \dfrac{z + 0{,}2}{z^2 - 0{,}7z + 0{,}1}$.

1. **(8 t.)** Z **indirektno metodo** uvedite $W(z) = \dfrac{U(z)}{D(z)}$, $D(z) = z^2 - 0{,}7z + 0{,}1$, in zapišite diferenčni enačbi za $w[k]$ in $y[k]$.
2. **(7 t.)** Narišite blokovno shemo z dvema zakasnilnima členoma $z^{-1}$.
3. **(5 t.)** Izračunajte $w[k]$ in $y[k]$ za $k = 0,\dots,4$ pri $u[k]=1$ ($k\geq 0$), $w[-1]=w[-2]=0$.
4. **(5 t.)** Poiščite pole $H(z)$, ugotovite stabilnost in izračunajte $y[\infty]$.
