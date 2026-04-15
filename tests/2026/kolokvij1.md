# Kolokvij I — Diskretni krmilni sistemi

**Datum:** 3.4.2026 
**Trajanje:** 60 min  

> Dovoljeni pripomočki: zapiski, kalkulator. Elektronske naprave s spletno povezavo niso dovoljene.

> **Predpostavka:** Vsi sistemi so kavzalni — za negativne čase je vrednost vsakega signala in impulznega odziva enaka nič.

---

## 1. naloga — Diskretna konvolucija (25 točk)

Dan je diskretni sistem, opisan z diferenčno enačbo:

$$y[k] - 0{,}8 \cdot y[k-1] = x[k], \quad y[-1] = 0$$


1. **(15 točk)** Z uporabo diskretne konvolucijske vsote izračunajte odziv sistema na rampni vhod za vzorce $k = 0, 1, 2, 3, 4$. 

   Rampni vhod je definiran kot:

   $$x_1[k] = \begin{cases} k, & k \geq 0 \\ 0, & k < 0 \end{cases}$$

   Signal linearno narašča z naklonom 1 na vzorec.

   $$y[k] = \sum_{m=0}^{k} h[m] \cdot x_1[k - m]$$

2. **(10 točk)** Izračunajte ustaljeno vrednost izhoda $y[\infty]$, če je vhod stopnična funkcija $u[k] = 1$ za $k \geq 0$.

---

## 2. naloga — Modeliranje z diferenčno enačbo (25 točk)

V jezeru živi populacija rib. Vsako leto se populacija poveča za $50\,\%$ (razmnoževanje). Ribiči vsako leto ulovijo $40$ rib. Ker visoka gostota populacije povečuje tekmovalnost za hrano in razširjenost bolezni, vsako leto naravno pogine $0{,}001 \cdot y[k-1]^2$ rib.

Začetna populacija ob času $k = 0$ je $y[0] = 200$ rib.


1. **(10 točk)** Zapišite diferenčno enačbo, ki opisuje časovni razvoj populacije $y[k]$. Jasno definirajte, kaj predstavlja $k$.

2. **(10 točk)** Izračunajte populacijo za $k = 1, 2, 3, 4, 5$.


3. **(5 točk)** Poiščite vrednosti populacije $y[\infty]$, pri katerih se populacija ne bi več spreminjala. Kateri vrednosti sistem konvergira pri $y[0] = 200$?

---

## 3. naloga — Inverzna Z transformacija (25 točk)

Dana je Z transformacija signala $x[k]$:

$$X(z) = \frac{z}{(z-1)(z-0{,}5)}, \quad k \geq 0$$

1. **(15 točk)** Z metodo delnih ulomkov poiščite zaprtokodno obliko inverza Z transformacije $x[k]$.

   *(Namig: razvijte $\frac{X(z)}{z}$ na delne ulomke, nato pomnožite z $z$.)*

2. **(10 točk)** Izračunajte vrednosti $x[k]$ za $k = 0, 1, 2, 3, 4$ in jih vpišite v tabelo.


---

## 4. naloga — Odziv sistema z Z transformacijo (25 točk)

Dan je diskretni sistem drugega reda z ničelnimi začetnimi pogoji ($y[-1] = y[-2] = 0$) in stopničnim vhodom $x[k] = u[k]$:

$$y[k] - 1{,}1\,y[k-1] + 0{,}3\,y[k-2] = u[k]$$

1. **(10 točk)** Z Z transformacijo zapišite $Y(z)$ kot razlomljeno funkcijo z ločenimi poli.

2. **(10 točk)** Z metodo delnih ulomkov poiščite $y[k]$.

3. **(5 točk)** Preverite rezultat: izračunajte $y[0]$ in $y[1]$ neposredno iz diferenčne enačbe in primerjajte z vrednostmi iz točke 2.

---
