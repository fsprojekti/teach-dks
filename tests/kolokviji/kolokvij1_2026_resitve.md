# Kolokvij I — Rešitve (INTERNO — za pedagoško osebje)

---

## 1. naloga — Diskretna konvolucija

### 1.1 Konvolucijska tabela — rampni vhod (15 točk)

Diskretna konvolucijska vsota deluje nad **signali**, ne nad diferenčno enačbo. Zato moramo najprej iz diferenčne enačbe izračunati impulzni odziv $h[k]$ — to je odziv sistema na vhod $x[k] = \delta[k]$. Šele ta signal nato konvoliramo z vhodnim signalom $x_1[k]$.

**Korak 1 — izračun $h[k]$ iz diferenčne enačbe** (vhod $x[k]=\delta[k]$, $h[-1]=0$):

| $k$ | Izračun $h[k] = 0{,}8 \cdot h[k-1] + \delta[k]$ | $h[k]$ |
|-----|------|--------|
| 0 | $0{,}8 \cdot 0 + 1$ | **1** |
| 1 | $0{,}8 \cdot 1 + 0$ | **0,8** |
| 2 | $0{,}8 \cdot 0{,}8 + 0$ | **0,64** |
| 3 | $0{,}8 \cdot 0{,}64 + 0$ | **0,512** |
| 4 | $0{,}8 \cdot 0{,}512 + 0$ | **0,4096** |

**Korak 2 — konvolucijska vsota** $y[k] = \sum_{m=0}^{k} h[m] \cdot x_1[k-m]$

Prispevek $h[m] \cdot x_1[k-m]$ je 0, kadar $m > k$ (sistem je kavzalen).

| $k$ | $x_1[k]$ | $h[0]{\cdot}x_1[k]$ | $h[1]{\cdot}x_1[k{-}1]$ | $h[2]{\cdot}x_1[k{-}2]$ | $h[3]{\cdot}x_1[k{-}3]$ | $h[4]{\cdot}x_1[k{-}4]$ | $y[k]$ |
|-----|--------|---------|---------|---------|---------|---------|--------|
| 0 | 0 | $1{\cdot}0=0$ | 0 | 0 | 0 | 0 | **0** |
| 1 | 1 | $1{\cdot}1=1$ | $0{,}8{\cdot}0=0$ | 0 | 0 | 0 | **1** |
| 2 | 2 | $1{\cdot}2=2$ | $0{,}8{\cdot}1=0{,}8$ | $0{,}64{\cdot}0=0$ | 0 | 0 | **2,8** |
| 3 | 3 | $1{\cdot}3=3$ | $0{,}8{\cdot}2=1{,}6$ | $0{,}64{\cdot}1=0{,}64$ | $0{,}512{\cdot}0=0$ | 0 | **5,24** |
| 4 | 4 | $1{\cdot}4=4$ | $0{,}8{\cdot}3=2{,}4$ | $0{,}64{\cdot}2=1{,}28$ | $0{,}512{\cdot}1=0{,}512$ | $0{,}4096{\cdot}0=0$ | **8,192** |

---

### 1.2 Ustaljeno stanje pri stopničnem vhodu (10 točk)

V ustaljenem stanju velja $y[\infty] = y[\infty - 1]$. Vstavimo v diferenčno enačbo:

$$y[\infty] - 0{,}8 \cdot y[\infty] = u[\infty] = 1$$

$$y[\infty](1 - 0{,}8) = 1 \quad\Rightarrow\quad y[\infty] = \frac{1}{0{,}2} = \boxed{5}$$

Fizikalna razlaga: sistem ojača stopnico s faktorjem $\dfrac{1}{1-0{,}8} = 5$ (statično ojačanje).

---

## 2. naloga — Modeliranje z diferenčno enačbo

### 2.1 Diferenčna enačba (10 točk)

$k$ predstavlja leto (diskretni čas). Populacija se poveča za $50\,\%$, zmanjša za ulov in za gostotno umrljivost:

$$\boxed{y[k] = 1{,}5 \cdot y[k-1] - 40 - 0{,}001 \cdot y[k-1]^2, \quad y[0] = 200}$$

### 2.2 Numerični izračun (10 točk)

| $k$ | Izračun | $y[k]$ |
|-----|---------|--------|
| 0 | (dano) | **200** |
| 1 | $1{,}5 \cdot 200 - 40 - 0{,}001 \cdot 200^2 = 300 - 40 - 40$ | **220** |
| 2 | $1{,}5 \cdot 220 - 40 - 0{,}001 \cdot 220^2 = 330 - 40 - 48{,}4$ | **241{,}6** |
| 3 | $1{,}5 \cdot 241{,}6 - 40 - 0{,}001 \cdot 241{,}6^2 = 362{,}4 - 40 - 58{,}37$ | **264{,}03** |
| 4 | $1{,}5 \cdot 264{,}03 - 40 - 0{,}001 \cdot 264{,}03^2 = 396{,}05 - 40 - 69{,}71$ | **286{,}33** |
| 5 | $1{,}5 \cdot 286{,}33 - 40 - 0{,}001 \cdot 286{,}33^2 = 429{,}50 - 40 - 81{,}98$ | **307{,}51** |

Populacija narašča in konvergira k ravnotežni vrednosti 400.

### 2.3 Ravnotežne točke (5 točk)

V ustaljenem stanju velja $y[\infty] = y[\infty - 1]$. Vstavimo:

$$y[\infty] = 1{,}5 \cdot y[\infty] - 40 - 0{,}001 \cdot y[\infty]^2$$

$$0{,}001 \cdot y[\infty]^2 - 0{,}5 \cdot y[\infty] + 40 = 0$$

Pomnožimo z 1000:

$$y[\infty]^2 - 500 \cdot y[\infty] + 40000 = 0$$

Splošna rešitev kvadratne enačbe $ay^2 + by + c = 0$:

$$y[\infty] = \frac{-b \pm \sqrt{b^2 - 4ac}}{2a} = \frac{500 \pm \sqrt{500^2 - 4 \cdot 1 \cdot 40000}}{2 \cdot 1}$$

$$= \frac{500 \pm \sqrt{250000 - 160000}}{2} = \frac{500 \pm \sqrt{90000}}{2} = \frac{500 \pm 300}{2}$$

$$\boxed{y_1 = \frac{500 - 300}{2} = 100 \quad (\text{nestabilna}), \qquad y_2 = \frac{500 + 300}{2} = 400 \quad (\text{stabilna})}$$

**Analiza stabilnosti:** Sprememba populacije med korakoma je $\Delta y = y[k] - y[k-1]$:

$$\Delta y = (1{,}5\,y - 40 - 0{,}001\,y^2) - y = 0{,}5\,y - 40 - 0{,}001\,y^2$$

$$= -0{,}001\,(y - 100)\,(y - 400)$$

Predznak $\Delta y$ določa smer gibanja populacije:

| Območje | $(y-100)$ | $(y-400)$ | $\Delta y$ | Smer |
|---------|-----------|-----------|------------|------|
| $y < 100$ | $-$ | $-$ | $-$ | pada → 0 (izumrtje) |
| $100 < y < 400$ | $+$ | $-$ | $+$ | raste → 400 |
| $y > 400$ | $+$ | $+$ | $-$ | pada → 400 |

Sistem je **bistabilen** — obstajata dve stabilni stanji:

- $y = 0$: absorpcijsko stanje (izumrtje) — dosegljivo iz $y < 100$
- $y = 400$: stabilno ravnovesje — dosegljivo iz $y > 100$

Točka $y = 100$ je **nestabilni separatriks** (meja preživetja) — ločuje območji privlaka $y=0$ in $y=400$. Pri $y[0] = 200 > 100$ sistem konvergira k $y = 400$.

---

## 3. naloga — Inverzna Z transformacija

### 3.1 Delni ulomki (15 točk)

Razvijemo $\frac{X(z)}{z}$ na delne ulomke:

$$\frac{X(z)}{z} = \frac{1}{(z-1)(z-0{,}5)} = \frac{A}{z-1} + \frac{B}{z-0{,}5}$$

$$A = \left.\frac{1}{z-0{,}5}\right|_{z=1} = \frac{1}{0{,}5} = 2$$

$$B = \left.\frac{1}{z-1}\right|_{z=0{,}5} = \frac{1}{-0{,}5} = -2$$

Pomnožimo z $z$:

$$X(z) = \frac{2z}{z-1} - \frac{2z}{z-0{,}5}$$

Iz Z transformacijske tabele ($\mathcal{Z}\{a^k u[k]\} = \frac{z}{z-a}$):

$$\boxed{x[k] = 2\,u[k] - 2\cdot(0{,}5)^k\,u[k] = 2\bigl(1 - (0{,}5)^k\bigr)\,u[k]}$$

### 3.2 Numerične vrednosti (10 točk)

| $k$ | Izračun $2(1-(0{,}5)^k)$ | $x[k]$ |
|-----|--------------------------|--------|
| 0 | $2(1-1)$ | **0** |
| 1 | $2(1-0{,}5)$ | **1** |
| 2 | $2(1-0{,}25)$ | **1,5** |
| 3 | $2(1-0{,}125)$ | **1,75** |
| 4 | $2(1-0{,}0625)$ | **1,875** |

Signal eksponentno konvergira k $x[\infty] = 2$.

---

## 4. naloga — Odziv sistema z Z transformacijo

### 4.1 Zapis $Y(z)$ (10 točk)

Z transformacijo diferenčne enačbe (ničelni začetni pogoji):

$$Y(z)\bigl(1 - 1{,}1z^{-1} + 0{,}3z^{-2}\bigr) = X(z) = \frac{z}{z-1}$$

Pomnožimo s $z^2$:

$$Y(z) \cdot \frac{z^2 - 1{,}1z + 0{,}3}{z^2} = \frac{z}{z-1}$$

Poli imenovalca: $z^2 - 1{,}1z + 0{,}3 = 0 \Rightarrow z = \frac{1{,}1 \pm \sqrt{1{,}21-1{,}2}}{2} = \frac{1{,}1 \pm 0{,}1}{2}$, torej $z_1 = 0{,}6$, $z_2 = 0{,}5$.

$$\boxed{Y(z) = \frac{z^3}{(z-1)(z-0{,}6)(z-0{,}5)}}$$

Oba pola sta znotraj enotske krožnice ($|z|<1$) → sistem je stabilen.

### 4.2 Delni ulomki (10 točk)

$$\frac{Y(z)}{z} = \frac{z^2}{(z-1)(z-0{,}6)(z-0{,}5)} = \frac{A}{z-1} + \frac{B}{z-0{,}6} + \frac{C}{z-0{,}5}$$

$$A = \left.\frac{z^2}{(z-0{,}6)(z-0{,}5)}\right|_{z=1} = \frac{1}{0{,}4 \cdot 0{,}5} = 5$$

$$B = \left.\frac{z^2}{(z-1)(z-0{,}5)}\right|_{z=0{,}6} = \frac{0{,}36}{(-0{,}4)(0{,}1)} = -9$$

$$C = \left.\frac{z^2}{(z-1)(z-0{,}6)}\right|_{z=0{,}5} = \frac{0{,}25}{(-0{,}5)(-0{,}1)} = 5$$

$$Y(z) = \frac{5z}{z-1} - \frac{9z}{z-0{,}6} + \frac{5z}{z-0{,}5}$$

$$\boxed{y[k] = \bigl(5 - 9\cdot(0{,}6)^k + 5\cdot(0{,}5)^k\bigr)\,u[k]}$$

Ustaljeno stanje: $y[\infty] = 5$ (prispeva le pol pri $z=1$).

Proveritev: $y[\infty](1-1{,}1+0{,}3) = y[\infty]\cdot 0{,}2 = 1 \Rightarrow y[\infty] = 5$ ✓

### 4.3 Preveritev z diferenčno enačbo (5 točk)

$$y[0] = 1{,}1\cdot 0 - 0{,}3\cdot 0 + u[0] = 1$$

Iz formule: $y[0] = 5 - 9 + 5 = \mathbf{1}$ ✓

$$y[1] = 1{,}1\cdot 1 - 0{,}3\cdot 0 + 1 = 2{,}1$$

Iz formule: $y[1] = 5 - 9\cdot 0{,}6 + 5\cdot 0{,}5 = 5 - 5{,}4 + 2{,}5 = \mathbf{2{,}1}$ ✓
