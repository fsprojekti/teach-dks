# VP1 — Stabilnostni kriterij Routh-Hurwitz (Laplaceova domena)

**Datum:** 22. 4. 2026  
**Tema:** Analiza stabilnosti z Routh-Hurwitzovim kriterijem  

---

## Teorija — Splošni postopek Routh-Hurwitz

Za polinom $n$-tega reda:

$$P(s) = a_n s^n + a_{n-1} s^{n-1} + a_{n-2} s^{n-2} + \cdots + a_1 s + a_0, \qquad a_n > 0$$

### Korak 1 — Nujen pogoj

Vsi koeficienti $a_n, a_{n-1}, \ldots, a_1, a_0$ morajo biti **različni od nič in istega predznaka**. Če kateri manjka ali je negativen → sistem je nestabilen (ni treba graditi tabele).

### Korak 2 — Začetni dve vrstici tabele

Koeficiente razporedimo v dve vrstici: sodi indeksi v prvo, lihi indeksi v drugo:

| Vrstica   | Stolpec 1   | Stolpec 2   | Stolpec 3   | Stolpec 4   | $\cdots$ |
|-----------|-------------|-------------|-------------|-------------|----------|
| $s^n$     | $a_n$       | $a_{n-2}$   | $a_{n-4}$   | $a_{n-6}$   | $\cdots$ |
| $s^{n-1}$ | $a_{n-1}$   | $a_{n-3}$   | $a_{n-5}$   | $a_{n-7}$   | $\cdots$ |

### Korak 3 — Izračun preostalih vrstic

Vsako naslednjo vrstico izračunamo iz dveh prejšnjih po formuli "navzkrižnega množenja":

| Vrstica    | Stolpec 1    | Stolpec 2    | Stolpec 3    | $\cdots$ |
|------------|--------------|--------------|--------------|----------|
| $s^n$      | $a_n$     | $a_{n-2}$  | $a_{n-4}$  | $\cdots$ |
| $s^{n-1}$  | $a_{n-1}$ | $a_{n-3}$  | $a_{n-5}$  | $\cdots$ |
| $s^{n-2}$  | $b_1$     | $b_3$      | $b_5$      | $\cdots$ |
| $s^{n-3}$  | $c_1$     | $c_3$      | $c_5$      | $\cdots$ |
| $\vdots$   | $\vdots$  | $\vdots$   |            |          |
| $s^1$      | $q_1$     |            |            |          |
| $s^0$      | $d_1$     |            |            |          |

> **Opomba o indeksih:** Lihi indeksi ($b_1, b_3, b_5, \ldots$) se ujemajo z indeksi koeficientov lihih potenč ($a_{n-1}, a_{n-3}, \ldots$) iz 2. vrstice, ki vstopajo v formulo.

Formula za vrstico $b$ ($s^{n-2}$):

$$b_1 = \frac{a_{n-1} \cdot a_{n-2} - a_n \cdot a_{n-3}}{a_{n-1}}, \qquad b_3 = \frac{a_{n-1} \cdot a_{n-4} - a_n \cdot a_{n-5}}{a_{n-1}}, \qquad b_5 = \frac{a_{n-1} \cdot a_{n-6} - a_n \cdot a_{n-7}}{a_{n-1}}$$

Formula za vrstico $c$ ($s^{n-3}$):

$$c_1 = \frac{b_1 \cdot a_{n-3} - a_{n-1} \cdot b_3}{b_1}, \qquad c_3 = \frac{b_1 \cdot a_{n-5} - a_{n-1} \cdot b_5}{b_1}$$

### Korak 4 — Analiza prvega stolpca

Preštej spremembe predznaka v prvem stolpcu $r_{1,1},\ r_{2,1},\ r_{3,1},\ \ldots,\ r_{n+1,1}$:

| Rezultat | Zaključek |
|----------|-----------|
| 0 sprememb predznaka | Sistem je **stabilen** |
| $k > 0$ sprememb | Sistem ima $k$ polov v desni polravnini → **nestabilen** |

> **Opomba — posebni primeri:**
> - Če je element v prvem stolpcu **enak nič** (a ostale vrstice niso ničelne), ga nadomestimo z majhnim $\varepsilon > 0$ in nadaljujemo.
> - Če je **cela vrstica ničelna**, imamo na imaginarni osi konjugiran par polov — uporabimo pomožni polinom iz predhodne vrstice.

---

## Rešen primer — Ali je sistem stabilen?

Podan je diferencialni enačaj sistema:

$$y'''' + 2y''' + 6y'' + 4y' + y = x(t)$$

---

## 1. Korak — Karakteristični polinom

Prevedemo diferencialni enačaj v Laplaceovo domeno (ob ničelnih začetnih pogojih):

$$s^4 Y(s) + 2s^3 Y(s) + 6s^2 Y(s) + 4s Y(s) + Y(s) = X(s)$$

Prenosna funkcija:

$$G(s) = \frac{Y(s)}{X(s)} = \frac{1}{s^4 + 2s^3 + 6s^2 + 4s + 1}$$

Karakteristični polinom imenovalca:

$$P(s) = s^4 + 2s^3 + 6s^2 + 4s + 1$$

Koeficienti: $a_4=1,\ a_3=2,\ a_2=6,\ a_1=4,\ a_0=1$

> **Nujen (ne zadosten) pogoj:** vsi koeficienti morajo biti pozitivni. ✓

---

## 2. Korak — Routhova tabela

Routhova tabela za polinom 4. reda:

| Vrstica | Stolpec 1       | Stolpec 2       | Stolpec 3       |
|---------|-----------------|-----------------|-----------------|
| $s^4$   | $a_4 = 1$       | $a_2 = 6$       | $a_0 = 1$       |
| $s^3$   | $a_3 = 2$       | $a_1 = 4$       | $0$             |
| $s^2$   | $b_1 = ?$       | $b_3 = ?$       | $0$             |
| $s^1$   | $c_1 = ?$       | $c_3 = ?$       |                 |
| $s^0$   | $d_1 = ?$       |                 |                 |

**Izračun elementov $s^2$ vrstice ($b_1$ in $b_3$):**

$$b_1 = -\frac{1}{a_3}\begin{vmatrix} a_4 & a_2 \\ a_3 & a_1 \end{vmatrix} = \frac{a_3 \cdot a_2 - a_4 \cdot a_1}{a_3} = \frac{2 \cdot 6 - 1 \cdot 4}{2} = \frac{12 - 4}{2} = \frac{8}{2} = 4$$

$$b_3 = -\frac{1}{a_3}\begin{vmatrix} a_4 & a_0 \\ a_3 & 0 \end{vmatrix} = \frac{a_3 \cdot a_0 - a_4 \cdot 0}{a_3} = \frac{2 \cdot 1 - 1 \cdot 0}{2} = \frac{2}{2} = 1$$

**Izračun elementov $s^1$ vrstice ($c_1$ in $c_3$):**

$$c_1 = -\frac{1}{b_1}\begin{vmatrix} a_3 & a_1 \\ b_1 & b_3 \end{vmatrix} = \frac{b_1 \cdot a_1 - a_3 \cdot b_3}{b_1} = \frac{4 \cdot 4 - 2 \cdot 1}{4} = \frac{16 - 2}{4} = \frac{14}{4} = 3{,}5$$

$$c_3 = -\frac{1}{b_1}\begin{vmatrix} a_3 & 0 \\ b_1 & 0 \end{vmatrix} = \frac{b_1 \cdot 0 - a_3 \cdot 0}{b_1} = 0$$

**Izračun elementa $s^0$ vrstice:**

$$d_1 = -\frac{1}{c_1}\begin{vmatrix} b_1 & b_3 \\ c_1 & 0 \end{vmatrix} = \frac{c_1 \cdot b_3 - b_1 \cdot 0}{c_1} = \frac{3{,}5 \cdot 1}{3{,}5} = 1$$

**Popolna Routhova tabela:**

| Vrstica | Stolpec 1 | Stolpec 2 | Stolpec 3 |
|---------|-----------|-----------|-----------|
| $s^4$   | $a_4=1$   | $a_2=6$   | $a_0=1$  |
| $s^3$   | $a_3=2$   | $a_1=4$   | $0$      |
| $s^2$   | $b_1=4$   | $b_3=1$   | $0$      |
| $s^1$   | $c_1=3{,}5$ | $c_3=0$   |          |
| $s^0$   | $d_1=1$   |           |          |

---

## 3. Korak — Analiza prvega stolpca

Elementi prvega stolpca: $1,\ 2,\ 4,\ 3{,}5,\ 1$

$$\text{Spremembe predznaka} = 0$$

> **Routh-Hurwitzov kriterij:** Število korenov karakterističnega polinoma v desni polravnini kompleksne ravnine je enako številu sprememb predznaka v prvem stolpcu.

---

## 4. Zaključek

$$\boxed{\text{Sistem je STABILEN}}$$

Vsi elementi prvega stolpca so **pozitivni** → ni sprememb predznaka → **vsi poli so v levi polravnini** ($\text{Re}(s) < 0$).

### MATLAB — preveritev

```matlab
% Primer 1: s^4 + 2s^3 + 6s^2 + 4s + 1
num = [1];
den = [1, 2, 6, 4, 1];

G = tf(num, den);

% Izpis polov (vsi morajo imeti Re(s) < 0)
p = pole(G)

% Odziv na stopnico
figure;
step(G);
title('Odziv na stopnico — Primer 1 (stabilen)');
grid on;
```

**Pričakovani izhod:** vsi poli imajo negativno realno komponento, odziv konvergira.

---

## Rešen primer 2 — Ali je sistem stabilen?

Podan je diferencialni enačaj sistema:

$$y''' + 2y'' + y' + 5y = x(t)$$

### 1. Korak — Karakteristični polinom

$$s^3 Y(s) + 2s^2 Y(s) + s Y(s) + 5 Y(s) = X(s)$$

Prenosna funkcija:

$$G(s) = \frac{1}{s^3 + 2s^2 + s + 5}$$

Koeficienti: $a_3 = 1,\ a_2 = 2,\ a_1 = 1,\ a_0 = 5$

> Nujen pogoj: vsi koeficienti so pozitivni ✓ — a to še ne zadošča.

### 2. Korak — Routhova tabela

Začetni dve vrstici:

| Vrstica | Stolpec 1       | Stolpec 2       |
|---------|-----------------|-----------------|
| $s^3$   | $a_3 = 1$       | $a_1 = 1$       |
| $s^2$   | $a_2 = 2$       | $a_0 = 5$       |
| $s^1$   | $b_1 = ?$       | $b_3 = ?$       |
| $s^0$   | $c_1 = ?$       |                 |

**Izračun elementov $s^1$ vrstice ($b_1$ in $b_3$):**

$$b_1 = -\frac{1}{a_2}\begin{vmatrix} a_3 & a_1 \\ a_2 & a_0 \end{vmatrix} = \frac{a_2 \cdot a_1 - a_3 \cdot a_0}{a_2} = \frac{2 \cdot 1 - 1 \cdot 5}{2} = \frac{2 - 5}{2} = \frac{-3}{2} = -1{,}5$$

$$b_3 = -\frac{1}{a_2}\begin{vmatrix} a_3 & 0 \\ a_2 & 0 \end{vmatrix} = \frac{a_2 \cdot 0 - a_3 \cdot 0}{a_2} = 0$$

**Izračun elementa $s^0$ vrstice:**

$$c_1 = -\frac{1}{b_1}\begin{vmatrix} a_2 & a_0 \\ b_1 & b_3 \end{vmatrix} = \frac{b_1 \cdot a_0 - a_2 \cdot b_3}{b_1} = \frac{-1{,}5 \cdot 5 - 2 \cdot 0}{-1{,}5} = \frac{-7{,}5}{-1{,}5} = 5$$

**Popolna Routhova tabela:**

| Vrstica | Stolpec 1        | Stolpec 2  |
|---------|------------------|------------|
| $s^3$   | $a_3 = 1$        | $a_1 = 1$  |
| $s^2$   | $a_2 = 2$        | $a_0 = 5$  |
| $s^1$   | $b_1 = -1{,}5$   | $b_3 = 0$  |
| $s^0$   | $c_1 = 5$        |            |

### 3. Korak — Analiza prvega stolpca

Elementi prvega stolpca: $1,\ 2,\ -1{,}5,\ 5$

| Prehod | Sprememba predznaka? |
|--------|----------------------|
| $1 \to 2$ | ne ($+\to+$) |
| $2 \to -1{,}5$ | **DA** ($+\to-$) |
| $-1{,}5 \to 5$ | **DA** ($-\to+$) |

$$\text{Spremembe predznaka} = 2$$

### 4. Zaključek

$$\boxed{\text{Sistem je NESTABILEN}}$$

Sta **2 spremembi predznaka** → sistem ima **2 pola v desni polravnini** ($\text{Re}(s) > 0$).

### MATLAB — preveritev

```matlab
% Primer 2: s^3 + 2s^2 + s + 5
num = [1];
den = [1, 2, 1, 5];

G = tf(num, den);

% Izpis polov (pričakujemo 2 pola z Re(s) > 0)
p = pole(G)

% Odziv na stopnico — divergira (nestabilen sistem)
figure;
t = linspace(0, 5, 1000);
lsim(G, ones(size(t)), t);
title('Odziv na stopnico — Primer 2 (nestabilen)');
ylabel('y(t)'); xlabel('t [s]');
grid on;
```

> **Opomba:** Za nestabilen sistem `step()` pogosto ne prikaže koristnega grafa zaradi divergence. Namesto tega uporabimo `lsim()` z omejenim časovnim oknom.

---

*VP1 — Diskretni krmilni sistemi*
