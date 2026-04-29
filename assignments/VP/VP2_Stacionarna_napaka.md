# VP2 — Pogrešek v ustaljenem stanju (Laplaceova domena)

**Datum:** 22. 4. 2026  
**Tema:** Analiza pogreška v ustaljenem stanju s pomočjo izreka o končni vrednosti

---

## Teorija — Pogrešek v ustaljenem stanju

### Definicija

Pogrešek sistema je razlika med referenčnim vhodom in izhodom:

$$E(s) = X(s) - Y(s) = X(s)\left[1 - G_{cl}(s)\right]$$

kjer je $G_{cl}(s) = Y(s)/X(s)$ zaprtozančna prenosna funkcija.

### Izrek o končni vrednosti (Final Value Theorem)

Pogrešek v ustaljenem stanju dobimo z limitiranjem:

$$e_{ss} = \lim_{t \to \infty} e(t) = \lim_{s \to 0} s \cdot E(s)$$

> **Pogoj:** Izrek velja samo, če so vsi poli $s \cdot E(s)$ v levi polravnini (sistem mora biti stabilen in pogrešek mora konvergirati).

### Standardni vhodni signali

| Vhod           | $X(s)$       | Izraz za $e_{ss}$                                |
|----------------|--------------|--------------------------------------------------|
| Stopnica       | $\dfrac{1}{s}$ | $\lim_{s \to 0}\left[1 - G_{cl}(s)\right]$     |
| Rampa          | $\dfrac{1}{s^2}$ | $\lim_{s \to 0} \dfrac{1 - G_{cl}(s)}{s}$   |
| Parabolični    | $\dfrac{1}{s^3}$ | $\lim_{s \to 0} \dfrac{1 - G_{cl}(s)}{s^2}$ |

---

## Blokovni diagram

Sistem ima naslednjo strukturo (iz sheme):

```
        +--------+
X --+-->| × 2    |
    |   +---+----+      +------+
    |       |           |      |
    |      [Σ]------>[ G1 ]---+---> Y
    |       ↑-          |      |
    |       |           |      |
    |   [ G2 ]<---------+      |
    |       ↑- (×2)            |
    |                          |
    +---------- (×3) ----------+
                    ↑-
```

Enačbe vozlišč:

$$E = 2\,X - 2\,G_2\,Y - 3\,Y$$

$$Y = G_1 \cdot E$$

### Izpeljava zaprtozančne prenosne funkcije

$$Y = G_1\left(2X - 2G_2 Y - 3Y\right)$$

$$Y = 2G_1 X - 2G_1 G_2 Y - 3G_1 Y$$

$$Y\left(1 + 2G_1 G_2 + 3G_1\right) = 2G_1 X$$

$$\boxed{G_{cl}(s) = \frac{Y(s)}{X(s)} = \frac{2\,G_1}{1 + 2\,G_1\,G_2 + 3\,G_1}}$$

---

## Rešen primer

### Podatki

$$G_1(s) = \frac{5}{s}, \qquad G_2(s) = 1$$

### Korak 1 — Zaprtozančna prenosna funkcija

$$G_{cl}(s) = \frac{2 \cdot \dfrac{5}{s}}{1 + 2 \cdot \dfrac{5}{s} \cdot 1 + 3 \cdot \dfrac{5}{s}} = \frac{\dfrac{10}{s}}{\dfrac{s + 10 + 15}{s}} = \frac{10}{s + 25}$$

### Korak 2 — Stabilnost

Pol zaprtozančnega sistema: $s = -25$ → levi polravnini → **sistem je stabilen** ✓

### Korak 3 — Pogrešek v ustaljenem stanju (stopničast vhod)

Za stopnico $X(s) = \dfrac{1}{s}$:

$$E(s) = X(s)\left[1 - G_{cl}(s)\right] = \frac{1}{s}\left[1 - \frac{10}{s+25}\right] = \frac{1}{s} \cdot \frac{s + 25 - 10}{s + 25} = \frac{s + 15}{s(s + 25)}$$

Izrek o končni vrednosti:

$$e_{ss} = \lim_{s \to 0}\, s \cdot \frac{s + 15}{s(s + 25)} = \lim_{s \to 0} \frac{s + 15}{s + 25} = \frac{15}{25} = \boxed{0{,}6}$$

> **Interpretacija:** Kljub integratorju v $G_1(s) = 5/s$ pogrešek v ustaljenem stanju **ni nič**. Razlog je, da to ni standardna enotna povratna zanka — v blokovnem diagramu sta prisotni dve povratni veji z ojačanjema 2 in 3, kar premakne statično ojačanje $G_{cl}(0) = 10/25 = 0{,}4 \neq 1$.

### Korak 4 — Pogrešek v ustaljenem stanju (rampast vhod)

Za rampo $X(s) = \dfrac{1}{s^2}$:

$$E(s) = \frac{1}{s^2} \cdot \frac{s + 15}{s + 25}$$

$$e_{ss} = \lim_{s \to 0}\, s \cdot \frac{s + 15}{s^2(s + 25)} = \lim_{s \to 0} \frac{s + 15}{s(s + 25)} = \frac{15}{0} \to \infty$$

> **Interpretacija:** Za rampast vhod pogrešek **divergira** — sistem ne more slediti naraščajočemu vhodu.

---

## MATLAB — preveritev

```matlab
% Parametri
G1 = tf([5], [1, 0]);   % G1(s) = 5/s
G2 = tf([1], [1]);      % G2(s) = 1

% Zaprtozančna prenosna funkcija (ročno izpeljana)
Gcl = tf([10], [1, 25]);

% Preveritev polov
disp('Poli Gcl:'); pole(Gcl)

% Odziv na stopnico
figure(1);
step(Gcl);
title('Odziv na stopnico — G_{cl}(s) = 10/(s+25)');
grid on;
yline(1, 'r--', 'Referenca');
yline(1 - 0.6, 'k:', 'e_{ss} = 0.6');

% Odziv na rampo z lsim
figure(2);
t = 0:0.01:5;
u_ramp = t;   % rampast vhod
lsim(Gcl, u_ramp, t);
hold on;
plot(t, u_ramp, 'r--', 'DisplayName', 'Referenca (rampa)');
title('Odziv na rampo — pogrešek divergira');
legend; grid on;
```

---

*VP2 — Diskretni krmilni sistemi*
