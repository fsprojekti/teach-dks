# VP3 — Metoda diagrama lege korenov (Root Locus)

**Datum:** 22. 4. 2026  
**Tema:** Izris diagrama lege korenov in analiza stabilnosti v odvisnosti od ojačanja K

---

## Teorija — Diagram lege korenov (Root Locus)

### Definicija

Diagram lege korenov prikazuje, kako se **poli zaprtozančnega sistema** premikajo po kompleksni ravnini, ko ojačanje $K$ narašča od $0$ do $\infty$.

Standardna oblika zaprtozančne prenosne funkcije z ojačanjem K:

$$G_{cl}(s) = \frac{K \cdot G(s)}{1 + K \cdot G(s)}$$

Poli zaprtozančnega sistema so rešitve **karakteristične enačbe**:

$$1 + K \cdot G(s) = 0 \quad \Rightarrow \quad K \cdot G(s) = -1$$

### Pravila za izris diagrama lege korenov (korenska krivulja)

Naj bo $G(s) = \dfrac{N(s)}{D(s)}$ z $n$ poli in $m$ ničlami ($n \geq m$):

| # | Pravilo | Opis |
|---|---------|------|
| 1 | **Število vej** | Enako številu polov $n$ |
| 2 | **Začetek** ($K=0$) | Vsaka veja začne na **polu** $G(s)$ |
| 3 | **Konec** ($K\to\infty$) | $m$ vej konča na **ničlah**, $n-m$ vej gre v neskončnost |
| 4 | **Simetrija** | Diagram je simetričen glede na realno os |
| 5 | **Lega na realni osi** | Korenska krivulja leži na delu realne osi, desno od katerega je **liho število** polov in ničel |
| 6 | **Asimptote** | $n - m$ asimptot z koti $\phi_k = \dfrac{(2k+1)\cdot 180°}{n-m},\ k=0,1,\ldots$ |
| 7 | **Težišče asimptot** | $\sigma_a = \dfrac{\sum \text{poli} - \sum \text{ničle}}{n - m}$ |

---

## Rešen primer — $G(s) = \dfrac{1}{s+1}$

### Postavitev problema

Enotna povratna zanka z ojačanjem $K > 0$:

$$G_{cl}(s) = \frac{K \cdot \dfrac{1}{s+1}}{1 + K \cdot \dfrac{1}{s+1}} = \frac{K}{s + 1 + K}$$

### Korak 1 — Določitev polov in ničel

$$G(s) = \frac{1}{s+1}$$

- **Pol:** $s_1 = -1$ (ena veja, $n = 1$)
- **Ničle:** ni ničel ($m = 0$)

### Korak 2 — Pravila za ta primer

| Pravilo | Vrednost |
|---------|----------|
| Število vej | 1 |
| Začetek ($K=0$) | $s = -1$ |
| Konec ($K \to \infty$) | $s \to -\infty$ (asimptota) |
| Lega na realni osi | Za polom $s=-1$, torej $s \in (-\infty,\ -1]$ |
| Asimptote | $n - m = 1$, kot: $\phi_0 = 180°$ |
| Težišče asimptot | $\sigma_a = \dfrac{-1}{1} = -1$ |

### Korak 3 — Korenska krivulja

$$\text{Za } K \geq 0: \quad s = -1 - K$$

Ko $K$ narašča od $0$ do $\infty$, se pol premika vzdolž realne osi od $-1$ proti $-\infty$:

```
                    K→∞          K=0
←←←←←←←←←←←←←←←←←←←×──────────×
-∞                   ...        -1        0
```

### Korak 4 — Stabilnost

Pol zaprtozančnega sistema: $s = -1 - K$

$$\text{Re}(s) = -1 - K < 0 \quad \forall K > 0$$

$$\boxed{\text{Sistem je stabilen za vse } K > 0}$$

### Korak 5 — Kritično ojačanje

Ker pol nikoli ne preide v desno polravnino, **ni kritičnega ojačanja** — sistem ne more postati nestabilen s povečevanjem $K$.

---

## MATLAB

```matlab
G = tf(1, [1 1]);
K_values = [0.5 1 2 5 10 20];
t = 0:0.01:8;

figure;
for i = 1:length(K_values)
    K = K_values(i);
    Gcl = feedback(K*G, 1);

    subplot(1,2,1); hold on;
    step(Gcl, t);
    title('Step response');

    subplot(1,2,2); hold on;
    p = pole(Gcl);
    plot(real(p), imag(p), 'x', 'MarkerSize', 10);
    title('Lega polov');
    xline(0,'k--'); yline(0,'k--');

    drawnow; pause(0.5);
end
```

---

*VP3 — Diskretni krmilni sistemi*

---

## Rešen primer 2 — $G(s) = \dfrac{1}{(s+1)(s-1)}$

### Postavitev problema

$$G(s) = \frac{1}{(s+1)(s-1)} = \frac{1}{s^2 - 1}$$

> ⚠️ Sistem je **odprtozančno nestabilen** — pol pri $s = +1$ leži v desni polravnini!

### Korak 1 — Določitev polov in ničel

- **Poli:** $s_1 = -1,\quad s_2 = +1 \quad$ ($n = 2$ veji)
- **Ničle:** ni ničel ($m = 0$)

### Korak 2 — Pravila za ta primer

| Pravilo | Vrednost |
|---------|----------|
| Število vej | 2 |
| Začetek ($K=0$) | $s = -1$ in $s = +1$ |
| Konec ($K \to \infty$) | obe veji po asimptotah v neskončnost |
| Lega na realni osi | Med poloma: $s \in [-1,\ +1]$ (desno je 1 pol → liho) |
| Asimptote | $n - m = 2$, kota: $\phi_{0,1} = 90°,\ 270°$ (imaginarni osi) |
| Težišče asimptot | $\sigma_a = \dfrac{(-1)+(+1)}{2} = 0$ |
| Točka odcepitve | $s = 0$ (po simetriji; ali iz $\frac{d}{ds}[s^2-1]=2s=0$) |

### Korak 3 — Korenska krivulja

Karakteristična enačba:

$$1 + \frac{K}{s^2 - 1} = 0 \quad \Rightarrow \quad s^2 = 1 - K$$

| Območje $K$ | Poli | Lega |
|-------------|------|------|
| $K = 0$ | $s = \pm 1$ | odprtozančni poli |
| $0 < K < 1$ | $s = \pm\sqrt{1-K}$ | realna os, premikata se k $0$ |
| $K = 1$ | $s = 0,\ 0$ | točka odcepitve |
| $K > 1$ | $s = \pm j\sqrt{K-1}$ | imaginarni osi, gresta v $\pm j\infty$ |

```
       Im
       ↑  K→∞
       |  ↑
  ×────┼──┼──────×   Re
 s=-1  | s=0   s=+1
       |  ↓
          K→∞
```

### Korak 4 — Stabilnost

$$\text{Re}(s) = 0 \quad \text{za } K > 1 \qquad \text{Re}(s) > 0 \quad \text{za } 0 < K < 1$$

$$\boxed{\text{Sistem je nestabilen za vse } K \geq 0}$$

- Za $0 < K < 1$: en pol še vedno v desni polravnini → **nestabilen**
- Za $K = 1$: poli pri $s = 0$ → **mejno stabilen**
- Za $K > 1$: poli na imaginarni osi → **mejno stabilen** (trajna nihanja)

> **Zaključek:** Enostavna enotna povratna zanka ne more stabilizirati sistema z nesosednjima poloma (eden stabilen, eden nestabilen). Za stabilizacijo bi potrebovali bolj kompleksnega regulatorja.

### MATLAB

```matlab
G = tf(1, [1 0 -1]);
K_values = [0.5 1 2 5 10];
t = 0:0.01:5;

figure;
for i = 1:length(K_values)
    K = K_values(i);
    Gcl = feedback(K*G, 1);

    subplot(1,2,1); hold on;
    step(Gcl, t);
    title('Step response'); ylim([-5 5]);

    subplot(1,2,2); hold on;
    p = pole(Gcl);
    plot(real(p), imag(p), 'x', 'MarkerSize', 10);
    title('Lega polov');
    xline(0,'k--'); yline(0,'k--');

    drawnow; pause(0.5);
end
```

---

*VP3 — Diskretni krmilni sistemi*
