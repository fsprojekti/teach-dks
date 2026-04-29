# VP4 — Diagram lege korenov za diskretne sisteme (Z-transformacija)

**Datum:** 22. 4. 2026  
**Tema:** Korenska krivulja v Z-domeni in analiza stabilnosti diskretnih sistemov

---

## Teorija — Diskretni sistemi in Z-transformacija

### Primerjava Laplaceove in Z-transformacije

| | **Zvezni sistemi** (Laplace) | **Diskretni sistemi** (Z) |
|---|---|---|
| Spremenljivka | $s$ | $z$ |
| Meja stabilnosti | Imaginarna os ($j\omega$) | Enotska krožnica ($\|z\| = 1$) |
| Stabilnost | $\text{Re}(s) < 0$ | $\|z\| < 1$ |
| Nestabilnost | $\text{Re}(s) > 0$ | $\|z\| > 1$ |
| Zveza | $z = e^{sT_s}$ | $s = \frac{1}{T_s}\ln z$ |

### Karakteristična enačba (diskretni sistem)

$$1 + K \cdot G(z) = 0$$

Poli zaprtozančnega sistema morajo biti **znotraj enotske krožnice**:

$$\boxed{|z_i| < 1 \quad \forall i \quad \Rightarrow \quad \text{sistem je stabilen}}$$

---

## Rešen primer 1 — $G(z) = \dfrac{1}{z - 0{,}5}$ (stabilen)

### Korak 1 — Poli in ničle

- **Pol:** $z_1 = 0{,}5$ (znotraj enotske krožnice → odprtozančno stabilen)
- **Ničle:** ni ničel ($m = 0$, $n = 1$)

### Korak 2 — Pravila

| Pravilo | Vrednost |
|---------|----------|
| Število vej | 1 |
| Začetek ($K=0$) | $z = 0{,}5$ |
| Konec ($K \to \infty$) | $z \to -\infty$ po realni osi |
| Lega na realni osi | $z \in (-\infty,\ 0{,}5]$ |
| Asimptota | kot $180°$ |
| Težišče asimptot | $\sigma_a = 0{,}5$ |

### Korak 3 — Korenska krivulja

Karakteristična enačba:

$$z - 0{,}5 + K = 0 \quad \Rightarrow \quad z = 0{,}5 - K$$

| $K$ | $z$ | $\|z\|$ | Stabilnost |
|-----|-----|---------|------------|
| $0$ | $0{,}5$ | $0{,}5$ | stabilen |
| $0{,}5$ | $0$ | $0$ | stabilen |
| $1$ | $-0{,}5$ | $0{,}5$ | stabilen |
| $1{,}5$ | $-1$ | $1$ | **mejno stabilen** |
| $2$ | $-1{,}5$ | $1{,}5$ | nestabilen |

### Korak 4 — Kritično ojačanje

$$|z| = 1 \quad \Rightarrow \quad |0{,}5 - K| = 1 \quad \Rightarrow \quad K_{kr} = 1{,}5$$

$$\boxed{\text{Sistem je stabilen za } 0 \leq K < 1{,}5}$$

```
     |←── nestabilno ──|── stabilno ──|
─────┼─────────────────┼──────────────×──────
    -∞                -1             0.5    Re(z)
                   K=1.5            K=0
```

---

## Rešen primer 2 — $G(z) = \dfrac{1}{(z-0{,}5)(z-1{,}2)}$ (odprtozančno nestabilen)

### Korak 1 — Poli in ničle

- **Pol 1:** $z_1 = 0{,}5$ — **znotraj** enotske krožnice ✓
- **Pol 2:** $z_2 = 1{,}2$ — **zunaj** enotske krožnice ✗ (odprtozančno nestabilen!)
- **Ničle:** ni ničel ($n = 2$, $m = 0$)

### Korak 2 — Pravila

| Pravilo | Vrednost |
|---------|----------|
| Število vej | 2 |
| Začetek ($K=0$) | $z = 0{,}5$ in $z = 1{,}2$ |
| Lega na realni osi | $z \in (-\infty,\ 0{,}5]$ in $z \in [1{,}2,\ +\infty)$ |
| Asimptote | $n-m=2$, kota: $90°$ in $270°$ |
| Težišče asimptot | $\sigma_a = \dfrac{0{,}5 + 1{,}2}{2} = 0{,}85$ |
| Točka odcepitve | $z = 0{,}85$ (po simetriji) |

### Korak 3 — Korenska krivulja

Karakteristična enačba:

$$z^2 - 1{,}7z + 0{,}6 + K = 0$$

Točka odcepitve iz $\frac{d}{dz}\left[(z-0{,}5)(z-1{,}2)\right] = 0$:

$$2z - 1{,}7 = 0 \quad \Rightarrow \quad z = 0{,}85$$

| $K$ | Poli | Stabilnost |
|-----|------|------------|
| $0$ | $0{,}5$ in $1{,}2$ | nestabilen ($z_2 > 1$) |
| $0{,}09$ | $0{,}85 \pm 0j$ | odcepitev |
| $K > 0{,}09$ | $0{,}85 \pm j\omega$ | $\|z\| = 0{,}85 < 1$ → **stabilen!** |
| $K_{kr}$ | na enotski krožnici | mejno stabilen |

Pogoj $|z|^2 = 1$: ker pola po odcepitvi potujeta po krožnici s polmerom $0{,}85$, **nikoli ne dosežeta enotske krožnice**.

Kritično ojačanje iz $|z|^2 = (0{,}85)^2 + \omega^2 = 1$:

$$\omega^2 = 1 - 0{,}7225 = 0{,}2775 \quad \Rightarrow \quad K_{kr} \approx 0{,}34$$

$$\boxed{\text{Sistem je stabilen za } 0{,}09 < K < 0{,}34}$$

> **Ključna ugotovitev:** Povratna zanka **CAN stabilize** odprtozančno nestabilen diskretni sistem — za razliko od zveznega primera z $1/((s+1)(s-1))$! Obstaja območje $K$, pri katerem so vsi poli znotraj enotske krožnice.

---

## MATLAB

```matlab
% Primer 1: G(z) = 1/(z - 0.5)
G1 = tf(1, [1 -0.5], 0.1);   % Ts = 0.1 s

figure;
subplot(1,2,1);
rlocus(G1);
hold on;
theta = linspace(0, 2*pi, 200);
plot(cos(theta), sin(theta), 'r--');   % enotska krožnica
title('Primer 1: G(z) = 1/(z-0.5)');

% Primer 2: G(z) = 1/((z-0.5)(z-1.2))
G2 = tf(1, conv([1 -0.5], [1 -1.2]), 0.1);

subplot(1,2,2);
rlocus(G2);
hold on;
plot(cos(theta), sin(theta), 'r--');
title('Primer 2: G(z) = 1/((z-0.5)(z-1.2))');

% Animacija: step response + lega polov (Primer 2)
K_values = [0.05 0.09 0.15 0.25 0.34 0.5];
figure;
for i = 1:length(K_values)
    K = K_values(i);
    Gcl = feedback(K*G2, 1);

    subplot(1,2,1); hold on;
    step(Gcl, 80);
    title('Step response'); ylim([-2 3]);

    subplot(1,2,2); hold on;
    p = pole(Gcl);
    plot(real(p), imag(p), 'x', 'MarkerSize', 10);
    plot(cos(theta), sin(theta), 'r--');
    title('Lega polov'); axis equal;
    xline(0,'k--'); yline(0,'k--');

    drawnow; pause(0.5);
end
```

---

*VP4 — Diskretni krmilni sistemi*
