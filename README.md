# teach-dks

Diskretni krmilni sistemi - laboratorijske vaje (VL), semester 2026.

## VL plan 2026 (kratki nazivi)

1. VL1: Branje senzorjev in pogon motorja
    
    VL1 je uvodna Arduino vaja, kjer studenti vzpostavijo osnovno komunikacijo z maketo, preberejo meritve senzorjev in izvedejo preprosto vodenje motorja. Cilj je, da skozi kratek primer razumejo razliko med odprtozančno in zaprtozančno zanko ter pripravijo osnovo za nadaljnje modeliranje in uglaševanje.
    
2. VL2: Simulink komunikacija, enkoderji in pogon motorja

    VL2 zdruzuje prejsnji vsebini VL2 in VL3: studenti v Simulinku postavijo bloke za komunikacijo z maketo, zajemajo signale obeh enkoderjev, iz pulzov izracunajo kotno hitrost ter izvedejo osnovne teste pogona motorja (npr. stopnica ali rampa) z analizo odziva.

3. VL3: Modeliranje in identifikacija sistema

    VL3 prevzame osrednjo vsebino prejsnjega VL4: na osnovi izmerjenih signalov studenti postavijo model sistema, ocenijo parametre (identifikacija) in kot rezultat pripravijo teoreticno prenosno funkcijo sistema za nadaljnje krmilne naloge.

4. VL4: PID uglaševanje v simulaciji in prenos na realni sistem

    VL4 temelji na prenosni funkciji iz VL3: studenti najprej uglašujejo PID v simulaciji in izberejo parametre glede na zeleni odziv, nato iste nastavitve prenesejo na realno maketo ter primerjajo razliko med simuliranim in izmerjenim odzivom.

5. VL5: Ziegler-Nichols PID (step response)

    VL5 uporablja Ziegler-Nichols metodo iz prehodnega (step) odziva: studenti dolocijo znacilne parametre procesa in izracunajo zacetne PID nastavitve, nato preverijo kakovost odziva na realnem sistemu.

6. VL6: Ziegler-Nichols PID (oscillatory mode)

    VL6 uporablja oscilacijski nacin Ziegler-Nichols metode: studenti povecujejo ojacanje do meje stabilnosti, izmerijo kriticno ojacanje in periodo oscilacij ter iz teh vrednosti dolocijo PID parametre in primerjajo odziv z VL5.

7. VL7: Frekvenčna analiza sistema
8. VL8: Model prostora stanj
9. VL9: Regulator stanj



## Povezani fajli

- Glavna Moodle tabela za 2026: moodle/Course_Diskretni_krmilni_sist..._.8096/index_2026.html
- Originalna tabela (referenca): moodle/Course_Diskretni_krmilni_sist..._.8096/index.html