# Mediciones: QE GrayLevel1

Tiempos de descarga en µs. Zumo quieto, misma iluminación. `timeout=0x00` en todas las filas.

## Parte 1 c) Posición de los sensores (S3 = 0)

| Sensor | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|
| sᵢ (m) | −0.045 | −0.010 | 0 | +0.010 | +0.045 |

## Parte 2 e) Punto (0.60, 0.80) m, r = 1.00 m, α = 0°

```
S1=  89 us | S2=  67 us | S3=  49 us | S4=  65 us | S5= 100 us | timeout=0x00
S1=  88 us | S2=  67 us | S3=  49 us | S4=  65 us | S5= 101 us | timeout=0x00
S1=  89 us | S2=  66 us | S3=  49 us | S4=  64 us | S5= 100 us | timeout=0x00
S1=  88 us | S2=  66 us | S3=  49 us | S4=  64 us | S5= 100 us | timeout=0x00
S1=  89 us | S2=  67 us | S3=  49 us | S4=  64 us | S5= 100 us | timeout=0x00
```

## Parte 2 f) Superficie clara

Muestra clara: esquina blanca del terreno, ≈ (0.05, 0.05) m, r ≈ 0.07 m. Foto: `fotos/muestra_clara.png`.

Tanda 1 (5 filas seguidas):

```
S1=  71 us | S2=  52 us | S3=  39 us | S4=  53 us | S5=  76 us | timeout=0x00
S1=  69 us | S2=  52 us | S3=  38 us | S4=  53 us | S5=  75 us | timeout=0x00
S1=  70 us | S2=  52 us | S3=  39 us | S4=  53 us | S5=  76 us | timeout=0x00
S1=  70 us | S2=  52 us | S3=  39 us | S4=  53 us | S5=  76 us | timeout=0x00
S1=  70 us | S2=  52 us | S3=  39 us | S4=  54 us | S5=  76 us | timeout=0x00
```

## Parte 2 f) Superficie oscura

Muestra oscura: esquina opuesta (la más oscura) del terreno, ≈ (1.45, 1.45) m, r ≈ 2.05 m, pegada a la orilla con el borde blanco del tapete. Foto: `fotos/muestra_oscura.png`.

Tanda 1 (5 filas seguidas):

```
S1= 378 us | S2= 382 us | S3= 239 us | S4= 353 us | S5= 429 us | timeout=0x00
S1= 381 us | S2= 385 us | S3= 241 us | S4= 355 us | S5= 432 us | timeout=0x00
S1= 382 us | S2= 387 us | S3= 242 us | S4= 357 us | S5= 435 us | timeout=0x00
S1= 380 us | S2= 384 us | S3= 240 us | S4= 354 us | S5= 433 us | timeout=0x00
S1= 379 us | S2= 384 us | S3= 240 us | S4= 354 us | S5= 431 us | timeout=0x00
```

Tanda 2 (5 filas seguidas, Zumo levantado y recolocado):

```
S1= 474 us | S2= 322 us | S3= 247 us | S4= 285 us | S5= 397 us | timeout=0x00
S1= 471 us | S2= 321 us | S3= 245 us | S4= 283 us | S5= 394 us | timeout=0x00
S1= 473 us | S2= 323 us | S3= 246 us | S4= 284 us | S5= 396 us | timeout=0x00
S1= 475 us | S2= 322 us | S3= 247 us | S4= 285 us | S5= 397 us | timeout=0x00
S1= 475 us | S2= 323 us | S3= 248 us | S4= 285 us | S5= 398 us | timeout=0x00
```

Tanda 3 (5 filas seguidas, Zumo levantado y recolocado):

```
S1= 463 us | S2= 402 us | S3= 240 us | S4= 293 us | S5= 366 us | timeout=0x00
S1= 464 us | S2= 404 us | S3= 241 us | S4= 293 us | S5= 367 us | timeout=0x00
S1= 464 us | S2= 403 us | S3= 242 us | S4= 293 us | S5= 367 us | timeout=0x00
S1= 462 us | S2= 401 us | S3= 240 us | S4= 292 us | S5= 365 us | timeout=0x00
S1= 464 us | S2= 403 us | S3= 242 us | S4= 294 us | S5= 366 us | timeout=0x00
```

## Tabla del inciso f)

Medianas en µs. La fila "Oscuro" es la mediana de las 15 filas (3 tandas × 5).

| Superficie | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|
| Claro | 70 | 52 | 39 | 53 | 76 |
| Oscuro, tanda 1 | 380 | 384 | 240 | 354 | 432 |
| Oscuro, tanda 2 | 474 | 322 | 247 | 285 | 397 |
| Oscuro, tanda 3 | 464 | 403 | 241 | 293 | 366 |
| **Oscuro** | **464** | **384** | **242** | **293** | **397** |
| Oscuro / claro | 6.6× | 7.4× | 6.2× | 5.5× | 5.2× |

Rango entre tandas oscuras: S3 240–247 µs (±1.5 %). Los sensores externos varían mucho más (S1 380–474, S4 285–354, S5 366–432), porque el Zumo quedó pegado a la orilla del terreno y al recolocarlo S1 y S5 se acercan o alejan del borde blanco del tapete.

## Parte 2 g) Radio constante r = 1.00 m, α = 0°

θ es el ángulo del punto medido desde la esquina blanca, no la orientación del Zumo.

Punto 1: (0.80, 0.60) m, θ = 36.9°

```
S1=  86 us | S2=  69 us | S3=  50 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  86 us | S2=  69 us | S3=  49 us | S4=  67 us | S5= 100 us | timeout=0x00
S1=  86 us | S2=  69 us | S3=  50 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  87 us | S2=  69 us | S3=  50 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  86 us | S2=  69 us | S3=  50 us | S4=  67 us | S5= 101 us | timeout=0x00
```

Punto 2: (0.30, 0.95) m, r = 0.996 m, θ = 72.5°

```
S1=  88 us | S2=  68 us | S3=  52 us | S4=  69 us | S5= 104 us | timeout=0x00
S1=  87 us | S2=  69 us | S3=  52 us | S4=  70 us | S5= 104 us | timeout=0x00
S1=  89 us | S2=  69 us | S3=  52 us | S4=  70 us | S5= 105 us | timeout=0x00
S1=  87 us | S2=  68 us | S3=  52 us | S4=  70 us | S5= 103 us | timeout=0x00
S1=  89 us | S2=  69 us | S3=  53 us | S4=  70 us | S5= 104 us | timeout=0x00
```

Punto 3: (0.942, 0.329) m, r = 0.998 m, θ = 19.3°

```
S1=  89 us | S2=  64 us | S3=  49 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  89 us | S2=  64 us | S3=  49 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  89 us | S2=  64 us | S3=  49 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  89 us | S2=  64 us | S3=  49 us | S4=  67 us | S5= 101 us | timeout=0x00
S1=  89 us | S2=  64 us | S3=  49 us | S4=  67 us | S5= 101 us | timeout=0x00
```

Punto 4: (0.60, 0.80) m, r = 1.00 m, θ = 53.1°. Mismas lecturas del inciso e).

## Tabla del inciso g)

Medianas en µs, α = 0°.

| Punto | (x, y) m | r (m) | θ | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|---|---|
| 3 | (0.942, 0.329) | 0.998 | 19.3° | 89 | 64 | 49 | 67 | 101 |
| 1 | (0.80, 0.60) | 1.000 | 36.9° | 86 | 69 | 50 | 67 | 101 |
| 4 | (0.60, 0.80) | 1.000 | 53.1° | 89 | 67 | 49 | 64 | 100 |
| 2 | (0.30, 0.95) | 0.996 | 72.5° | 88 | 69 | 52 | 70 | 104 |
| | | | **Rango** | 3 | 5 | 3 | 6 | 4 |
