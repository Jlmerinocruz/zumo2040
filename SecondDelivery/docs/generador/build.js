const fs = require("fs");
const path = require("path");
const d = require("docx");
const { Document, Packer, Paragraph, TextRun, ImageRun, Table, TableRow, TableCell, WidthType, BorderStyle,
  AlignmentType, ShadingType, LevelFormat, PageBreak, HighlightColor } = d;

const S = __dirname;
const SD = "/home/albertg/SistemasEmbebidosAvanzados/Reto/SecondDelivery";
const FONT = "Times New Roman";
const SZ = 22; // 11 pt
const TW = 9360; // ancho de texto (carta, márgenes 1")

// ---------- helpers ----------
function runs(x, base = {}) {
  // x: string | array of [text, opts] | TextRun
  if (typeof x === "string") return [new TextRun({ text: x, font: FONT, size: SZ, ...base })];
  return x.map((r) => (typeof r === "string" ? new TextRun({ text: r, font: FONT, size: SZ, ...base })
    : new TextRun({ text: r[0], font: FONT, size: SZ, ...base, ...r[1] })));
}
const P = (x, o = {}) => new Paragraph({ children: runs(x, o.run), spacing: { after: 120, line: 264 }, alignment: o.align, ...o.p });
const B = (t) => [t, { bold: true }];
const I = (t) => [t, { italics: true }];
const H1 = (t) => new Paragraph({ children: runs(t, { bold: true }), spacing: { before: 240, after: 120 }, keepNext: true });
const H2 = (t) => new Paragraph({ children: runs(t, { bold: true }), spacing: { before: 200, after: 100 }, keepNext: true });
const bullet = (x) => new Paragraph({ children: runs(x), numbering: { reference: "bul", level: 0 }, spacing: { after: 60, line: 264 } });
const numbered = (x, ref = "num") => new Paragraph({ children: runs(x), numbering: { reference: ref, level: 0 }, spacing: { after: 60, line: 264 } });
const caption = (t) => new Paragraph({ children: runs(t, { italics: true }), spacing: { before: 60, after: 200 }, keepLines: true });
const pageBreak = () => new Paragraph({ children: [new PageBreak()] });

function img(file, widthIn, align = AlignmentType.CENTER) {
  const buf = fs.readFileSync(file);
  const { w, h } = pngSize(buf);
  const wpx = Math.round(widthIn * 96), hpx = Math.round(wpx * h / w);
  return new Paragraph({ alignment: align, keepNext: true, spacing: { before: 120, after: 0 },
    children: [new ImageRun({ type: "png", data: buf, transformation: { width: wpx, height: hpx },
      altText: { title: path.basename(file), description: path.basename(file), name: path.basename(file) } })] });
}
function pngSize(buf) { return { w: buf.readUInt32BE(16), h: buf.readUInt32BE(20) }; }

const border = { style: BorderStyle.SINGLE, size: 4, color: "000000" };
const borders = { top: border, bottom: border, left: border, right: border };
function table(rows, widths, o = {}) {
  const total = widths.reduce((a, b) => a + b, 0);
  return new Table({
    width: { size: total, type: WidthType.DXA }, columnWidths: widths,
    rows: rows.map((r, ri) => new TableRow({ cantSplit: true, tableHeader: ri === 0,
      children: r.map((c, ci) => new TableCell({ borders, width: { size: widths[ci], type: WidthType.DXA },
        margins: { top: 40, bottom: 40, left: 80, right: 80 },
        children: (Array.isArray(c) && c.length && c[0] instanceof Paragraph ? c :
          [new Paragraph({ children: runs(c, { size: o.size || SZ, bold: false }), spacing: { after: 0 } })]) })) })),
  });
}
const spacer = () => new Paragraph({ children: [], spacing: { after: 60 } });

function code(lines) {
  return lines.map((l, i) => new Paragraph({
    children: runs(l === "" ? " " : l, { size: SZ }),
    shading: { type: ShadingType.CLEAR, color: "auto", fill: "F2F2F2" },
    spacing: { after: 0, line: 240 }, keepLines: true, keepNext: i < lines.length - 1,
  }));
}
// marcador para que el equipo pegue fotos / links
function placeholder(t) {
  return new Paragraph({ alignment: AlignmentType.CENTER, spacing: { before: 120, after: 60 },
    border: { top: { style: BorderStyle.DASHED, size: 6, color: "7F7F7F", space: 6 }, bottom: { style: BorderStyle.DASHED, size: 6, color: "7F7F7F", space: 6 },
      left: { style: BorderStyle.DASHED, size: 6, color: "7F7F7F", space: 6 }, right: { style: BorderStyle.DASHED, size: 6, color: "7F7F7F", space: 6 } },
    children: [new TextRun({ text: t, font: FONT, size: SZ, highlight: HighlightColor.YELLOW })] });
}

// ---------- contenido ----------
const FIG = (n) => path.join(S, "fig", n);
const c = [];

// Portada
c.push(new Paragraph({ spacing: { before: 0, after: 600 }, children: [new ImageRun({ type: "png", data: fs.readFileSync(path.join(S, "rep/logo-000.png")),
  transformation: { width: 335 * 0.95, height: 100 * 0.95 }, altText: { title: "Logo", description: "Tecnológico de Monterrey", name: "logo" } })] }));
const cen = (x, o = {}) => new Paragraph({ alignment: AlignmentType.CENTER, children: runs(x, o.run), spacing: { after: o.after ?? 80 } });
c.push(cen([B("Challenge: Second Delivery")]));
c.push(cen([B("Seguimiento de waypoints con posicionamiento local y PCB")]));
c.push(cen([B("17/11/2026")], { after: 1400 }));
c.push(cen([B("Autor:")], { after: 160 }));
["Mauricio Rodríguez Méndez - A00840901", "Jose Luis Merino Cruz - A00840956",
  "Albert Gerardo Constantino Salazar - A01664787", "Claudio Patricio Campos Ajuria - A01738878"].forEach((n) => c.push(cen(n, { after: 160 })));
c.push(new Paragraph({ children: [], spacing: { after: 1400 } }));
c.push(cen([B("Profesor:")], { after: 160 }));
c.push(cen("Alfonso Ávila Ortega", { after: 160 }));
c.push(cen([I("Diseño de sistemas embebidos avanzados (Gpo 601)")]));
c.push(pageBreak());

// 2. Documentación
c.push(H1("1. Documentación"));
c.push(H2("1a. Diagrama de contexto actualizado"));
c.push(P("Respecto a la entrega anterior se agregan la pista con sus waypoints, el router de la red embedded como intermediario entre la PC de la cámara y el tractor, los botones del operador y el terreno con gradiente de gris de la parte E."));
c.push(img(FIG("fig_contexto.png"), 6.5));
c.push(caption("Figura 1. Diagrama de contexto del tractor autónomo."));

c.push(H2("1b. Tabla de requisitos actualizada"));
const req = [
  ["ID", "Requisito", "Tipo", "Cómo se verificó / resultado"],
  ["R1", "La PCB comunica la Pico 2 W y el Zumo por CAN a 500 kbps", "Funcional", "Loopback SPI y 10/10 mensajes en ambas direcciones; parte A de 50 cm"],
  ["R2", "La Pico 2 W recibe la pose del marcador 4 por WiFi y la reenvía como LP_POSE (0x100)", "Funcional", "Serial de la Pico: ~9.5 poses/s recibidas y confirmadas por CAN"],
  ["R3", "Los 14 waypoints se guardan en memoria del Zumo", "Funcional", "Tabla WAYPOINTS en waypoints.c; el Zumo la imprime al arrancar"],
  ["R4", "Se identifican las vueltas: 5 de 180° y 2 de 90°", "Funcional", "Tabla de tramos impresa (arco, radio, centro y sentido)"],
  ["R5", "La vuelta inicia y termina en P14; no arranca si el Zumo está a más de 10 cm", "Funcional", "Log: “# FIN en P14” en las 3 vueltas"],
  ["R6", "En cada pose nueva se ubica en el tramo, elige el siguiente waypoint en orden y actualiza velocidad y giro", "Funcional", "Líneas L,… del log (tramo, avance t, e_y)"],
  ["R7", "No se salta waypoints", "Funcional", "El tramo solo avanza cuando t ≥ 1; recorrió los 14 tramos en orden"],
  ["R8", "Las vueltas de 90° y 180° se trazan como arcos que mantienen al tractor sobre la pista", "Funcional", "Gráfica de ruta deseada vs medida (Figura 8)"],
  ["R9", "Si no hay pose sigue con el último comando hasta 800 ms, luego se detiene; a los 3 s termina", "Seguridad", "Log: “# PAUSA” y “# ALTO”"],
  ["R10", "B detiene la vuelta; la calibración solo se activa manteniendo B 3 s", "Seguridad", "Vuelta 4: la línea “Vuelta:” del log muestra K = 7000, DEAD = 850, WB = 0.085"],
  ["R11", "Error lateral menor a ±5 cm", "Desempeño", "Vueltas 1 y 2: máx. 38–39 mm. Vuelta 4: RMS 21 mm, máx. 92 mm en P6→P7 (no se cumplió en ese tramo)"],
  ["R12", "Vuelta lo más rápida posible (parte D)", "Desempeño", "102.15 → 93.48 → 89.04 s"],
  ["R13", "Método y código para medir α con los 5 sensores", "Funcional", "alpha.c (sección 4)"],
];
c.push(table(req, [600, 3700, 1160, 3900], { size: 20 }));
c.push(caption("Tabla 1. Requisitos del sistema y su verificación."));

c.push(H2("1c. Diagrama de despliegue"));
c.push(P("El software de posicionamiento corre en la PC del laboratorio y manda la pose por broadcast UDP a través del router. En la PCB, la Pico 2 W ejecuta PicoLPBridge y el Zumo ejecuta ZumoWaypointFollower; los dos se comunican solo por el bus CAN. La Pico y los dos TJA1050 se alimentan con el power bank, porque con la alimentación desde el Zumo el CAN no transmitía (problema 2)."));
c.push(img(FIG("fig_despliegue.png"), 6.5));
c.push(caption("Figura 2. Diagrama de despliegue del sistema."));

// 3. PCB
c.push(H1("2. Implementación con PCB"));
c.push(H2("2a. PCB con los componentes soldados"));
c.push(P("La PCB lleva la Pico 2 W soldada, sockets de 7 pines para los dos módulos CAN (MCP2515 + TJA1050), la tira de 5 pines hacia el Zumo (CS, MISO, MOSI, SCK, INT), el conector VSW con dos diodos 1N4001 y las borneras CAN H/L de cada módulo."));
c.push(placeholder("[Insertar foto: cara superior de la PCB con todos los componentes soldados]"));
c.push(caption("Figura 3. Cara superior de la PCB."));
c.push(P("En el diseño original las etiquetas MISO, MOSI y SCK eran las mismas para la Pico y para el Zumo, así que el cobre unía los dos buses SPI y había dos maestros sobre las mismas líneas. Se corrigió con 4 cortes de pista y un puente (Figura 4)."));
c.push(img(path.join(SD, "../pcb_agustin/cortes_pcb.png"), 6.3));
c.push(caption("Figura 4. Cortes y puente para separar el SPI de la Pico y el del Zumo."));

c.push(H2("2b. Tractor con la PCB y el marcador"));
c.push(placeholder("[Insertar foto: Zumo con la PCB montada y el marcador ArUco arriba]"));
c.push(caption("Figura 5. Tractor con la PCB incorporada y el marcador de posicionamiento local."));
c.push(bullet("El marcador va sobre un soporte rígido fijado a la PCB. Con el soporte suelto de la primera prueba el marcador se cayó a media vuelta (problema 5)."));
c.push(bullet("La Pico 2 W se alimenta con el power bank por su micro-USB; el Zumo usa sus 4 pilas AA (≈6.3 V)."));

c.push(H2("2c. Video del avance de 50 cm"));
c.push(P("Se repitió la Parte 2 del experimento de posicionamiento local, ahora con la PCB: el Zumo arranca en el waypoint inicial y se detiene cuando la cámara marca que avanzó 50 cm en y."));
c.push(placeholder("Video: [pegar el link del video de los 50 cm]"));
c.push(table([
  ["Medición", "Resultado"],
  ["y inicial según la cámara", "126 mm"],
  ["y final según la cámara", "637 mm"],
  ["Avance según la cámara", "511 mm (11 mm de más por la inercia al frenar)"],
  ["Deriva lateral (x) y de ángulo", "−375 → −427 mm; 89° → 101°"],
], [4200, 5160]));
c.push(caption("Tabla 2. Resultado del avance de 50 cm con la PCB."));
c.push(P("Esta prueba solo controla la distancia, no el rumbo, por eso el tractor se desvía en x y en ángulo. El seguidor de la vuelta completa sí corrige ambos."));

// 4. Vuelta completa
c.push(H1("3. Vuelta completa usando posicionamiento local"));
c.push(H2("3a. Mapa calibrado con los waypoints numerados"));
c.push(P("Los waypoints se tomaron colocando el Zumo sobre la pista, sin marcas de plumón, y leyendo la pose de la cámara (mediana de 8 poses con el Zumo quieto; variación ±1–2 mm y ±1°). Se eligieron con este criterio:"));
c.push(bullet([B("Un waypoint a la entrada y otro a la salida de cada vuelta. "), "Ahí cambia el movimiento de recta a arco. Con esos dos puntos y sus ángulos el Zumo calcula el centro y el radio del arco (R ≈ 7 cm), así que no hacen falta puntos intermedios."]));
c.push(bullet([B("Las rectas no llevan puntos intermedios: "), "el seguidor corrige el error lateral contra la línea entre los dos waypoints de los extremos."]));
c.push(bullet([B("Vueltas de 90°: "), "P11 → P12 y P13 → P14. ", B("Vueltas de 180°: "), "P1 → P2, P3 → P4, P5 → P6, P7 → P8 y P9 → P10."]));
c.push(bullet([B("Posición inicial y final: P14 (−0.372, 0.193, 93°). "), "Está al inicio de la recta más larga, así que el Zumo arranca alineado, y al cerrar la vuelta llega saliendo de una curva a baja velocidad, lo que facilita detenerse."]));
c.push(img(FIG("fig_mapa.png"), 4.6));
c.push(caption("Figura 6. Mapa calibrado con los 14 waypoints. Los cuadros marcan las vueltas de 90°."));

c.push(H2("3b. Tabla de waypoints en el código"));
const wps = [
  ["P1", "−0.375", "1.005", "92", "Entrada vuelta 180°"], ["P2", "−0.226", "1.062", "−88", "Salida vuelta 180°"],
  ["P3", "−0.214", "0.359", "−88", "Entrada vuelta 180°"], ["P4", "−0.075", "0.355", "92", "Salida vuelta 180°"],
  ["P5", "−0.073", "1.034", "94", "Entrada vuelta 180°"], ["P6", "0.075", "1.058", "−88", "Salida vuelta 180°"],
  ["P7", "0.074", "0.362", "−89", "Entrada vuelta 180°"], ["P8", "0.223", "0.346", "91", "Salida vuelta 180°"],
  ["P9", "0.225", "1.038", "91", "Entrada vuelta 180°"], ["P10", "0.371", "1.067", "−88", "Salida vuelta 180°"],
  ["P11", "0.373", "0.163", "−88", "Entrada vuelta 90°"], ["P12", "0.304", "0.075", "−179", "Salida vuelta 90°"],
  ["P13", "−0.278", "0.079", "−179", "Entrada vuelta 90°"], ["P14", "−0.372", "0.193", "93", "Salida vuelta 90°, inicio y fin"],
];
c.push(table([["WP", "x (m)", "y (m)", "Ángulo (°)", "Papel"], ...wps], [900, 1300, 1300, 1400, 4460]));
c.push(caption("Tabla 3. Waypoints medidos con la cámara el 17/11."));
c.push(P("La tabla se guarda como un arreglo constante en waypoints.c. El orden del arreglo es el orden de recorrido; WAYPOINT_START = 13 indica que la vuelta empieza en P14, pasa por P1 … P13 y regresa a P14."));
c.push(...code([
  "typedef struct { float x, y, theta_deg; } waypoint_t;   // waypoints.h",
  "#define WAYPOINT_COUNT 14",
  "#define WAYPOINT_START 13                               // P14",
  "",
  "const waypoint_t WAYPOINTS[WAYPOINT_COUNT] = {",
  "    /* P1  */ {-0.375f, 1.005f,   92.0f},  // entrada vuelta 180° (arriba)",
  "    /* P2  */ {-0.226f, 1.062f,  -88.0f},  // salida",
  "    /* ...  P3 a P12 igual que la Tabla 3 ... */",
  "    /* P13 */ {-0.278f, 0.079f, -179.0f},  // entrada vuelta 90°",
  "    /* P14 */ {-0.372f, 0.193f,   93.0f},  // salida = INICIO y FIN",
  "};",
]));
c.push(spacer());
c.push(P("Al arrancar, follower.c recorre la tabla y arma los tramos de la ruta:"));
c.push(bullet("Si dos waypoints seguidos tienen el mismo ángulo, el tramo es una recta entre ellos."));
c.push(bullet("Si el ángulo cambia, el tramo es un arco. Su centro y su radio se calculan con los dos puntos y sus rumbos (centro = A + R·n_A = B + R·n_B, por mínimos cuadrados), y el sentido de giro con un producto cruz."));
c.push(bullet("Resultado: 7 rectas, 5 arcos de 180° y 2 de 90°. El Zumo imprime esta tabla de tramos al encender, lo que sirve para revisar la ruta antes de correr."));

c.push(H2("3c. Diagrama de actividades"));
c.push(P("En cada pose nueva el Zumo se ubica en el tramo actual, revisa si ya llegó al siguiente waypoint, y calcula la velocidad y el giro. La pose llega por WiFi en ráfagas (un hueco de ~450 ms cada ~0.8 s), así que durante un hueco corto sigue con el último comando y solo se detiene si pasan más de 800 ms."));
c.push(img(FIG("fig_actividad.png"), 5.6));
c.push(caption("Figura 7. Diagrama de actividades del seguimiento de la vuelta, de P14 a P14."));
c.push(P("Control usado en cada pose:"));
c.push(bullet([B("Velocidad: "), "0.08 m/s en rectas y 0.06 m/s en arcos, frenando antes de entrar a cada arco."]));
c.push(bullet([B("Giro: "), "ω = v/R (solo en arcos) + k_θ·(θ_ref − θ), con θ_ref = tangente − atan(k_e·e_y). k_θ = 4 y k_e se ajusta con la velocidad para mantener el amortiguamiento ζ = 0.8."]));
c.push(bullet([B("Motores: "), "v_izq = v − ω·WB/2 y v_der = v + ω·WB/2, convertidos a cuentas con MOTOR_K = 7000 cuentas/(m/s) y una zona muerta MOTOR_DEAD = 850 cuentas (WB = 0.085 m, obtenidos con la calibración del botón B)."]));

c.push(P("La Tabla 4 relaciona cada paso que pide la parte C con lo que hace el firmware en cada pose:"));
c.push(table([
  ["Paso de la guía (parte C)", "Cómo lo hace el Zumo", "Dónde"],
  ["Recibir una medición fresca (x, y, θ)", "Lee LP_POSE (0x100) del MCP2515. Si hubo un hueco de más de 250 ms, espera 20 ms a que llegue la ráfaga y usa solo la pose más nueva", "main.c: poll_pose(), run_lap()"],
  ["Ubicar al tractor en el tramo actual", "Compensa el retardo de la cámara (120 ms) y proyecta la pose sobre la recta o el arco: avance t (0 a 1) y error lateral e_y", "follower.c: follower_update()"],
  ["Identificar el objetivo siguiente en el orden guardado", "Cuando t ≥ 1 pasa al siguiente tramo de la tabla. Nunca retrocede ni se salta un waypoint", "follower.c: follower_update()"],
  ["Definir el giro necesario", "ω = v/R en arcos + k_θ·(θ_ref − θ), con θ_ref = tangente − atan(k_e·e_y)", "follower.c"],
  ["Definir la velocidad de avance", "0.08 m/s en rectas y 0.06 m/s en arcos, frenando antes de cada arco para llegar a la velocidad del arco", "follower.c"],
  ["Actualizar los motores izquierdo y derecho", "v − ω·WB/2 (izquierda) y v + ω·WB/2 (derecha), convertidas a cuentas con MOTOR_K y la zona muerta MOTOR_DEAD", "main.c: set_wheels()"],
  ["Detener el tractor en la posición final", "Al terminar el tramo P13 → P14 pone los motores en 0 e imprime el tiempo de vuelta", "main.c: run_lap()"],
  ["Vuelta de 90° calculada para seguir sobre la pista", "No gira en su lugar: traza un arco cuyo centro y radio (R ≈ 7 cm) salen de los waypoints de entrada y salida medidos sobre la pista", "follower.c: follower_init()"],
], [2600, 4760, 2000], { size: 20 }));
c.push(caption("Tabla 4. Pasos de la parte C y su implementación."));
c.push(H2("3d. Ruta deseada vs trayectoria medida"));
c.push(img(FIG("fig_vuelta4.png"), 4.6));
c.push(caption("Figura 8. Ruta deseada y trayectoria medida por la cámara en la vuelta 4."));
c.push(P("El Zumo sigue la ruta en las rectas largas y en las vueltas de 90°. El error más grande (92 mm) aparece al salir de la vuelta P5 → P6: sale 6.5 cm abierto, entra a la recta con un ángulo de ~50° y zigzaguea hasta P7. En las vueltas 1 y 2, más lentas por las pausas, el error máximo fue de 38–39 mm."));

c.push(H2("3e. Tiempo de tres vueltas"));
c.push(table([
  ["Vuelta", "Tiempo (s)", "Error lateral RMS (mm)", "Error máx. (mm)", "Pausas por falta de pose", "Notas"],
  ["1", "102.15", "15.3", "38", "68", "Tabla anterior de waypoints; pausa a los 400 ms sin pose"],
  ["2", "93.48", "16.7", "39", "61", "Igual que la vuelta 1"],
  ["4", "89.04", "21.3", "92", "3", "Waypoints nuevos; sigue con el último comando hasta 800 ms"],
  ["Promedio", "94.89", "17.8", "—", "—", ""],
], [1000, 1100, 1500, 1200, 1500, 3060], { size: 20 }));
c.push(caption("Tabla 5. Tiempos de las tres vueltas completas (17/11, v_recta = 0.08 m/s, v_arco = 0.06 m/s)."));
c.push(P("La vuelta 3 no se completó: alguien tocó B con el Zumo detenido, se recalibró mal colocado y la separación de orugas quedó en 0.19 m (problema 7). Las tres vueltas se hicieron a la misma velocidad; la diferencia de tiempo coincide con el número de pausas (68, 61 y 3)."));

c.push(H2("3f. Video de la vuelta completa"));
c.push(placeholder("Video: [pegar el link del video de la vuelta 4]"));
c.push(H2("3g. Estrategia para reducir el tiempo de la vuelta (parte D)"));
c.push(P("La meta fue bajar el tiempo sin salirse de la ruta ni saltarse waypoints. Lo que se probó en la pista:"));
c.push(numbered([B("Perfil de velocidad por tramo. "), "En las rectas acelera hasta v_recta y antes de cada arco frena con v = √(v_arco² + 2·a·d), donde d es la distancia que falta para el arco (a = 0.6 m/s²). En el arco no se detiene a girar: avanza mientras gira con ω = v/R."], "numD"));
c.push(numbered([B("Calibración del motor con la cámara (botón B). "), "Se midieron MOTOR_K = 7000 cuentas/(m/s), la zona muerta MOTOR_DEAD = 850 cuentas y WB = 0.085 m, y se cargaron en el firmware."], "numD"));
c.push(numbered([B("Quitar las pausas por los huecos de WiFi. "), "En las vueltas 1 y 2 el Zumo se detuvo 61–68 veces por vuelta. Al seguir con el último comando hasta 800 ms y usar solo la pose más nueva de cada ráfaga, en la vuelta 4 las pausas bajaron a 3."], "numD"));
c.push(numbered([B("Velocidad. "), "Se probó 0.20 m/s y la cámara dejó de reconocer el marcador en cuanto el Zumo se movió, por eso las vueltas se hicieron a 0.08 m/s en rectas y 0.06 m/s en arcos."], "numD"));
c.push(table([
  ["Vuelta", "Cambio respecto a la anterior", "Pausas", "Tiempo (s)"],
  ["1", "Versión base: pausa a los 400 ms sin pose", "68", "102.15"],
  ["2", "Misma versión (repetición)", "61", "93.48"],
  ["4", "Sigue con el último comando hasta 800 ms, usa la pose más nueva de cada ráfaga, waypoints medidos de nuevo", "3", "89.04"],
], [900, 5660, 1100, 1700]));
c.push(caption("Tabla 6. Evolución del tiempo de vuelta con cada cambio."));

// 5. Alfa
c.push(H1("4. Posicionamiento con nivel de gris: método para medir α"));
c.push(P("El terreno se oscurece con la distancia r a la esquina blanca, así que cada sensor de reflectancia indica a qué radio está. Si la fila de sensores es tangente al arco (α = 0), los 5 sensores ven el mismo gris; si está girada, los de un lado ven más oscuro que los del otro. La diferencia entre ellos da el ángulo."));
c.push(img(FIG("fig_alfa.png"), 6.3));
c.push(caption("Figura 9. Geometría usada para calcular α."));
c.push(P("Secuencia de pasos:"));
c.push(numbered([B("Calibración (una vez): "), "con α = 0 se leen los 5 tiempos de descarga en varios radios y se guarda una tabla t_i(r) por sensor, porque cada sensor tiene ganancia distinta (S1 y S5 leen ~1.8 veces S3)."]));
c.push(numbered([B("Lectura a radio: "), "cada tiempo t_i se convierte a radio r_i con la tabla de su sensor (interpolación lineal por tramos)."]));
c.push(numbered([B("Geometría: "), "el sensor i está a s_i = −45, −10, 0, +10, +45 mm del central. Si la fila forma un ángulo α con la tangente, r_i = r₀ + s_i·sen α."]));
c.push(numbered([B("Ajuste: "), "con los 5 puntos (s_i, r_i) se ajusta una recta por mínimos cuadrados; la pendiente es sen α."]));
c.push(numbered([B("Ángulo: "), "α = asen(pendiente), en el rango [−90°, 90°]."]));
c.push(P("Código (alpha.c):"));
c.push(...code([
  "const float ALPHA_S[ALPHA_SENSORS] = {-0.045f, -0.010f, 0.0f, 0.010f, 0.045f};",
  "",
  "float alpha_radius(const alpha_cal_t *cal, int sensor, float t_us) {",
  "    const float *t = cal->t[sensor];",
  "    int n = cal->n, k = 0;",
  "    while (k < n - 2 && t_us > t[k + 1]) k++;       // tramo de la tabla",
  "    float dt = t[k + 1] - t[k];",
  "    if (dt < 1e-3f) return cal->r[k];",
  "    return cal->r[k] + (t_us - t[k]) * (cal->r[k + 1] - cal->r[k]) / dt;",
  "}",
  "",
  "bool alpha_estimate(const alpha_cal_t *cal, const float t_us[ALPHA_SENSORS],",
  "                    float *alpha_rad, float *r0) {",
  "    bool in_range = true;",
  "    float r[ALPHA_SENSORS], sm = 0.0f, rm = 0.0f;",
  "    for (int i = 0; i < ALPHA_SENSORS; i++) {",
  "        if (t_us[i] < cal->t[i][0] || t_us[i] > cal->t[i][cal->n - 1]) in_range = false;",
  "        r[i] = alpha_radius(cal, i, t_us[i]);",
  "        sm += ALPHA_S[i];  rm += r[i];",
  "    }",
  "    sm /= ALPHA_SENSORS;  rm /= ALPHA_SENSORS;",
  "    float num = 0.0f, den = 0.0f;",
  "    for (int i = 0; i < ALPHA_SENSORS; i++) {",
  "        num += (ALPHA_S[i] - sm) * (r[i] - rm);",
  "        den += (ALPHA_S[i] - sm) * (ALPHA_S[i] - sm);",
  "    }",
  "    float sin_a = num / den;                          // pendiente = sen(alfa)",
  "    if (sin_a > 1.0f)  sin_a = 1.0f;",
  "    if (sin_a < -1.0f) sin_a = -1.0f;",
  "    *alpha_rad = asinf(sin_a);",
  "    *r0 = rm - sm * sin_a;",
  "    return in_range;",
  "}",
]));
c.push(spacer());

// 6. Problemas
c.push(H1("5. Problemas encontrados y cómo se resolvieron"));
c.push(P("Se encontraron nueve problemas, en el orden en que aparecieron:"));
const prob = [
  ["#", "Problema", "Causa", "Solución"],
  ["1", "Con el multímetro, SCK y MOSI de la Pico daban continuidad con los del Zumo", "La PCB unía en cobre el SPI de la Pico y el del Zumo (mismas etiquetas MISO/MOSI/SCK): dos maestros en las mismas líneas", "4 cortes de pista y un puente de GP18 al pin 6 del módulo de la Pico (Figura 4)"],
  ["2", "Con la Pico alimentada desde la PCB, la Pico se conectaba al WiFi pero el CAN no transmitía", "La alimentación desde VSW pasa por dos diodos (caída medida de 1.2 V). Con la misma Pico alimentada por USB el CAN sí funcionaba", "Se alimenta la Pico (y con ella los TJA1050) con el power bank"],
  ["3", "No llegaba telemetría a la Pico ni a la laptop", "La PC de la cámara estaba conectada a otra red", "Se conectó la PC del laboratorio a la red embedded"],
  ["4", "El Zumo hacía 61–68 pausas por vuelta “sin pose”", "La pose llega por WiFi en ráfagas: hueco de ~450 ms cada ~0.8 s y luego 4–5 poses viejas en 10 ms", "Durante un hueco ≤ 800 ms sigue con el último comando, y de cada ráfaga usa solo la pose más nueva"],
  ["5", "La cámara dejaba de ver el marcador; a media vuelta el marcador se cayó", "A 0.20 m/s la imagen sale movida; el soporte del marcador estaba suelto", "Velocidad de 0.08 m/s en rectas y 0.06 m/s en arcos; soporte rígido del marcador"],
  ["6", "Al presionar A decía “No hay pose reciente”", "Mientras A estaba presionado el programa no leía el CAN", "Al soltar A espera una pose nueva (hasta 500 ms) antes de decidir"],
  ["7", "La vuelta 3 dio giros bruscos y no terminó", "Un toque de B en reposo recalibró con el Zumo mal colocado: WHEEL_BASE ≈ 0.19 m", "La calibración solo se activa manteniendo B 3 s; la línea “Vuelta:” imprime K, DEAD y WB"],
  ["8", "La ruta no coincidía con la pista (hasta ~8 cm)", "El marcador quedó en otra posición sobre el Zumo al cambiar del protoboard a la PCB", "Se volvieron a medir los 14 waypoints con el soporte de la PCB"],
  ["9", "Al conectar el USB el Zumo arrancaba en modo BOOTSEL", "El botón B del Zumo es también BOOTSEL", "picotool reboot o botón Reset"],
];
c.push(table(prob, [450, 2600, 3100, 3210], { size: 20 }));
c.push(caption("Tabla 7. Problemas encontrados y cómo se resolvieron."));

const doc = new Document({
  creator: "Equipo TE2004B", title: "Challenge: Second Delivery",
  styles: { default: { document: { run: { font: FONT, size: SZ } } } },
  numbering: { config: [
    { reference: "bul", levels: [{ level: 0, format: LevelFormat.BULLET, text: "•", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
    { reference: "numD", levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
    { reference: "num", levels: [{ level: 0, format: LevelFormat.DECIMAL, text: "%1.", alignment: AlignmentType.LEFT, style: { paragraph: { indent: { left: 720, hanging: 360 } } } }] },
  ] },
  sections: [{ properties: { page: { size: { width: 12240, height: 15840 }, margin: { top: 1440, bottom: 1440, left: 1440, right: 1440 } } }, children: c }],
});
Packer.toBuffer(doc).then((b) => { fs.writeFileSync(process.argv[2], b); console.log("ok"); });
