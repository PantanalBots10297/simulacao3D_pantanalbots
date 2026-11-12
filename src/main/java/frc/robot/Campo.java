package frc.robot;

/**
 * Medidas do campo usado na simulação (2025, versão AndyMark).
 *
 * Os valores são os oficiais da WPILib (arquivo 2025-reefscape-andymark.json).
 * O ponto (0, 0) é o canto do campo do lado da aliança azul.
 */
public final class Campo {

    /** Comprimento do campo, no eixo X (metros). */
    public static final double COMPRIMENTO = 17.548;

    /** Largura do campo, no eixo Y (metros). */
    public static final double LARGURA = 8.042;

    // =========================================================
    // OBSTÁCULOS DO MEIO DO CAMPO
    // =========================================================

    /**
     * REEF: o hexágono onde vão os Corais. Tratamos como um círculo.
     * O hexágono tem 0,83 m do centro até a face e 0,96 m até a ponta;
     * 0,90 m fica no meio-termo.
     */
    public static final double REEF_RAIO = 0.90;

    /** Centros dos dois Reefs {x, y}, calculados pelas AprilTags 18/21 (azul) e 7/10 (vermelho). */
    public static final double[][] REEFS = {
            { 4.489, 4.021 },   // azul
            { 13.059, 4.021 },  // vermelho
    };

    /**
     * BARGE (Game Manual 2025, seção 5.4): estrutura de treliça no meio do
     * campo, onde ficam a Net e as Cages. A treliça horizontal fica a 1,57 m
     * do chão: por baixo dela o robô e as Fuels passam. No chão, só
     * bloqueiam o suporte central e as 6 Cages penduradas.
     * Faces pelas AprilTags 14 (x = 8,272) e 4 (x = 9,276).
     */
    public static final double BARGE_X_MIN = 8.272;
    public static final double BARGE_X_MAX = 9.276;
    public static final double BARGE_X_CENTRO = (BARGE_X_MIN + BARGE_X_MAX) / 2.0;

    /** Altura da parte de baixo da treliça e do topo da Barge (manual: 157 e 257 cm). */
    public static final double BARGE_TRELICA_ALTURA = 1.57;
    public static final double BARGE_TOPO_ALTURA = 2.57;

    /**
     * CAGES: 6, com 19 cm de largura, a 1,05 m, 2,14 m e 3,24 m do meio do
     * campo para cada lado (manual, seção 5.4 e 5.4.1). Os vãos entre elas
     * têm 0,90 m: o robô (0,85 m) passa de frente, bem justo.
     */
    public static final double CAGE_LARGURA = 0.19;
    public static final double[] CAGES_Y = {
            LARGURA / 2.0 - 3.24, LARGURA / 2.0 - 2.14, LARGURA / 2.0 - 1.05,
            LARGURA / 2.0 + 1.05, LARGURA / 2.0 + 2.14, LARGURA / 2.0 + 3.24,
    };

    /** Suporte vertical no centro do campo (medido na imagem oficial do campo). */
    public static final double SUPORTE_X_MIN = 8.46;
    public static final double SUPORTE_X_MAX = 9.07;
    public static final double SUPORTE_Y_MIN = 3.87;
    public static final double SUPORTE_Y_MAX = 4.20;

    /**
     * A Barge inteira bloqueia o robô, como uma parede?
     * false (como no manual): o robô passa por baixo, entre as Cages.
     * true: cada robô fica do seu lado.
     */
    public static final boolean BARGE_BLOQUEIA_ROBO = false;

    /**
     * NET: a rede pendurada no topo da Barge. O ponto mais baixo fica a
     * 1,93 m do chão (manual, seção 5.4.2).
     */
    public static final double NET_ALTURA = 1.93;

    /**
     * PROCESSOR: abertura na parede lateral, 71 cm de largura por 51 cm de
     * altura, começando 18 cm acima do chão (manual, seção 5.5).
     * O azul fica na parede de baixo (y = 0) e o vermelho na de cima
     * (y = LARGURA). Centros pelas AprilTags 16 (azul) e 3 (vermelho).
     */
    public static final double PROCESSOR_AZUL_X = 6.058;
    public static final double PROCESSOR_VERMELHO_X = 11.491;
    public static final double PROCESSOR_LARGURA = 0.71;
    public static final double PROCESSOR_ALTURA_MIN = 0.18;
    public static final double PROCESSOR_ALTURA_MAX = 0.69;

    /** Pontos de cada peça no teleoperado (manual, tabela 6-2). */
    public static final int PONTOS_NET = 4;
    public static final int PONTOS_PROCESSOR = 6;

    private Campo() {
    }
}
