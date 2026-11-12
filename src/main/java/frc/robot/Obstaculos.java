package frc.robot;

import edu.wpi.first.math.MathUtil;

/**
 * Colisão com os obstáculos do meio do campo (Reefs, suporte da Barge e Cages).
 *
 * Serve tanto para o robô quanto para as Fuels. O objeto é descrito por:
 * - o centro (x, y);
 * - metadeX e metadeY: quanto ele ocupa a partir do centro (retângulo);
 * - raio: para objetos redondos (a Fuel usa metade 0 e raio 0,15).
 *
 * Se o objeto entrou em um obstáculo, ele é empurrado para fora pelo
 * caminho mais curto.
 */
public final class Obstaculos {

    /** Resultado de uma batida: a nova posição e para onde o objeto foi empurrado. */
    public static final class Batida {
        public final double x;
        public final double y;
        public final double normalX;
        public final double normalY;

        Batida(double x, double y, double normalX, double normalY) {
            this.x = x;
            this.y = y;
            this.normalX = normalX;
            this.normalY = normalY;
        }
    }

    private Obstaculos() {
    }

    /**
     * Confere os obstáculos que chegam até o chão: Reefs, suporte central
     * da Barge e Cages. Por baixo da treliça da Barge dá para passar.
     *
     * @return a batida, ou null se o objeto não encostou em nenhum obstáculo
     */
    public static Batida empurrarParaFora(double x, double y, double metadeX, double metadeY, double raio) {
        return empurrarParaFora(x, y, metadeX, metadeY, raio, false);
    }

    /**
     * Confere os obstáculos.
     *
     * @param bargeInteira true para tratar a Barge inteira como parede
     * @return a batida, ou null se o objeto não encostou em nenhum obstáculo
     */
    public static Batida empurrarParaFora(
            double x, double y, double metadeX, double metadeY, double raio, boolean bargeInteira) {
        Batida ultima = null;
        Batida batida;

        // Reefs (círculos).
        for (double[] reef : Campo.REEFS) {
            batida = contraCirculo(x, y, metadeX, metadeY, raio, reef[0], reef[1], Campo.REEF_RAIO);
            if (batida != null) {
                ultima = batida;
                x = batida.x;
                y = batida.y;
            }
        }

        if (bargeInteira) {
            // A Barge vai de uma parede à outra; por isso o Y passa um pouco das paredes.
            batida = contraRetangulo(x, y, metadeX, metadeY, raio,
                    Campo.BARGE_X_MIN, Campo.BARGE_X_MAX, -1.0, Campo.LARGURA + 1.0);
            return batida != null ? batida : ultima;
        }

        // Suporte central da Barge.
        batida = contraRetangulo(x, y, metadeX, metadeY, raio,
                Campo.SUPORTE_X_MIN, Campo.SUPORTE_X_MAX, Campo.SUPORTE_Y_MIN, Campo.SUPORTE_Y_MAX);
        if (batida != null) {
            ultima = batida;
            x = batida.x;
            y = batida.y;
        }

        // As 6 Cages (quadradinhos de 19 cm).
        double meiaCage = Campo.CAGE_LARGURA / 2.0;
        for (double cageY : Campo.CAGES_Y) {
            batida = contraRetangulo(x, y, metadeX, metadeY, raio,
                    Campo.BARGE_X_CENTRO - meiaCage, Campo.BARGE_X_CENTRO + meiaCage,
                    cageY - meiaCage, cageY + meiaCage);
            if (batida != null) {
                ultima = batida;
                x = batida.x;
                y = batida.y;
            }
        }

        return ultima;
    }

    /** Objeto contra um obstáculo redondo (centro cx, cy). */
    private static Batida contraCirculo(
            double x, double y, double metadeX, double metadeY, double raio,
            double cx, double cy, double raioObstaculo) {

        // 1. Ponto do objeto mais perto do centro do obstáculo.
        double pertoX = MathUtil.clamp(cx, x - metadeX, x + metadeX);
        double pertoY = MathUtil.clamp(cy, y - metadeY, y + metadeY);

        double dx = pertoX - cx;
        double dy = pertoY - cy;
        double distancia = Math.hypot(dx, dy);

        // 2. Longe o bastante? Então não encostou.
        double limite = raioObstaculo + raio;
        if (distancia >= limite) {
            return null;
        }

        // 3. Caso raro: o centro do obstáculo ficou dentro do objeto.
        //    Usa a direção entre os centros.
        if (distancia < 0.000001) {
            dx = x - cx;
            dy = y - cy;
            distancia = Math.hypot(dx, dy);
            if (distancia < 0.000001) {
                dx = 1.0;
                dy = 0.0;
                distancia = 1.0;
            }
        }

        // 4. Empurra para fora, na direção que sai do centro do obstáculo.
        double normalX = dx / distancia;
        double normalY = dy / distancia;
        double quantoEntrou = limite - distancia;

        return new Batida(x + normalX * quantoEntrou, y + normalY * quantoEntrou, normalX, normalY);
    }

    /** Objeto contra um obstáculo retangular (xMin a xMax, yMin a yMax). */
    private static Batida contraRetangulo(
            double x, double y, double metadeX, double metadeY, double raio,
            double xMin, double xMax, double yMin, double yMax) {

        // Aumenta o retângulo pelo tamanho do objeto. Assim basta olhar o centro.
        double esquerda = xMin - metadeX - raio;
        double direita = xMax + metadeX + raio;
        double baixo = yMin - metadeY - raio;
        double cima = yMax + metadeY + raio;

        if (x <= esquerda || x >= direita || y <= baixo || y >= cima) {
            return null; // está fora
        }

        // Está dentro: sai pelo lado mais perto.
        double paraEsquerda = x - esquerda;
        double paraDireita = direita - x;
        double paraBaixo = y - baixo;
        double paraCima = cima - y;
        double menor = Math.min(Math.min(paraEsquerda, paraDireita), Math.min(paraBaixo, paraCima));

        if (menor == paraEsquerda) {
            return new Batida(esquerda, y, -1.0, 0.0);
        }
        if (menor == paraDireita) {
            return new Batida(direita, y, 1.0, 0.0);
        }
        if (menor == paraBaixo) {
            return new Batida(x, baixo, 0.0, -1.0);
        }
        return new Batida(x, cima, 0.0, 1.0);
    }
}
