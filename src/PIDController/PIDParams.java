package PIDController;

/**
 * Represents a PID parameter configuration
 */
public class PIDParams {
    private double KP;
    private double KD;
    private double KI;

    public PIDParams(double KP, double KD, double KI) {
        this.KP = KP;
        this.KD = KD;
        this.KI = KI;
    }

    public double getKP() {
        return KP;
    }

    public void setKP(double KP) {
        this.KP = KP;
    }

    public double getKD() {
        return KD;
    }

    public double[] getParams(){
        return new double[]{getKP(), getKD(), getKI()};
    }

    public void setParams(double[] params){
        this.KP = params[0];
        this.KD = params[1];
        this.KI = params[2];
    }

    public void setKD(double KD) {
        this.KD = KD;
    }

    public double getKI() {
        return KI;
    }

    public void setKI(double KI) {
        this.KI = KI;
    }

    @Override
    public String toString() {
        return "KP: " + KP + " KD: " + KD + " KI: " + KI;
    }
}
