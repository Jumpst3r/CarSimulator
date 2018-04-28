package PIDController;

import java.text.DecimalFormat;

/**
 * Represents a PID parameter configuration
 */
public class PIDParams {
    private double KP;
    private double KD;
    private double KI;
    private volatile double value;

    public PIDParams(double value) {
        this.value = value;
    }

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

    public double getValue() {
        return value;
    }

    public synchronized void setValue(double value) {
        this.value = value;
    }

    @Override
    public String toString() {
        DecimalFormat format = new DecimalFormat("##.000");

        return "KP: " + format.format(KP) + " KD: " + format.format(KD) + " KI: " + format.format(KI);
    }

    public boolean equals_pid(PIDParams o) {
        return (this.getKP() == o.getKP() && this.getKD() == o.getKD() && this.getKI() == o.getKI());
    }
}
