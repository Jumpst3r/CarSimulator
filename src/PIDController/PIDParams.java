package PIDController;

import java.text.DecimalFormat;

/**
 * Wrapper class for the PID parameters
 * @author Nicolas Dutly
 * @version 1.0
 */
public class PIDParams {
    /**
     * The proportional gain
     */
    private double KP;
    /**
     * The derivative gain
     */
    private double KD;
    /**
     * The integral gain
     */
    private double KI;
    /**
     * The error value associated with the
     * PID configuration (only used by the PSO
     * algorithm)
     */
    private volatile double value;

    /**
     * Create a new PID empty parameter set
     * with a given score (the error that
     * the specified values produced during
     * training)
     *
     * @param value the MSE obtained during the simulation
     *              of the given values
     * @see ParticleSwarmOptimizer.ParticleSwarmOptimizer
     */
    public PIDParams(double value) {
        this.value = value;
    }

    /**
     * Create a new PID parameter set
     * @param KP the proportional gain constant
     * @param KD the derivative gain constant
     * @param KI the integral gain constant
     */
    public PIDParams(double KP, double KD, double KI) {
        this.KP = KP;
        this.KD = KD;
        this.KI = KI;
    }

    /**
     * Returns the proportional gain constant
     * @return the proportional gain constant
     */
    public double getKP() {
        return KP;
    }

    /**
     * Set the proportional gain constant
     * @param KP the new constant
     */
    public void setKP(double KP) {
        this.KP = KP;
    }

    /**
     * Returns the derivative gain constant
     * @return the derivative gain constant
     */
    public double getKD() {
        return KD;
    }

    /**
     * Set the derivative gain constant
     *
     * @param KD the new derivative gain constant
     */
    public void setKD(double KD) {
        this.KD = KD;
    }

    /**
     * Returns an array of doubles containing the PID
     * parameters
     * @return the PID parameters
     */
    public double[] getParams(){
        return new double[]{getKP(), getKD(), getKI()};
    }

    /**
     * Set new PID parameters from an array of new values
     * @param params the new PID parameters
     */
    public void setParams(double[] params){
        this.KP = params[0];
        this.KD = params[1];
        this.KI = params[2];
    }

    /**
     * Get the integral gain constant
     * @return the integral gain constant
     */
    public double getKI() {
        return KI;
    }

    /**
     * Set the integral gain constant
     * @param KI the new integral gain constant
     */
    public void setKI(double KI) {
        this.KI = KI;
    }

    /**
     * Return the MSE associated with the current
     * parameters
     * @return the MSE associated with the current
     *         parameters
     */
    public double getValue() {
        return value;
    }

    /**
     * Set the MSE associated with the current parameters
     * @param value the new MSE value associated with the current parameters
     */
    public synchronized void setValue(double value) {
        this.value = value;
    }

    @Override
    public String toString() {
        DecimalFormat format = new DecimalFormat("##.000");

        return "KP: " + format.format(KP) + " KD: " + format.format(KD) + " KI: " + format.format(KI);
    }

    /**
     * Used to compare two PID configurations
     * @param o the other PID configuration to compare to
     * @return true if the parameters are equal
     */
    public boolean equals_pid(PIDParams o) {
        return (this.getKP() == o.getKP() && this.getKD() == o.getKD() && this.getKI() == o.getKI());
    }
}
