package ParticleSwarmOptimizer;

/**
 * Wrapper class for the mse during the
 * PSO simulation. Used for communication
 * between the anonymous thread running the
 * car simulation in the particle's <code>eval()</code>
 * method and the class itself.
 *
 * @author Nicolas Dutly
 * @version 1.0
 */
public class AbsErr {

    /**
     * the MSE
     */
    private double absErr;

    /**
     * Create a new wrapper for the MSE
     * @param absErr the MSE
     */
    public AbsErr(double absErr) {
        this.absErr = absErr;
    }

    /**
     * Return the MSE
     *
     * @return the MSE
     */
    double getAbsErr() {
        return absErr;
    }

    /**
     * Set the MSE
     *
     * @param absErr the new MSE
     */
    void setAbsErr(double absErr) {
        this.absErr = absErr;
    }
}
