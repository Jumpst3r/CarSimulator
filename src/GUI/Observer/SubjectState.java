package GUI.Observer;

import PIDController.PIDParams;

public class SubjectState {
    /**
     * Wrapper class for a subject's state
     */
    private double currentSpeed;
    private double currentError;
    private double setPoint;
    private PIDParams currentPIDConfig;

    /**
     * Create a new subject state
     * @param currentSpeed the current speed of the subject
     * @param currentError the current error of the subject
     * @param sp the setpoint of the subject
     * @param currentPIDConfig the current PID config of the subject
     */
    public SubjectState(double currentSpeed, double currentError, double sp, PIDParams currentPIDConfig) {
        this.currentSpeed = currentSpeed;
        this.currentError = currentError;
        this.setPoint = sp;
        this.currentPIDConfig = currentPIDConfig;
    }

    /**
     * Return the subject's current speed
     * @return
     */
    public double getCurrentSpeed() {
        return currentSpeed;
    }

    /**
     * Get the subject's current error
     * @return the subject's current error
     */
    public double getCurrentError() {
        return currentError;
    }

    /**
     * Get the subject's setpoint
     * @return
     */
    public double getSetPoint() {
        return setPoint;
    }

    /**
     * Get the subject's current PID configuration
     * @return
     */
    public PIDParams getCurrentPIDConfig() {
        return currentPIDConfig;
    }
}
