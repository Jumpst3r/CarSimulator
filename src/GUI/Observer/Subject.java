package GUI.Observer;

import java.util.ArrayList;
import java.util.List;

/**
 * Represent a subject
 */
public abstract class Subject {
    private List<Observer> observerList = new ArrayList<Observer>();

    /**
     * Register a new observer
     * @param observer new observer
     */
    public void register(Observer observer) {
        if (!observerList.contains(observer)) {
            observerList.add(observer);
        }
    }

    /**
     * Remove an observer
     * @param observer observer
     */
    public void detach(Observer observer) {
        observerList.remove(observer);
    }

    /**
     * notify all observers
     */
    public void notify_observers(){
        for (Observer obs : observerList) {
            obs.update();
        }
    }

    /**
     * Get the subject state
     * @return the subject's state
     */
    abstract public SubjectState getState();
}
