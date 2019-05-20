package auto;

import auto.actions.Action;

/**
 * AutoBase
 */
public abstract class AutoBase {
    protected double updateRate = 1.0 / 100.0;
    private double lastUpdate = System.currentTimeMillis();
    private double dT;
    protected boolean active = false;

    protected abstract void routine() throws AutoModeEndedException;

    public void run() {
        active = true;

        try {
            lastUpdate = System.currentTimeMillis();
            routine();
        } catch (AutoModeEndedException e) {
            return;
        }

        done();
    }

    public void done() {

    }

    public void stop() {
        active = false;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }

        return isActive();
    }

    public void runAction(Action action) throws AutoModeEndedException {
        action.start();

        while (isActive() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (updateRate * 1000.0);
            dT = System.currentTimeMillis() - lastUpdate;
            lastUpdate = System.currentTimeMillis();
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        action.done();
    }

    public double getdT() {
        return dT;
    }

}