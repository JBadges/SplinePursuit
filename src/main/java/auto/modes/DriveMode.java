package auto.modes;

import auto.AutoBase;
import auto.AutoModeEndedException;
import auto.actions.RunOnceAction;
import auto.actions.SplineDrivePath;
import util.Point;
import util.simulation.SkidRobot;


/**
 * Created by jacks on 2019-05-05.
 */
public class DriveMode extends AutoBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new RunOnceAction() {
            @Override
            public void runOnce() {
                SkidRobot.getInstance().setPosition(new Point(0, 0, Math.PI/2));
            }
        });

        runAction(new SplineDrivePath(SkidRobot.getInstance(), -1, 1,1, 12,5, 0.025, 0.01, 2,
                new Point(0, 0, Math.PI/2),
                new Point(0.7,0.8,0),
                new Point(1,0.8,-Math.PI/2),
                new Point(.5,.5,Math.PI/2),
                new Point(.35,.6, Math.PI)));
    }
}
