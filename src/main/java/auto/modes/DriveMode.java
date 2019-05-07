package auto.modes;

import auto.AutoBase;
import auto.AutoModeEndedException;
import auto.actions.RunOnceAction;
import auto.actions.SplineDrivePath;
import util.Point;
import util.simulation.SkidRobot;
import util.spline.Path;

/**
 * Created by jacks on 2019-05-05.
 */
public class DriveMode extends AutoBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new RunOnceAction() {
            @Override
            public void runOnce() {
                SkidRobot.getInstance().setPosition(new Point(0, 0, -Math.PI/2));
            }
        });
        runAction(new SplineDrivePath(SkidRobot.getInstance(), -1, 1,25, 25,70, 0.20,
                new Path(new Point(0, 0, Math.PI/2),
                        new Point(4, 7, 0),
                        new Point(8, 8, -Math.PI/2),
                        new Point(2, 5, Math.PI),
                        new Point(7,3.25, 0))));
    }
}
