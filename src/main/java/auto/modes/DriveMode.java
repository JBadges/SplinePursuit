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
                SkidRobot.getInstance().setPosition(new Point(0, 0, Math.PI/2));
            }
        });
        runAction(new SplineDrivePath(SkidRobot.getInstance(), -12, 12,25, 125,70, 0.35,
                new Path(new Point(0, 0, Math.PI/2),
                        new Point(0, 4, Math.PI/2),
                        new Point(1, 5, 0),
                        new Point(6, 5, 0),
                        new Point(7,6, Math.PI/2),
                        new Point(6, 7, Math.PI))));
    }
}
