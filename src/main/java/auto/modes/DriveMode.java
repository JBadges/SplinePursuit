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
                SkidRobot.getInstance().setPosition(new Point(0,0,Math.PI/2));
            }
        });
        runAction(new SplineDrivePath(SkidRobot.getInstance(), 60, 85,100, 0.25,
                new Path(new Point(0, 0, Math.PI/2),
                        new Point(0, 3, Math.PI/2),
                        new Point(1, 4, 0),
                        new Point(6, 4, 0),
                        new Point(7,5, Math.PI/2),
                        new Point(6, 6, -Math.PI)).getPath()));
    }
}
