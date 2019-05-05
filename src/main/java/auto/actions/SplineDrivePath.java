package auto.actions;

import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import util.Logger;
import util.Point;
import util.Util;
import util.simulation.SkidRobot;
import util.spline.Spline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by jacks on 2019-05-04.
 */
public class SplineDrivePath implements Action {

    private double maxLookahead;
    private List<Spline> path;
    private double startTime;
    private double lastTime;
    private double[] voltages = {0.0, 0.0};
    private double driveConst;
    private double turnConst;
    private double addedTurnConst;
    private double lastT = 0;

    //TESTING
    private SkidRobot robot;
    public static Circle goalCircle = new Circle(0,0,10);
    public static Circle robotGuessedCircle = new Circle(0,0,10);
    public static Circle robotCircle = new Circle(0,0,10);
    public static Line headingLine = new Line();
    private double guessedPositionT = 0;
    public static List<Circle> robotsPath = new ArrayList<>();
    public static Label data = new Label("X: 0 Y: 0 H: 0");
    public static Pane pane;

    public SplineDrivePath(SkidRobot robot, double driveConst, double turnConst, double addedTurnConst, double maxLookahead, List<Spline> path) {
        this.robot = robot;
        this.driveConst = driveConst;
        this.turnConst = turnConst;
        this.addedTurnConst = addedTurnConst;
        this.path = path;
        this.maxLookahead = maxLookahead;
    }

    public SplineDrivePath(SkidRobot robot, double driveConst, double turnConst, double addedTurnConst, double maxLookahead, Spline... path) {
        this(robot, driveConst, turnConst, addedTurnConst, maxLookahead, Arrays.asList(path));
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis()/1000.0;
        lastTime = startTime;
    }

    @Override
    public void update() {
        double curTime = (System.currentTimeMillis()/1000.0 - startTime);
        robot.updatePos(curTime - lastTime, voltages[0], voltages[1]);

        //Works but if turns are too sharp lags SUPER far behind
        //double percentComplete = curTime / maxLookahead * path.size();
        double percentComplete = findCloestestPercent(robot.getPosition(), lastT);
        Point desiredPoint, errorPoint;
        boolean last = Util.epsilonEquals(robot.getPosition().getX(), path.get(path.size()-1).getPoint2D(1).getX(), 0.08) && Util.epsilonEquals(robot.getPosition().getY(), path.get(path.size()-1).getPoint2D(1).getY(), 0.08);
        if(percentComplete >= path.size()) {
            desiredPoint = path.get(path.size()-1).getPoint(1);
        } else {
            desiredPoint = path.get((int) percentComplete).getPoint(percentComplete % 1);
        }
        errorPoint = getErrorPoint(robot.getPosition(), desiredPoint);
        lastT = percentComplete;

        final double normalizedGoalHeading = normalizeAngle(desiredPoint.getHeading());
        final double normalizedRobotHeading = normalizeAngle(robot.getPosition().getHeading());
        double addedConst = 0;
        if(last){
            goalCircle.setFill(Color.ORANGE);
            addedConst = addedTurnConst * (normalizedGoalHeading - normalizedRobotHeading);
        }
        voltages[0] = driveConst * errorPoint.getX() - turnConst * errorPoint.getY() - addedConst;
        voltages[1] = driveConst * errorPoint.getX() + turnConst * errorPoint.getY() + addedConst;

        if(Math.abs(voltages[0]) > 12 || Math.abs(voltages[1]) > 12) {
            if(Math.abs(voltages[0]) > Math.abs(voltages[1])) {
                voltages[0] /= Math.abs(voltages[0]);
                voltages[1] /= Math.abs(voltages[0]);
            } else {
                voltages[0] /= Math.abs(voltages[1]);
                voltages[1] /= Math.abs(voltages[1]);
            }
            voltages[0] *= 12;
            voltages[1] *= 12;
        }


        Point robotPos = robot.getPosition();
        Logger.write("path", curTime + ", " + desiredPoint.getX() + ", " + desiredPoint.getY() + ", " + desiredPoint.getHeading()
        + ", " + robotPos.getX() + ", " + robotPos.getY() + ", " + robotPos.getHeading() + ", " + voltages[0] + ", " + voltages[1]);

        Platform.runLater(new Runnable() {
            @Override
            public void run() {
                data.setText("Time: " + String.format("%.2f",curTime) + " X: " + errorPoint.getX() + " Y: " + errorPoint.getY() + " RH: " + Math.toDegrees(normalizedRobotHeading) + " GH: " + Math.toDegrees(normalizedGoalHeading) +" H: " + Math.toDegrees(normalizedGoalHeading - normalizedRobotHeading));
                goalCircle.setCenterX(desiredPoint.getX() * 100);
                goalCircle.setCenterY(1000 - desiredPoint.getY() * 100);
                Point guessedPos = path.get((int) guessedPositionT).getPoint(guessedPositionT % 1);
                robotGuessedCircle.setCenterX(guessedPos.getX() * 100);
                robotGuessedCircle.setCenterY(1000 - guessedPos.getY() * 100);
                robotCircle.setCenterX(robot.getPosition().getX() * 100);
                robotCircle.setCenterY(1000 - robot.getPosition().getY() * 100);
                headingLine.setStartX(robotCircle.getCenterX());
                headingLine.setStartY(robotCircle.getCenterY());
                headingLine.setEndX(robotCircle.getCenterX() + robotCircle.getRadius() * Math.cos(robot.getPosition().getHeading()));
                headingLine.setEndY(robotCircle.getCenterY() - robotCircle.getRadius() * Math.sin(robot.getPosition().getHeading()));
                Circle lastPos = new Circle(robotCircle.getCenterX(), robotCircle.getCenterY(), 3);
                lastPos.setFill(Color.PURPLE);
                robotsPath.add(lastPos);
                pane.getChildren().removeAll(SplineDrivePath.robotsPath);
                pane.getChildren().addAll(SplineDrivePath.robotsPath);
            }
        });

        lastTime = (System.currentTimeMillis()/1000.0 - startTime);
    }

    private double normalizeAngle(double rad) {
        while(rad > 2*Math.PI) {
            rad -= Math.PI * 2;
        }
        while(rad < 0) {
            rad += Math.PI * 2;
        }
        return rad;
    }

    private Point getErrorPoint(Point robot, Point goal) {
        Point error = goal.copy();
        error.substractXY(robot);
        error.rotateAroundOrigin(-robot.getHeading());
        return error;
    }

    private double findCloestestPercent(Point position) {
        double distance = position.distance(path.get(0).getPoint(0));
        double smallestT = 0;
        for(double t = 0; t < path.size(); t += 0.01) {
            double newDist = position.distance(path.get((int) t).getPoint(t % 1));
            if(newDist < distance) {
                distance = newDist;
                smallestT = t;
            }
        }
        return smallestT;
    }

    private double findCloestestPercent(Point position, double curT) {
        curT = guessedPositionT = findCloestestPercent(position);
        position = path.get((int) curT).getPoint(curT % 1);
        for(double t = curT; t < path.size(); t += 0.01) {
            double newDist = position.distance(path.get((int) t).getPoint(t % 1));
            if(newDist > maxLookahead) {
                return t;
            }
        }
        return path.size();
    }

    @Override
    public boolean isFinished() {
        return  (Util.epsilonEquals(robot.getPosition().getX(), path.get(path.size()-1).getPoint2D(1).getX(), 0.08) && Util.epsilonEquals(robot.getPosition().getY(), path.get(path.size()-1).getPoint2D(1).getY(), 0.08) && Util.epsilonEquals(normalizeAngle(robot.getPosition().getHeading()), normalizeAngle(path.get(path.size()-1).getHeading(1)), 3 * Math.PI/180));
    }

    @Override
    public void done() {
        System.out.println("done path");
    }

}
