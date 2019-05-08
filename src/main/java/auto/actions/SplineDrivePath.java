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
import util.spline.Path;
import util.spline.QuinticHermiteSpline;
import util.spline.Spline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by jacks on 2019-05-04.
 */
public class SplineDrivePath implements Action {

    private double maxLookahead;
    private Path path;
    private double startTime;
    private double lastTime;
    private double[] voltages = {0.0, 0.0};
    private double addedTurnConst;
    private double throttleConstant;
    private double curvatureConstant;
    private double minThrottle;
    private double maxThrottle;
    private boolean isBackwards;

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

    public SplineDrivePath(SkidRobot robot, double minThrottle, double maxThrottle, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, Path path) {
        this.robot = robot;
        this.minThrottle = minThrottle;
        this.maxThrottle = maxThrottle;
        this.throttleConstant = throttleConstant;
        this.curvatureConstant = curvatureConstant;
        this.addedTurnConst = addedTurnConst;
        this.path = path;
        this.maxLookahead = maxLookahead;
    }

    public SplineDrivePath(SkidRobot robot, double minThrottle, double maxThrottle, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, QuinticHermiteSpline... path) {
        this(robot, minThrottle, maxThrottle, throttleConstant, curvatureConstant, addedTurnConst, maxLookahead, new Path(Arrays.asList(path)));
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis()/1000.0;
        lastTime = startTime;
        isBackwards = getErrorPoint(robot.getPosition(), path.getPoint(findClosestPercent(robot.getPosition(), 0))).getX() < 0;
    }

    @Override
    public void update() {
        double curTime = (System.currentTimeMillis()/1000.0 - startTime);
        robot.updatePosNorm(curTime - lastTime, voltages[0], voltages[1]);

        double percentComplete = findClosestPercent(robot.getPosition(), 0);
        Point desiredPoint, errorPoint;
        boolean last = Util.epsilonEquals(robot.getPosition().getX(), path.get(path.size()-1).getPoint2D(1).getX(), 0.1) && Util.epsilonEquals(robot.getPosition().getY(), path.get(path.size()-1).getPoint2D(1).getY(), 0.1);
        desiredPoint = path.getPoint(percentComplete);
        errorPoint = getErrorPoint(robot.getPosition(), desiredPoint);

        final double normalizedGoalHeading = normalizeAngle(desiredPoint.getHeading());
        final double normalizedRobotHeading = normalizeAngle(robot.getPosition().getHeading() + (isBackwards ?  Math.PI : 0));
        double addedConst = 0;

        if(last) {
            goalCircle.setFill(Color.ORANGE);
            addedConst = addedTurnConst * (angleBetween(normalizedGoalHeading, normalizedRobotHeading));
        }

        addedConst *= isTurnCCW(normalizedRobotHeading, normalizedGoalHeading) ? -1 : 1;
        addedConst *= isBackwards ? -1 : 1;

        double throttle = Util.limit(throttleConstant * errorPoint.getX() / maxLookahead, minThrottle, maxThrottle);

        voltages[0] = throttle - curvatureConstant * errorPoint.getY() / maxLookahead - addedConst;
        voltages[1] = throttle + curvatureConstant * errorPoint.getY() / maxLookahead + addedConst;

        if(isBackwards) {
            double temp = voltages[0];
            voltages[0] = voltages[1];
            voltages[1] = temp;
            voltages[0] *= 1;
            voltages[1] *= 1;
        }

        if(Math.abs(voltages[0]) > 1 || Math.abs(voltages[1]) > 1) {
            if(Math.abs(voltages[0]) > Math.abs(voltages[1])) {
                voltages[0] /= Math.abs(voltages[0]);
                voltages[1] /= Math.abs(voltages[0]);
            } else {
                voltages[0] /= Math.abs(voltages[1]);
                voltages[1] /= Math.abs(voltages[1]);
            }
            voltages[0] *= 1;
            voltages[1] *= 1;
        }

        Point robotPos = robot.getPosition();
        Logger.write("path", curTime + ", " + desiredPoint.getX() + ", " + desiredPoint.getY() + ", " + desiredPoint.getHeading()
        + ", " + robotPos.getX() + ", " + robotPos.getY() + ", " + robotPos.getHeading() + ", " + voltages[0] + ", " + voltages[1]);

        Platform.runLater(new Runnable() {
            @Override
            public void run() {
                data.setText((isBackwards ? "R " : "F ") + "Time: " + String.format("%.2f", curTime) + " X: " + String.format("%.4f", errorPoint.getX()) + " Y: " + String.format("%.4f", errorPoint.getY()) + " RH: " + String.format("%.4f", Math.toDegrees(normalizedRobotHeading)) + " GH: " + String.format("%.4f",Math.toDegrees(normalizedGoalHeading)) +" H: " + String.format("%.4f", Math.toDegrees(angleBetween(normalizedGoalHeading, normalizedRobotHeading))));
                goalCircle.setCenterX(desiredPoint.getX() * 50);
                goalCircle.setCenterY(600 - desiredPoint.getY() * 50);
                Point guessedPos = path.getPoint(guessedPositionT);
                robotGuessedCircle.setCenterX(guessedPos.getX() * 50);
                robotGuessedCircle.setCenterY(600 - guessedPos.getY() * 50);
                robotCircle.setCenterX(robot.getPosition().getX() * 50);
                robotCircle.setCenterY(600 - robot.getPosition().getY() * 50);
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

    private double angleBetween(double a, double b) {
        double diff = (a - b + Math.PI*2*10) % (Math.PI*2);
        return diff <= Math.PI ? diff : Math.PI*2 - diff;
    }

    private boolean isTurnCCW(double curRad, double wantedRad) {
        double diff = wantedRad - curRad;        // CCW = counter-clockwise ie. left
        return diff > 0 ? diff > Math.PI : diff >= -Math.PI;
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

    private double findClosestPercent(Point position) {
        double distance = position.distance(path.get(0).getPoint(0));
        double smallestT = 0;
        for(double t = 0; t < path.size();) {
            double newDist = position.distance(path.getPoint(t));
            if(newDist < distance) {
                distance = newDist;
                smallestT = t;
            }
            t += 1 / (path.get((int) t).getPathLength() * 50);
        }
        return smallestT;
    }

    private double findClosestPercent(Point position, double curT) {
        curT = guessedPositionT = findClosestPercent(position);
        double distance = 0;
        double dT = 0;
        for(double t = curT; t < path.size();) {
            distance += path.getPoint(t).distance(path.getPoint(t+dT));
            if(distance > maxLookahead) {
                return t;
            }
            dT = 1 / (path.get((int) t).getPathLength() * 50);
            t += dT;
        }
        return path.size();
    }

    @Override
    public boolean isFinished() {
        return  (Util.epsilonEquals(robot.getPosition().getX(), path.get(path.size()-1).getPoint2D(1).getX(), 0.075) && Util.epsilonEquals(robot.getPosition().getY(), path.get(path.size()-1).getPoint2D(1).getY(), 0.075) && Util.epsilonEquals(normalizeAngle(robot.getPosition().getHeading() + (isBackwards ? Math.PI : 0)), normalizeAngle(path.get(path.size()-1).getHeading(1)), 3 * Math.PI/180));
    }

    @Override
    public void done() {
        System.out.println("done path");
    }

}
