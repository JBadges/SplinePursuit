package auto.actions;

import javafx.application.Platform;
import javafx.scene.control.Label;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import main.Main;
import util.Logger;
import util.Point;
import util.Util;
import util.simulation.SkidRobot;
import util.spline.Path;

import java.util.ArrayList;
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
    private double distEpsilon;
    private double angleEpsilon;

    //TESTING
    private SkidRobot robot;
    public static Circle goalCircle = new Circle(0,0,10);
    public static Circle robotGuessedCircle = new Circle(0,0,10);
    public static Circle robotCircle = new Circle(0,0,10);
    public static Line headingLine = new Line();
    private double guessedPositionT = 0;
    private double lastLookaheadT = 0;
    public static List<Circle> robotsPath = new ArrayList<>();
    public static Label data = new Label("X: 0 Y: 0 H: 0");
    public static Pane pane;
    private double ongoingScore = 0;
    private Point predictedPoint;

    /**
     *
     * @param robot - Simulation bot
     * @param minThrottle - min throttle [-1,1]
     * @param maxThrottle - max throttle [-1,1]
     * @param throttleConstant - throttle proportionality constant
     * @param curvatureConstant - turning proportionality constant
     * @param addedTurnConst - final heading correction proportionality constants
     * @param maxLookahead - distance to find goal point
     * @param distEpsilon - max distance error to finish path
     * @param angleEpsilon - max angle error (deg) to finish path
     * @param path - the path to follow
     */
    public SplineDrivePath(SkidRobot robot, double minThrottle, double maxThrottle, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, double distEpsilon, double angleEpsilon, Path path) {
        this.robot = robot;
        this.minThrottle = minThrottle;
        this.maxThrottle = maxThrottle;
        this.throttleConstant = throttleConstant;
        this.curvatureConstant = curvatureConstant;
        this.addedTurnConst = addedTurnConst;
        this.path = path;
        this.maxLookahead = maxLookahead;
        this.distEpsilon = distEpsilon;
        this.angleEpsilon = angleEpsilon;
    }
    /**
     *
     * @param robot - Simulation bot
     * @param minThrottle - min throttle [-1,1]
     * @param maxThrottle - max throttle [-1,1]
     * @param throttleConstant - throttle proportionality constant
     * @param curvatureConstant - turning proportionality constant
     * @param addedTurnConst - final heading correction proportionality constants
     * @param maxLookahead - distance to find goal point
     * @param distEpsilon - max distance error to finish path
     * @param angleEpsilon - max angle error (deg) to finish path
     * @param path - the path to follow
     */
    public SplineDrivePath(SkidRobot robot, double minThrottle, double maxThrottle, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, double distEpsilon, double angleEpsilon, Point... path) {
        this(robot, minThrottle, maxThrottle, throttleConstant, curvatureConstant, addedTurnConst, maxLookahead, distEpsilon, angleEpsilon, new Path(path));
    }

    @Override
    public void start() {
        ongoingScore = 0;
        startTime = System.currentTimeMillis()/1000.0;
        lastTime = startTime;
        isBackwards = getErrorPoint(robot.getPosition(), path.getPoint(lookaheadPoint(robot.getPosition()))).getX() < 0;
        goalCircle.setFill(Color.RED);
        Logger.write("path", "Time, Goal X, Goal Y, Goal Heading, Predicted X, Predicted Y, Predicted Heading, Robot X, Robot Y, Robot Heading, Left Voltage, Right Voltage, Throttle, Curve, Added Curve");
    }

    @Override
    public void update() {
        double curTime = (System.currentTimeMillis()/1000.0 - startTime);
        robot.updatePosNorm(curTime - lastTime, voltages[0], voltages[1]);
        double percentComplete = lookaheadPoint(robot.getPosition());
        Point desiredPoint, errorPoint;
        boolean last = Util.epsilonEquals(robot.getPosition().getX(), path.getPoint(path.size()).getX(), distEpsilon) && Util.epsilonEquals(robot.getPosition().getY(), path.getPoint(path.size()).getY(), distEpsilon);
        desiredPoint = path.getPoint(percentComplete);
        errorPoint = getErrorPoint(robot.getPosition(), desiredPoint);

        final double normalizedGoalHeading = Util.normalizeAngle(desiredPoint.getHeading());
        final double normalizedRobotHeading = Util.normalizeAngle(robot.getPosition().getHeading() + (isBackwards ?  Math.PI : 0));

        double curve = curvatureConstant * errorPoint.getY() / maxLookahead;
        double throttle = Util.limit(throttleConstant * errorPoint.getX() / maxLookahead, minThrottle, maxThrottle);
        double addedConst = addedTurnConst * Util.angleBetween(normalizedGoalHeading, normalizedRobotHeading);
        addedConst *= Util.isTurnCCW(normalizedRobotHeading, normalizedGoalHeading) ? -1 : 1;
        addedConst *= isBackwards ? -1 : 1;

        if(last) {
            goalCircle.setFill(Color.ORANGE);
        }

        voltages[0] = throttle - (last ? addedConst : curve);
        voltages[1] = throttle + (last ? addedConst : curve);

        // When going backwards the left and right drives are essentially swapped
        // we dont need to invert voltage because the x error will be negative
        if(isBackwards) {
            double temp = voltages[0];
            voltages[0] = voltages[1];
            voltages[1] = temp;
        }

        voltages = Util.absLimitWithRatio(voltages, 1);

        Point robotPos = robot.getPosition();

        double accuracyScore = robotPos.distance(path.getPoint(guessedPositionT)) +
                Math.min(
                        Util.angleBetween(Util.normalizeAngle(robotPos.getHeading()), Util.normalizeAngle(path.getHeading(guessedPositionT))),
                        Util.angleBetween(Util.normalizeAngle(robotPos.getHeading()), Util.normalizeAngle(path.getHeading(guessedPositionT) + Math.PI))) / Math.PI;

        ongoingScore += accuracyScore;
                Logger.write("path", curTime + ", " + desiredPoint.getX() + ", " + desiredPoint.getY() + ", " + desiredPoint.getHeading()
        + ", " + predictedPoint.getX() + ", " + predictedPoint.getY() + ", " + predictedPoint.getHeading() + ", " + robotPos.getX() + ", " + robotPos.getY() + ", " + robotPos.getHeading() + ", " + voltages[0] + ", " + voltages[1] + ", " + throttle + ", " + curve + ", " + addedConst);

        Platform.runLater(new Runnable() {
            @Override
            public void run() {
                Main.throttle.setAngle( (throttle-0)/(2-0) * 360);
                Main.throttle.setName(String.format("%.4f", throttle));
                Main.curve.setAngle( (curve - (-10))/(10-(-10)) * 360);
                Main.curve.setName(String.format("%.4f", curve));
                data.setText("dT: " + Main.auto.getAutoMode().getdT() + (isBackwards ? "R " : "F ") + "Time: " + String.format("%.2f", curTime) + " X: " + String.format("%.4f", errorPoint.getX()) + " Y: " + String.format("%.4f", errorPoint.getY()) + " RH: " + String.format("%.4f", Math.toDegrees(normalizedRobotHeading)) + " GH: " + String.format("%.4f",Math.toDegrees(normalizedGoalHeading)) +" H: " + String.format("%.4f", Math.toDegrees(Util.angleBetween(normalizedGoalHeading, normalizedRobotHeading))) + " Score: " + String.format("%.2f", ongoingScore));
                goalCircle.setCenterX(desiredPoint.getX() * 600);
                goalCircle.setCenterY(600 - desiredPoint.getY() * 600);
                Point guessedPos = path.getPoint(guessedPositionT);
                robotGuessedCircle.setCenterX(guessedPos.getX() * 600);
                robotGuessedCircle.setCenterY(600 - guessedPos.getY() * 600);
                robotCircle.setCenterX(robot.getPosition().getX() * 600);
                robotCircle.setCenterY(600 - robot.getPosition().getY() * 600);
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

    private Point getErrorPoint(Point robot, Point goal) {
        Point error = goal.copy();
        error.substractXY(robot);
        error.rotateAroundOrigin(-robot.getHeading());
        return error;
    }

    private double getClosestPercent(Point position) {
        double distance = position.distance(path.getPoint(guessedPositionT));
        double smallestT = guessedPositionT;
        for(double t = guessedPositionT; t < path.size();) {
            double newDist = position.distance(path.getPoint(t));
            if(newDist < distance) {
                distance = newDist;
                smallestT = t;
            }
            t += 1 / (path.get((int) t).getPathLength() * 1e3);
        }
        return smallestT;
    }

    private double lookaheadPoint(Point position) {
        guessedPositionT = getClosestPercent(position);
        predictedPoint = path.getPoint(guessedPositionT);
        double distance;
        double dT;
        for(double t = lastLookaheadT; t < path.size(); t += dT) {
            distance = path.getDistanceBetween(guessedPositionT, t);
            if(distance > lookaheadDistance()) {
                lastLookaheadT = t;
                return t;
            }
            dT = 1 / (path.get((int) t).getPathLength() * 1e3);
        }
        lastLookaheadT = path.size();
        return path.size();
    }

    private double lookaheadDistance() {
        double look = maxLookahead;
        return Math.abs(look);
    }

    @Override
    public boolean isFinished() {
        Point backwardsPoint = path.getPoint(path.size());
        backwardsPoint.setHeading(backwardsPoint.getHeading() + Math.PI);
        return robot.getPosition().equals(path.getPoint(path.size()), distEpsilon, Math.toRadians(angleEpsilon)) || robot.getPosition().equals(backwardsPoint, distEpsilon, Math.toRadians(angleEpsilon));
    }

    @Override
    public void done() {
        System.out.println("done path");
        goalCircle.setFill(Color.GREEN);
    }

}
