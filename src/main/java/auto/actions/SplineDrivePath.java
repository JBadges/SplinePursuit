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
    private double addedTurnConst;
    private double lastT = 0;
    private double throttleConstant;
    private double curvatureConstant;
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

    public SplineDrivePath(SkidRobot robot, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, List<Spline> path) {
        this.robot = robot;
        this.throttleConstant = throttleConstant;
        this.curvatureConstant = curvatureConstant;
        this.addedTurnConst = addedTurnConst;
        this.path = path;
        this.maxLookahead = maxLookahead;
    }

    public SplineDrivePath(SkidRobot robot, double throttleConstant, double curvatureConstant, double addedTurnConst, double maxLookahead, Spline... path) {
        this(robot, throttleConstant, curvatureConstant, addedTurnConst, maxLookahead, Arrays.asList(path));
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis()/1000.0;
        lastTime = startTime;

        double percentComplete = findClosestPercent(robot.getPosition(), lastT);
        Point desiredPoint;
        if(percentComplete >= path.size()) {
            desiredPoint = path.get(path.size()-1).getPoint(1);
        } else {
            desiredPoint = path.get((int) percentComplete).getPoint(percentComplete % 1);
        }
        isBackwards = getErrorPoint(robot.getPosition(), desiredPoint).getX() < 0;
    }

    @Override
    public void update() {
        double curTime = (System.currentTimeMillis()/1000.0 - startTime);
        robot.updatePos(curTime - lastTime, voltages[0], voltages[1]);

        double percentComplete = findClosestPercent(robot.getPosition(), lastT);
        Point desiredPoint, errorPoint;
        boolean last = Util.epsilonEquals(robot.getPosition().getX(), path.get(path.size()-1).getPoint2D(1).getX(), 0.1) && Util.epsilonEquals(robot.getPosition().getY(), path.get(path.size()-1).getPoint2D(1).getY(), 0.1);
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

        if(last) {
            goalCircle.setFill(Color.ORANGE);
            if (isBackwards) {
                addedConst = -addedTurnConst * (normalizedGoalHeading - normalizeAngle(normalizedRobotHeading + Math.PI));
            } else {
                addedConst = addedTurnConst * (normalizedGoalHeading - normalizedRobotHeading);
            }
        }

        voltages[0] = throttleConstant * errorPoint.getX() - curvatureConstant * errorPoint.getY() - addedConst;
        voltages[1] = throttleConstant * errorPoint.getX() + curvatureConstant * errorPoint.getY() + addedConst;

        if(isBackwards) {
            double temp = voltages[0];
            voltages[0] = voltages[1];
            voltages[1] = temp;
            voltages[0] *= 1;
            voltages[1] *= 1;
        }

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
                data.setText("Time: " + String.format("%.2f", curTime) + " X: " + String.format("%.4f", errorPoint.getX()) + " Y: " + String.format("%.4f", errorPoint.getY()) + " RH: " + String.format("%.4f", Math.toDegrees(normalizeAngle(normalizedRobotHeading + (isBackwards ? Math.PI : 0)))) + " GH: " + String.format("%.4f",Math.toDegrees(normalizedGoalHeading)) +" H: " + String.format("%.4f",Math.toDegrees(normalizedGoalHeading - normalizedRobotHeading)));
                goalCircle.setCenterX(desiredPoint.getX() * 50);
                goalCircle.setCenterY(600 - desiredPoint.getY() * 50);
                Point guessedPos = path.get((int) guessedPositionT).getPoint(guessedPositionT % 1);
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
        for(double t = 0; t < path.size(); t += 0.01) {
            double newDist = position.distance(path.get((int) t).getPoint(t % 1));
            if(newDist < distance) {
                distance = newDist;
                smallestT = t;
            }
        }
        return smallestT;
    }

    private double findClosestPercent(Point position, double curT) {
        curT = guessedPositionT = findClosestPercent(position);
        double distance = 0;
        for(double t = curT; t < path.size(); t += 0.01) {
            distance += path.get((int) (t-0.01)).getPoint((t-0.01) % 1).distance(path.get((int) t).getPoint(t % 1));
            if(distance > maxLookahead) {
                return t;
            }
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
