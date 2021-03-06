package main;

import auto.AutoModeExecutor;
import auto.actions.SplineDrivePath;
import auto.modes.DriveMode;
import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.stage.Stage;
import util.Point;
import util.spline.Path;
import util.spline.QuinticHermiteSpline;

import java.io.File;
import java.util.Arrays;
import java.util.List;

/**
 * Created by jacks on 2019-05-04.
 */
public class Main extends Application {
    public static AutoModeExecutor auto;
    public static Dial throttle, curve;

    public static void main(String[] args) {
        new File("path.csv").delete();
        auto = new AutoModeExecutor();
        auto.setAutoMode(new DriveMode());
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) {
        SplineDrivePath.pane = new Pane();
        List<List<QuinticHermiteSpline>> splines = Arrays.asList(new Path(
                new Point(0, 0, Math.PI/2),
                new Point(0.7,0.8,0),
                new Point(1,0.8,-Math.PI/2),
                new Point(.5,.5,Math.PI/2),
                new Point(.35,.6, Math.PI)).getPath());

        auto.start();

        for(int i = 0; i < splines.size(); i++) {
            for (double t = 0; t < splines.get(i).size(); t += 0.01) {
                Point p = splines.get(i).get((int) t).getPoint(t % 1);
                Circle c = new Circle(p.getX() * 600, 600 - p.getY() * 600, 3);
                c.setFill(Color.BLUE);
                SplineDrivePath.pane.getChildren().add(c);
            }
        }
        SplineDrivePath.goalCircle.setFill(Color.RED);
        SplineDrivePath.robotGuessedCircle.setFill(Color.YELLOW);
        SplineDrivePath.robotCircle.setFill(Color.GREEN);
        SplineDrivePath.robotCircle.setOpacity(0.25);
        SplineDrivePath.headingLine.setFill(Color.BLACK);
        SplineDrivePath.headingLine.setOpacity(0.75);

        SplineDrivePath.pane.getChildren().add(SplineDrivePath.goalCircle);
        SplineDrivePath.pane.getChildren().add(SplineDrivePath.robotGuessedCircle);
        SplineDrivePath.pane.getChildren().add(SplineDrivePath.robotCircle);
        SplineDrivePath.pane.getChildren().add(SplineDrivePath.headingLine);
        SplineDrivePath.pane.getChildren().add(SplineDrivePath.data);
        throttle = new Dial(50, true, 8,4, "Throttle", Color.RED, false);
        curve = new Dial(50, true, 8,4, "Curve", Color.RED, false);
        throttle.setLayoutX(100);
        throttle.setLayoutY(100);
        curve.setLayoutX(500);
        curve.setLayoutY(100);
        SplineDrivePath.pane.getChildren().add(throttle);
        SplineDrivePath.pane.getChildren().add(curve);

        Scene scene = new Scene(SplineDrivePath.pane, 600, 600);
        primaryStage.setScene(scene);
        primaryStage.show();

        scene.setOnKeyPressed(e -> {
            if (e.getCode() == KeyCode.SPACE) {
                auto.stop();
            }
        });
    }

    public void stop() {
        auto.stop();
    }
}
