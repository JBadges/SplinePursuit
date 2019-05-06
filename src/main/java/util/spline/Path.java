package util.spline;

import util.Point;

import java.util.ArrayList;
import java.util.List;

public class Path {

	private List<Spline> path;

	public Path(List<Spline> path) {
		this.path = path;
	}

	public Path(Point... points) {
		List<Spline> splines = new ArrayList<>();
		for(int i = 0; i < points.length-1; i++) {
			splines.add(new QuinticHermiteSpline(points[i], points[i+1]));
		}
		this.path = splines;
	}

	public Point getPoint(double i) {
		if(i < 0) {
			return path.get(0).getPoint(0);
		}
		if(i >= path.size()) {
			return path.get(path.size()-1).getPoint(1);
		}
		return path.get((int) i).getPoint(i % 1);
	}

	public List<Spline> getPath() {
		return path;
	}

	public Spline get(int i) {
		return path.get(i);
	}

	public int size() {
		return path.size();
	}

}
