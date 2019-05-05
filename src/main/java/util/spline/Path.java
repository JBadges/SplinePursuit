package util.spline;

import util.Point;

import java.util.ArrayList;
import java.util.List;

public class Path {

	private List<Spline> path;
	private final double dT = 1e-6;

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

//	public double getTotalDistance() {
//		double dist = 0;
//		for (double t = 0; t < path.size(); t += dT) {
//			dist += Math.sqrt(path.get((int)t).dx(t%1) * path.get((int)t).dx(t%1) + path.get((int)t).dy(t%1) * path.get((int)t).dy(t%1)) * dT;
//		}
//		return dist;
//	}

}
