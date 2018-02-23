package tools;

import java.awt.Point;
import java.awt.geom.Point2D;

public class Ligne {
    private Point p;
    private Point2D.Double vecteurDir;

    public Ligne(Point p, Point vecteurDir) {
	this.p = p;
	this.vecteurDir = new Point2D.Double(vecteurDir.getX(),
		vecteurDir.getY());
    }

    public Ligne(Point p, Point2D.Double vecteurDir) {
	this.p = p;
	this.vecteurDir = vecteurDir;
    }

    public Ligne(Point p, Point a, Point b) {
	this.p = p;
	this.vecteurDir = ConstruireVecteur(a, b);
    }

    public Ligne(Point p, Point2D.Double a, Point2D.Double b) {
	this.p = p;
	this.vecteurDir = ConstruireVecteur(a, b);
    }

    public static Point2D.Double ConstruireVecteur(Point a, Point b) {
	return new Point2D.Double((b.getX() - a.getX()), (b.getY() - a.getY()));
    }

    public static Point2D.Double ConstruireVecteur(Point2D.Double a,
	    Point2D.Double b) {
	return new Point2D.Double((b.getX() - a.getX()), (b.getY() - a.getY()));
    }

    public Point getP() {
	return p;
    }

    public void setP(Point p) {
	this.p = p;
    }

    public Point2D.Double getVecteurDir() {
	return vecteurDir;
    }

    public void setVecteurDir(Point2D.Double vecteurDir) {
	this.vecteurDir = vecteurDir;
    }
}
