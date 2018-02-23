package tools;

import java.awt.geom.Point2D;

public class Rectangle {
    private Point2D.Double phg, pbg, phd, pbd;

    public Rectangle(Point2D.Double phg, Point2D.Double pbg,
	    Point2D.Double phd, Point2D.Double pbd) {
	this.phg = phg;
	this.pbg = pbg;
	this.phd = phd;
	this.pbd = pbd;
    }

    public Point2D.Double getPhg() {
	return phg;
    }

    public void setPhg(Point2D.Double phg) {
	this.phg = phg;
    }

    public Point2D.Double getPbg() {
	return pbg;
    }

    public void setPbg(Point2D.Double pbg) {
	this.pbg = pbg;
    }

    public Point2D.Double getPhd() {
	return phd;
    }

    public void setPhd(Point2D.Double phd) {
	this.phd = phd;
    }

    public Point2D.Double getPbd() {
	return pbd;
    }

    public void setPbd(Point2D.Double pbd) {
	this.pbd = pbd;
    }

}
