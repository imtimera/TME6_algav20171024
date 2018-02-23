package algorithms;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Vector;

import supportGUI.Circle;
import supportGUI.Line;

import tools.Ligne;
import tools.MyVector;
import tools.Rectangle;

public class DefaultTeam {
	
	public ArrayList<MyVector> BestObb = new ArrayList();
	public double BestObbArea = Double.MAX_VALUE;

	public ArrayList<MyVector> pointsToMyVector(ArrayList<Point> points){
		ArrayList<MyVector> listVec = new ArrayList();
		for (Point p : points) {
			listVec.add(new MyVector(p.getX(), p.getY()));
		}
		return listVec;
	}
	
	public ArrayList<Point> myVectorToPoint(ArrayList<MyVector> myVectors){
		ArrayList<Point> points = new ArrayList();
		for (MyVector myV : myVectors) {
			points.add(new Point((int)myV.x, (int)myV.y));
		}
		return points;
	}
	
	public MyVector IntersectLines(MyVector start0, MyVector dir0, MyVector start1, MyVector dir1)
	{
		
	    double dd = dir0.x*dir1.y-dir0.y*dir1.x;
	    // dd=0 => lines are parallel. we don't care as our lines are never parallel.
	    double dx = start1.x-start0.x; 
	    double dy = start1.y-start0.y;
	    double t = (dx*dir1.y-dy*dir1.x)/dd; 
		
	    return new MyVector(start0.x+t*dir0.x, start0.y+t*dir0.y); 
	}
	
	
	public ArrayList<MyVector> CalcOmbb(ArrayList<MyVector> convexHull)
	{

		ArrayList<MyVector> edgeDirs = new ArrayList<MyVector>();
	    
	    for (int i=0; i<convexHull.size(); i++)
	    {
	        edgeDirs.add(convexHull.get((i+1)%convexHull.size()).diff(convexHull.get(i)));
	        edgeDirs.get(i).normalize();
	    }
	    MyVector minPt = new MyVector(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
	    MyVector maxPt = new MyVector(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);
	    int leftIdx = 0, rightIdx = 0, topIdx = 0, bottomIdx = 0;
	    
	    for (int i=0; i<convexHull.size(); i++)
	    {
	        MyVector pt = convexHull.get(i);
	        
	        if (pt.x < minPt.x)
	        {
	            minPt.x = pt.x;
	            leftIdx = i;
	        }
	        
	        if (pt.x > maxPt.x)
	        {
	            maxPt.x = pt.x;
	            rightIdx = i;
	        }
	        
	        if (pt.y < minPt.y)
	        {
	            minPt.y = pt.y;
	            bottomIdx = i;
	        }

	        if (pt.y > maxPt.y)
	        {
	            maxPt.y = pt.y;
	            topIdx = i;
	        }
	    }     
	    
	    MyVector leftDir = new MyVector(0.0, -1);
	    MyVector rightDir = new MyVector(0, 1);
	    MyVector topDir = new MyVector(-1, 0);
	    MyVector bottomDir = new MyVector(1, 0);
	    
	    for (int i=0; i<convexHull.size(); i++)
	    {
	        // of course the acos() can be optimized.
	        // but it's a JS prototype anyways, so who cares.
	        ArrayList<Double> phis = new ArrayList<Double>();// 0=left, 1=right, 2=top, 3=bottom
	        
	        phis.add(Math.acos(leftDir.dot(edgeDirs.get(leftIdx))));
	        phis.add(Math.acos(rightDir.dot(edgeDirs.get(rightIdx))));
    		phis.add(Math.acos(topDir.dot(edgeDirs.get(topIdx))));
			phis.add(Math.acos(bottomDir.dot(edgeDirs.get(bottomIdx))));

	        
	       // int lineWithSmallestAngle = phis.indexOf(Math.min.apply(Math, phis));
	        double min1 = Math.min(phis.get(0), phis.get(1));
	        double min2 = Math.min(phis.get(2), phis.get(3));
	        double min3 = Math.min(min1, min2);
	        int lineWithSmallestAngle = phis.indexOf(min3);
	        switch (lineWithSmallestAngle)
	        {
	        case 0: // left
	            leftDir = edgeDirs.get(leftIdx).clone();
	            rightDir = leftDir.clone();
	            rightDir.negate();
	            topDir = leftDir.orthogonal();
	            bottomDir = topDir.clone();
	            bottomDir.negate();
	            leftIdx = (leftIdx+1)%convexHull.size();
	            break;
	        case 1: // right
	            rightDir = edgeDirs.get(rightIdx).clone();
	            leftDir = rightDir.clone();
	            leftDir.negate();
	            topDir = leftDir.orthogonal();
	            bottomDir = topDir.clone();
	            bottomDir.negate();
	            rightIdx = (rightIdx+1)%convexHull.size();
	            break;
	        case 2: // top
	            topDir = edgeDirs.get(topIdx).clone();
	            bottomDir = topDir.clone();
	            bottomDir.negate();
	            leftDir = bottomDir.orthogonal();
	            rightDir = leftDir.clone();
	            rightDir.negate();
	            topIdx = (topIdx+1)%convexHull.size();
	            break;
	        case 3: // bottom
	            bottomDir = edgeDirs.get(bottomIdx).clone();
	            topDir = bottomDir.clone();
	            topDir.negate();
	            leftDir = bottomDir.orthogonal();
	            rightDir = leftDir.clone();
	            rightDir.negate();
	            bottomIdx = (bottomIdx+1)%convexHull.size();
	            break;
	        }                   
	        
	        this.UpdateOmbb(convexHull.get(leftIdx), leftDir, convexHull.get(rightIdx), rightDir, convexHull.get(topIdx), topDir, convexHull.get(bottomIdx), bottomDir);
	    }
	    
	    
	    return BestObb;
	}
	
	
	
	public void UpdateOmbb(MyVector leftStart, MyVector leftDir, MyVector rightStart, MyVector rightDir, MyVector topStart, MyVector topDir, MyVector bottomStart, MyVector bottomDir)
	{
		    MyVector obbUpperLeft = IntersectLines(leftStart, leftDir, topStart, topDir);
	        MyVector obbUpperRight = IntersectLines(rightStart, rightDir, topStart, topDir);
	        MyVector obbBottomLeft = IntersectLines(bottomStart, bottomDir, leftStart, leftDir);
	        MyVector obbBottomRight = IntersectLines(bottomStart, bottomDir, rightStart, rightDir);
	        double distLeftRight = obbUpperLeft.distance(obbUpperRight);        
	        double distTopBottom = obbUpperLeft.distance(obbBottomLeft);
	        double obbArea = distLeftRight*distTopBottom;
	        
	        if (obbArea < this.BestObbArea)
	        {
	        	BestObb.clear();
	            BestObb.add(obbUpperLeft);
	            BestObb.add(obbBottomLeft);
	            BestObb.add(obbBottomRight);
	            BestObb.add(obbUpperRight);
	            BestObbArea = obbArea;
	        }
	}
	
	public static ArrayList<Point> ToussaintRectMin(ArrayList<Point> enveloppe) {
		
		int xmin = Integer.MAX_VALUE, xmax = Integer.MIN_VALUE;
		int ymin = Integer.MAX_VALUE, ymax = Integer.MIN_VALUE;
		double a1, a2, a3, a4, angleMin, aire;
		double longueur, largeur, Amin = Double.MAX_VALUE;

		Point Pleft = null, Pright = null, Ptop = null, Pbottom = null;
		Point2D.Double phg, pbg, phd, pbd;
		Ligne leftV, rightV, topH, bottomH;
		Rectangle rectangleMin;
		ArrayList<Double> angles;
		// ArrayList<Point> enveloppe = Graham.enveloppeConvexeGraham(points);
		// Circle c = Ritter.calculCercleMin(points);
		//@SuppressWarnings("unused")
		//WindowView f;

		/* Recuperation du point le plus a l'ouest de l'enveloppe */
		/* Recuperation du point le plus a l'est de l'enveloppe */
		/* Recuperation du point le plus au sud de l'enveloppe */
		/* Recuperation du point le plus au nord de l'enveloppe */
		for (Point p : enveloppe) {
		    if (p.x < xmin) {
			xmin = p.x;
			Pleft = p;
		    }
		    if (p.x > xmax) {
			xmax = p.x;
			Pright = p;
		    }
		    if (p.y < ymin) {
			ymin = p.y;
			Ptop = p;
		    }
		    if (p.y > ymax) {
			ymax = p.y;
			Pbottom = p;
		    }
		}
		
		
		
		

		leftV = new Ligne(Pleft, new Point(0, -1));
		rightV = new Ligne(Pright, new Point(0, 1));
		topH = new Ligne(Ptop, new Point(1, 0));
		bottomH = new Ligne(Pbottom, new Point(-1, 0));

		phg = intersectionDroites(topH, leftV);
		pbg = intersectionDroites(bottomH, leftV);
		phd = intersectionDroites(topH, rightV);
		pbd = intersectionDroites(bottomH, rightV);

		/*Point Ppbg = new Point();
		Ppbg.x = (int)phg.getX();
		Ppbg.y = (int)phg.getY();
		
		Point Pphg = new Point();
		Pphg.x = (int)pbg.getX();
		Pphg.y = (int)pbg.getY();
		
		Point Ppbd = new Point();
		Ppbd.x = (int)phd.getX();
		Ppbd.y = (int)phd.getY();
		
		Point Pphd= new Point();
		Pphd.x = (int)pbd.getX();
		Pphd.y = (int)pbd.getY();
		*/
		rectangleMin = new Rectangle(phg, pbg, phd, pbd);

		/* calcul de l'aire du rectangle Longueur*largeur */
		longueur = phg.distance(pbg);
		largeur = phg.distance(phd);
		Amin = longueur * largeur;
		
		
		
		
		ArrayList<Point> toto = new ArrayList<Point>();
		Point Pphg = new Point();
		Point Ppbg = new Point();
		Point Ppbd = new Point();
		Point Pphd= new Point();
		
		for (int j = 0; j < enveloppe.size(); j++) {

		    /* calculer les 4 angles puis chercher angle minmal */

		    a1 = calculAngle(
			    topH,
			    Ptop,
			    enveloppe.get((enveloppe.indexOf(Ptop) + 1)
				    % enveloppe.size()));
		    a2 = calculAngle(
			    bottomH,
			    Pbottom,
			    enveloppe.get((enveloppe.indexOf(Pbottom) + 1)
				    % enveloppe.size()));
		    a3 = calculAngle(
			    leftV,
			    Pleft,
			    enveloppe.get((enveloppe.indexOf(Pleft) + 1)
				    % enveloppe.size()));
		    a4 = calculAngle(
			    rightV,
			    Pright,
			    enveloppe.get((enveloppe.indexOf(Pright) + 1)
				    % enveloppe.size()));

		    angles = new ArrayList<Double>();

		    angles.add(a1);
		    angles.add(a2);
		    angles.add(a3);
		    angles.add(a4);

		    angleMin = Collections.min(angles);

		    /* rotation des 4 droites de angleMin */

		    rotationDroite(topH, -angleMin);
		    rotationDroite(bottomH, -angleMin);
		    rotationDroite(leftV, -angleMin);
		    rotationDroite(rightV, -angleMin);

		    /* calcul des nouveaux points d'intersections des 4 droites */

		    phg = intersectionDroites(topH, leftV);
		    pbg = intersectionDroites(bottomH, leftV);
		    phd = intersectionDroites(topH, rightV);
		    pbd = intersectionDroites(bottomH, rightV);
		    
			Pphg.x = (int)phg.getX();
			Pphg.y = (int)phg.getY();		    
		    
			Ppbg.x = (int)pbg.getX();
			Ppbg.y = (int)pbg.getY();
			
			Ppbd.x = (int)pbd.getX();
			Ppbd.y = (int)pbd.getY();
			
			Pphd.x = (int)phd.getX();
			Pphd.y = (int)phd.getY();
				
			
			
			if(j==0)
				break;
		       
			
			
		    
		    /* calcul du nouvel air et sauvegarde du nouveau rectangle */
		    longueur = Point2D.Double.distance(phg.getX(), phg.getY(),
			    pbg.getX(), pbg.getY());
		    largeur = Point2D.Double.distance(phg.getX(), phg.getY(),
			    phd.getX(), phd.getY());
		    aire = longueur * largeur;

		    if (aire < Amin) {
			Amin = aire;
			rectangleMin = new Rectangle(phg, pbg, phd, pbd);
		    }

		    /*
		     * f = new Fenetre(i, new Segment(pbg, phg), new Segment(pbg, pbd),
		     * new Segment(phg, phd), new Segment(pbd, phd), points, enveloppe,
		     * c);
		     */

		    if (angleMin == a1) {
			Ptop = enveloppe.get((enveloppe.indexOf(Ptop) + 1)
				% enveloppe.size());
			topH.setP(Ptop);
		    }
		    if (angleMin == a2) {
			Pbottom = enveloppe.get((enveloppe.indexOf(Pbottom) + 1)
				% enveloppe.size());
			bottomH.setP(Pbottom);
		    }
		    if (angleMin == a3) {
			Pleft = enveloppe.get((enveloppe.indexOf(Pleft) + 1)
				% enveloppe.size());
			leftV.setP(Pleft);
		    }
		    if (angleMin == a4) {
			Pright = enveloppe.get((enveloppe.indexOf(Pright) + 1)
				% enveloppe.size());
			rightV.setP(Pright);
		    }

		}
		toto.add(Ppbg);toto.add(Pphg);toto.add(Pphd);toto.add(Ppbd);
		pbg = rectangleMin.getPbg();
		phg = rectangleMin.getPhg();
		pbd = rectangleMin.getPbd();
		phd = rectangleMin.getPhd();
		
		/*Point Ppbg = new Point();
		Ppbg.x = (int)pbg.getX();
		Ppbg.y = (int)pbg.getY();
		
		Point Pphg = new Point();
		Pphg.x = (int)phg.getX();
		Pphg.y = (int)phg.getY();
		
		Point Ppbd = new Point();
		Ppbd.x = (int)pbd.getX();
		Ppbd.y = (int)pbd.getY();
		
		Point Pphd= new Point();
		Pphd.x = (int)phd.getX();
		Pphd.y = (int)phd.getY();
		*/
		ArrayList<Point> l = new ArrayList<Point>();
		//l.add(Ppbg);l.add(Ppbd);l.add(Pphd);l.add(Pphg);
		//l.add(Ppbg);l.add(Ppbd);l.add(Pphd);l.add(Pphg);
		/*
		 * f = new Fenetre(imin, new Segment(pbg, phg), new Segment(pbg, pbd),
		 * new Segment(phg, phd), new Segment(pbd, phd), points, enveloppe, c);
		 */

		//return rectangleMin;
		return toto;
	    }

	    public static double calculAngle(Ligne l, Point p1, Point p2) {
		Point2D.Double v1, v2;
		double EPSILON = 0.00000001, res;
		Vector v = new Vector();
		
		v1 = l.getVecteurDir();
		v2 = Ligne.ConstruireVecteur(p1, p2);
		res = Math
			.abs(Math.atan(((v1.getY() * v2.getX() - v2.getY() * v1.getX()) / (v1
				.getX() * v2.getX() + v1.getY() * v2.getY()))));
		if (res < EPSILON)
		    return 0;

		return res;
	    }

	    public static void rotationDroite(Ligne l, double angle) {
		double x, y;
		Point2D.Double v = l.getVecteurDir();

		x = (v.getX() * Math.cos(angle)) + (v.getY() * Math.sin(angle));
		y = (-(v.getX() * Math.sin(angle))) + (v.getY() * Math.cos(angle));

		l.setVecteurDir(new Point2D.Double(x, y));
	    }

	    public static Point2D.Double intersectionDroites(Ligne l1, Ligne l2) {
		double t;
		Point2D.Double q, p, a, s, r;

		q = new Point2D.Double(l1.getP().getX(), l1.getP().getY());
		p = new Point2D.Double(l2.getP().getX(), l2.getP().getY());
		a = new Point2D.Double(q.getX() - p.getX(), q.getY() - p.getY());
		s = l1.getVecteurDir();
		r = l2.getVecteurDir();

		t = (a.getX() * s.getY() - a.getY() * s.getX())
			/ (r.getX() * s.getY() - r.getY() * s.getX());

		return new Point2D.Double((p.getX() + (t * r.getX())),
			(p.getY() + (t * r.getY())));
	    }
	
	
	
	

    // calculDiametre: ArrayList<Point> --> Line
    //   renvoie une pair de points de la liste, de distance maximum.
    public Line calculDiametre(ArrayList<Point> points) {
        return tme7naif(points);
    }
    private Line tme7naif(ArrayList<Point> points) {
        if (points.size()<2) {
            return null;
        }
       
        Point p=points.get(0);
        Point q=points.get(1);

        for (Point s: points) for (Point t: points) if (s.distance(t)>p.distance(q)) {p=s;q=t;}

        return new Line(p,q);
    }

    // calculDiametreOptimise: ArrayList<Point> --> Line
    //   renvoie une pair de points de la liste, de distance maximum.
    public Line calculDiametreOptimise(ArrayList<Point> points) {
        return tme7exercice2(points);
    }
    private Line tme7exercice2(ArrayList<Point> points) {
        if (points.size()<2) {
            return null;
        }

        ArrayList<Line> antipodales = calculPairesAntipodales(points);

        Point p=antipodales.get(0).getP();
        Point q=antipodales.get(0).getQ();

        for (Line a: antipodales) if (a.getP().distance(a.getQ())>p.distance(q)) {p=a.getP();q=a.getQ();}

        return new Line(p,q);
    }
    //################## paires Antipodale ##############
    private ArrayList<Line> calculPairesAntipodales(ArrayList<Point> points) {
        ArrayList<Point> p = tme6exercice5(points); // p est l'enveloppe convexe de points
        int n = p.size();
        ArrayList<Line> antipodales = new ArrayList<Line>();
        int k = 1;
        while (distance(p.get(k),p.get(n-1),p.get(0)) < distance(p.get((k+1)%n),p.get(n-1),p.get(0))) k++;
        int i = 0;
        int j = k;
        while (i<=k && j<n) {
            while (distance(p.get(j),p.get(i),p.get(i+1))<distance(p.get((j+1)%n),p.get(i),p.get(i+1)) && j<n-1) {
                antipodales.add(new Line(p.get(i),p.get(j)));
                j++;
            }
            antipodales.add(new Line(p.get(i),p.get(j)));
            i++;
        }
        return antipodales;
    }
    
    private double distance(Point p, Point a, Point b) {
        return Math.abs(crossProduct(a,b,a,p));
    }

    // calculCercleMin: ArrayList<Point> --> Circle
    //   renvoie un cercle couvrant tout point de la liste, de rayon minimum.
    public Circle calculCercleMin(ArrayList<Point> points) {
        if (points.isEmpty()) {
            return null;
        }

        Point center=points.get(0);
        int radius=100;

        /*******************
         * PARTIE A ECRIRE *
         *******************/
        return new Circle(center,radius);
    }

    // enveloppeConvexe: ArrayList<Point> --> ArrayList<Point>
    //   renvoie l'enveloppe convexe de la liste.
    public ArrayList<Point> enveloppeConvexe(ArrayList<Point> points){
        //return tme6exercice1(points);
        //return tme6exercice2(points);
        //return tme6exercice1(tme6exercice2(points));
        //return tme6exercice1(tme6exercice3(points));
        //return tme6exercice4(points);
        //return tme6exercice5(points);

       ArrayList<Point> enveloppeConvexe = tme6exercice5(points);
       //ToussaintRectMin(CalcOmbb(pointsToMyVector(points)));
       
       
     
       return tme6exercice3(ToussaintRectMin(enveloppeConvexe));
        //return tme6exercice3(myVectorToPoint((CalcOmbb(pointsToMyVector(enveloppeConvexe)))));
    }
    
    public ArrayList<Point> Toussaint(ArrayList<Point> enveloppeConvexe, ArrayList<Line> PairesAntipodales){
    	
    	ArrayList<Point> pointRectangle = new ArrayList<Point>();
    	
    	for(Line line : PairesAntipodales)
    	{
    		
    	}
        
        return tme6exercice3(pointRectangle);
    }
    private ArrayList<Point> tme6exercice1(ArrayList<Point> points){
        if (points.size()<4) return points;

        ArrayList<Point> enveloppe = new ArrayList<Point>();

        for (Point p: points) {
            for (Point q: points) {
                if (p.equals(q)) continue;
                Point ref=p;
                for (Point r: points) if (crossProduct(p,q,p,r)!=0) {ref=r;break;}
                if (ref.equals(p)) {enveloppe.add(p); enveloppe.add(q); continue;}
                double signeRef = crossProduct(p,q,p,ref);
                boolean estCote = true;
                for (Point r: points) if (signeRef * crossProduct(p,q,p,r)<0) {estCote = false;break;} //ici sans le break le temps de calcul devient horrible
                if (estCote) {enveloppe.add(p); enveloppe.add(q);}
            }
        }

        return enveloppe; //ici l'enveloppe n'est pas trie dans le sens trigonometrique, et contient des doublons, mais tant pis!
    }
    private double crossProduct(Point p, Point q, Point s, Point t){
        return ((q.x-p.x)*(t.y-s.y)-(q.y-p.y)*(t.x-s.x));
    }
    private ArrayList<Point> tme6exercice2(ArrayList<Point> points){
        if (points.size()<4) return points;
        int maxX=points.get(0).x;
        for (Point p: points) if (p.x>maxX) maxX=p.x;
        Point[] maxY = new Point[maxX+1];
        Point[] minY = new Point[maxX+1];
        for (Point p: points) {
            if (maxY[p.x]==null||p.y>maxY[p.x].y) maxY[p.x]=p;
            if (minY[p.x]==null||p.y<minY[p.x].y) minY[p.x]=p;
        }
        ArrayList<Point> result = new ArrayList<Point>();
        for (int i=0;i<maxX+1;i++) if (maxY[i]!=null) result.add(maxY[i]);
        for (int i=maxX;i>=0;i--) if (minY[i]!=null && !result.get(result.size()-1).equals(minY[i])) result.add(minY[i]);

        if (result.get(result.size()-1).equals(result.get(0))) result.remove(result.size()-1);

        return result;
    }
    //####################"" rectangle minimum ########################
    private ArrayList<Point> tme6exercice3(ArrayList<Point> points){
        if (points.size()<4) return points;

        Point ouest = points.get(0);
        Point sud = points.get(0);
        Point est = points.get(0);
        Point nord = points.get(0);
        for (Point p: points){
            if (p.x<ouest.x) ouest=p;
            if (p.y>sud.y) sud=p;
            if (p.x>est.x) est=p;
            if (p.y<nord.y) nord=p;
        }
        ArrayList<Point> result = (ArrayList<Point>)points.clone();
        for (int i=0;i<result.size();i++) {
            if (triangleContientPoint(ouest,sud,est,result.get(i)) ||
                    triangleContientPoint(ouest,est,nord,result.get(i))) {
                result.remove(i);
                i--;
            }
        }
        
        return result;
    }
    
    
    private boolean triangleContientPoint(Point a, Point b, Point c, Point x) {
        double l1 = ((b.y-c.y)*(x.x-c.x)+(c.x-b.x)*(x.y-c.y))/(double)((b.y-c.y)*(a.x-c.x)+(c.x-b.x)*(a.y-c.y));
        double l2 = ((c.y-a.y)*(x.x-c.x)+(a.x-c.x)*(x.y-c.y))/(double)((b.y-c.y)*(a.x-c.x)+(c.x-b.x)*(a.y-c.y));
        double l3 = 1-l1-l2;
        return (0<l1 && l1<1 && 0<l2 && l2<1 && 0<l3 && l3<1);
    }
    private ArrayList<Point> tme6exercice4(ArrayList<Point> points){
        if (points.size()<4) return points;

        Point ouest = points.get(0);
        for (Point p: points) if (p.x<ouest.x||(p.x==ouest.x && p.y>ouest.x)) ouest=p;
        ArrayList<Point> enveloppe = new ArrayList<Point>();
        enveloppe.add(ouest);
        for (Point q: points) {
            if (q.equals(ouest)) continue;
            Point ref=q;
            for (Point r: points) if (crossProduct(ouest,q,ouest,r)!=0) {ref=r;break;}
            if (ref.equals(q)) { enveloppe.add(q); continue;}
            double signeRef = crossProduct(ouest,q,ouest,ref);
            boolean estCote = true;
            for (Point r: points) if (signeRef * crossProduct(ouest,q,ouest,r)<0) {estCote = false;break;}
            if (estCote) {enveloppe.add(q);break;}
        }

        do {
            Point p = enveloppe.get(enveloppe.size()-2);
            Point q = enveloppe.get(enveloppe.size()-1);
            Point r = points.get(0);
            for (Point s: points) if (!s.equals(p) && !s.equals(q)) {r=s;break;}
            for (Point s: points) {
                if (s.equals(p)||s.equals(q)) continue;
                if (angle(p,q,q,s)<angle(p,q,q,r)) r=s;
            }
            enveloppe.add(r);
        } while (!enveloppe.get(enveloppe.size()-1).equals(enveloppe.get(0)));
        enveloppe.remove(0);
        return enveloppe;
    }
    private double angle(Point p, Point q, Point s, Point t) {
        if (p.equals(q)||s.equals(t)) return Double.MAX_VALUE;
        double cosTheta = dotProduct(p,q,s,t)/(double)(p.distance(q)*s.distance(t));
        return Math.acos(cosTheta);
    }
    private double dotProduct(Point p, Point q, Point s, Point t) {
        return ((q.x-p.x)*(t.x-s.x)+(q.y-p.y)*(t.y-s.y));
    }
   //############## enveloppeConvexe
    private ArrayList<Point> tme6exercice5(ArrayList<Point> points){
        if (points.size()<4) return points;

        ArrayList<Point> result = tme6exercice2(points);
        for (int i=1;i<result.size()+2;i++) {
            Point p = result.get((i-1)%result.size());
            Point q = result.get(i%result.size());
            Point r = result.get((i+1)%(result.size()));
            if (crossProduct(p,q,p,r)>0) {
                result.remove(i%result.size());
                if (i==2) i=1;
                if (i>2) i-=2;
            }
        }

        return result;
    }
    //############################"enveloppe convexe#################################
    private ArrayList<Point> tme6exercice6(ArrayList<Point> points){
        if (points.size()<4) return points;

        Point ouest = points.get(0);
        Point sud = points.get(0);
        Point est = points.get(0);
        Point nord = points.get(0);
        for (Point p: points){
            if (p.x<ouest.x) ouest=p;
            if (p.y>sud.y) sud=p;
            if (p.x>est.x) est=p;
            if (p.y<nord.y) nord=p;
        }
        ArrayList<Point> result = new ArrayList<Point>();
        result.add(ouest);
        result.add(sud);
        result.add(est);
        result.add(nord);

        ArrayList<Point> rest = (ArrayList<Point>)points.clone();
        for (int i=0;i<rest.size();i++) {
            if (triangleContientPoint(ouest,sud,est,rest.get(i)) ||
                    triangleContientPoint(ouest,est,nord,rest.get(i))) {
                rest.remove(i);
                i--;
                    }
        }

        for (int i=0;i<result.size();i++) {
            Point a = result.get(i);
            Point b = result.get((i+1)%result.size());
            Point ref = result.get((i+2)%result.size());

            double signeRef = crossProduct(a,b,a,ref);
            double maxValue = 0;
            Point maxPoint = a;

            for (Point p: points) {
                double piki = crossProduct(a,b,a,p);
                if (signeRef*piki<0 && Math.abs(piki)>maxValue) {
                    maxValue = Math.abs(piki);
                    maxPoint = p;
                }
            }
            if (maxValue!=0){
                for (int j=0;j<rest.size();j++) {
                    if (triangleContientPoint(a,b,maxPoint,rest.get(j))){
                        rest.remove(j);
                        j--;
                    }
                }
                result.add(i+1,maxPoint);
                i--;
            }
        }

        return result;
    }
}