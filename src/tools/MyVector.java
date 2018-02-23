package tools;

public class MyVector
{
    public double x;
    public double y;

    public MyVector (double _x, double _y)
    {
    	x = _x;
    	y = _y;
    }
    
    public void min(MyVector vec)
    {
        x = Math.min(x, vec.x);
        y = Math.min(y, vec.y);
    }

    public void max(MyVector vec)
    {
        x = Math.max(x, vec.x);
        y = Math.max(y, vec.y);
    }
    
    public MyVector midpoint(MyVector vec)
    {
        return new MyVector((x+vec.x)*0.5, (y+vec.y)*0.5);
    }

    public MyVector clone()
    {
        return new MyVector(this.x, this.y);
    }
    
    public double length()
    {
        return Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public void normalize()
    {
        double len = this.length();
        this.x /= len;
        this.y /= len;
    }

    public MyVector normalized()
    {
    	MyVector vec = new MyVector(this.x, this.y);
        vec.normalize();
        return vec;
    }

    public void negate()
    {
        this.x = -this.x;
        this.y = -this.y;
    }

    public double sqrLength()
    {
        return this.x * this.x + this.y * this.y;
    }

    public void scale(double len)
    {
        this.x *= len;
        this.y *= len;
    }

    public void add(MyVector vec)
    {
        this.x += vec.x;
        this.y += vec.y;
    }

    public void sub (MyVector vec)
    {
        this.x -= vec.x;
        this.y -= vec.y;
    }

    public MyVector diff(MyVector vec)
    {
        return new MyVector(this.x-vec.x, this.y-vec.y);
    }

    public double distance (MyVector vec)
    {
        double x = this.x-vec.x;
        double y = this.y-vec.y;
        return Math.sqrt(x*x+y*y);
    }

    public double dot (MyVector vec)
    {
        return this.x*vec.x+this.y*vec.y;
    }

    public boolean equals (Object vec)
    {
    	if(this == vec) 
    		return true;
    	if(vec instanceof MyVector) {
    		MyVector v = (MyVector) vec;
    		if(Double.compare(v.x, this.x)== 0 && Double.compare(v.y, this.y) == 0)
    			return true;
    	}
        return false;
    }

    public MyVector orthogonal()
    {
        return new MyVector(this.y, -this.x);
    }

    public double distanceToLine(MyVector v0,MyVector v1)
    {
        double sqrLen = v1.diff(v0).sqrLength();
        double u = ((this.x-v0.x)*(v1.x-v0.x)+(this.y-v0.y)*(v1.y-v0.y))/sqrLen;
        MyVector v1c = v1.diff(v0);
        v1c.scale(u);
        MyVector pl = v0.clone();
        pl.add(v1c);
        return this.distance(pl);
    }
}
