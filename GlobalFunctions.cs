using UnityEngine;
using System.Collections;

using System.Drawing;
using System.Collections.Generic;
using System.IO;
using System;
using System.Linq;

#if UNITY_EDITOR
using UnityEngine.iOS;
#endif

public class GlobalFunctions  {

	public static Rect scaleRectToWidth(Rect r, int width)
	{
		float scaling = (float)(r.width)/(float)(width);
		
		return new Rect((int)(r.x*scaling),
		                (int)(r.y*scaling),
		                (int)(r.width*scaling),
		                (int)(r.height*scaling)) ;
	}

	public static Rect scaleInverseRectToWidth(Rect r, int width)
	{
		float scaling = (float)(width)/(float)(r.width);
		
		return new Rect((int)(r.x*scaling),
		                (int)(r.y*scaling),
		                (int)(r.width*scaling),
		                (int)(r.height*scaling)) ;
	}
	

	public static float Remap (float value, float from1, float to1, float from2, float to2) {
		return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
	}

	public static Rectangle RectToRectangle(Rect rect)
	{
		return new Rectangle() {  Width = (int)rect.width, Height = (int)rect.height, X = (int)rect.x, Y = (int)rect.y };
	}
	
	public static Rect RectangleToRect(Rectangle rectangle)
	{
		return new Rect() { width = rectangle.Width, height = rectangle.Height, x = rectangle.X, y = rectangle.Y };
	}

	public static Rect ScaleRect(Rect r, float scale)
	{
		return new Rect (r.x * scale, r.y * scale, r.width * scale, r.height * scale);
	}

	public static float getAngle(Point p1, Point p2)
	{
		return Mathf.Atan2(p2.Y-p1.Y, p2.X-p1.X)*180 / Mathf.PI;
	}

	public static double getDistance(Point p1, Point p2)
	{
		return Math.Sqrt((p1.X - p2.X) ^ 2 + (p1.Y - p2.Y) ^ 2);
	}

	public static Vector2[] PointsToVector2(Point[] pts)
	{
		Vector2[] result = new Vector2[pts.Length];
		for (int x =0; x < pts.Length; x++) {
			result[x] = PointToVector2(pts[x]);
		}
		return result;
	}
	public static Vector2 PointToVector2(Point pt)
	{
		return  new Vector2(pt.X,pt.Y);
	}

	public static PointF[] PointsToPointsF(Point[] pt)
	{
		PointF[] fpts = new PointF[pt.Length];
		for (int x=0; x< pt.Length; x++)
			fpts [x] = PointToPointF (pt [x]);
		return  fpts;
	}

	public static Point[] PointsFToPoints(PointF[] ptF)
	{
		Point[] pts = new Point[ptF.Length];
		for (int x=0; x< ptF.Length; x++)
			pts [x] = PointFToPoint (ptF [x]);
		return  pts;
	}

	public static PointF PointToPointF(Point pt)
	{
		return  new PointF(pt.X,pt.Y);
	}


	public static Point PointFToPoint(PointF pt)
	{
		return  new Point((int)pt.X,(int)pt.Y);
	}

	public static Point[] scalePoints(Point[] pts, float scale, Point move)
	{
		for (int x=0; x< pts.Length; x++) {
			scalePoint(pts[x],scale,move);
		}
		return pts;
	}

	public static Point scalePoint(Point pt, float scale, Point move)
	{
			pt.X = move.X +(int)((float)(pt.X) * scale);
			pt.Y = move.Y +(int)((float)(pt.Y) * scale);

		return pt;
	}

	//Compute the dot product AB . AC
	public static double DotProduct(Point pointA, Point pointB, Point pointC)
	{
		double[] AB = new double[2];
		double[] BC = new double[2];
		AB[0] = (double)pointB.X - (double)pointA.X;
		AB[1] = (double)pointB.Y - (double)pointA.Y;
		BC[0] = (double)pointC.X - (double)pointA.X;
		BC[1] = (double)pointC.Y - (double)pointA.Y;
		double dot = AB[0] * BC[0] + AB[1] * BC[1];
		
		return dot;
	}


	public static double CrossProduct(Point pointA, Point pointB, Point pointC)
	{
		double[] AB = new double[2];
		double[] AC = new double[2];
		AB[0] = (double)pointB.X - (double)pointA.X;
		AB[1] = (double)pointB.Y - (double)pointA.Y;
		AC[0] = (double)pointC.X - (double)pointA.X;
		AC[1] = (double)pointC.Y - (double)pointA.Y;
		double cross = AB[0] * AC[1] - AB[1] * AC[0];
		
		return cross;
	}
	
	//Compute the distance from A to B
	public static double Distance(Point pointA, Point pointB)
	{
		double d1 = (double)pointA.X - (double)pointB.X;
		double d2 = (double)pointA.Y - (double)pointB.Y;
		
		return Math.Sqrt(d1 * d1 + d2 * d2);
	}



	//Compute the distance from AB to C
	//if isSegment is true, AB is a segment, not a line.
	public static double LineToPointDistance2D(Point pointA, Point pointB, Point pointC, 
	                             bool isSegment)
	{
		double dist = CrossProduct(pointA, pointB, pointC) / Distance(pointA, pointB);
		if (isSegment)
		{
			double dot1 = DotProduct(pointA, pointB, pointC);
			if (dot1 > 0) 
				return Distance(pointB, pointC);
			
			double dot2 = DotProduct(pointB, pointA, pointC);
			if (dot2 > 0) 
				return Distance(pointA, pointC);
		}
		return Math.Abs(dist);
	}  

	public static Vector2 Normalized2D(  Vector2 vector )  
	{
		float mag = vector.magnitude;
		if( mag > .00001 )
		{
			return (vector / mag);
		}
		return Vector2.zero;
	}
	
	public static Vector2 ClosestPointOnLine( Vector2 vA,  Vector2 vB,  Vector2 vPoint)
	{
		var vVector1 = vPoint - vA;
		var vVector2 = Normalized2D(vB - vA);
		
		var d = Vector2.Distance(vA, vB);
		var t = Vector2.Dot(vVector2, vVector1);
		
		if (t <= 0)
			return vA;
		
		if (t >= d)
			return vB;
		
		var vVector3 = vVector2 * t;
		
		var vClosestPoint = vA + vVector3;
		
		return vClosestPoint;
	}

	// Calculate the distance between
	// point pt and the segment p1 --> p2.
	public static double FindDistanceToSegment( PointF pt, PointF p1, PointF p2, out PointF closest)
	{
		float dx = p2.X - p1.X;
		float dy = p2.Y - p1.Y;
		if ((dx == 0) && (dy == 0))
		{
			// It's a point not a line segment.
			closest = p1;
			dx = pt.X - p1.X;
			dy = pt.Y - p1.Y;
			return Math.Sqrt(dx * dx + dy * dy);
		}
		
		// Calculate the t that minimizes the distance.
		float t = ((pt.X - p1.X) * dx + (pt.Y - p1.Y) * dy) /
			(dx * dx + dy * dy);
		
		// See if this represents one of the segment's
		// end points or a point in the middle.
		if (t < 0)
		{
			closest = new PointF(p1.X, p1.Y);
			dx = pt.X - p1.X;
			dy = pt.Y - p1.Y;
		}
		else if (t > 1)
		{
			closest = new PointF(p2.X, p2.Y);
			dx = pt.X - p2.X;
			dy = pt.Y - p2.Y;
		}
		else
		{
			closest = new PointF(p1.X + t * dx, p1.Y + t * dy);
			dx = pt.X - closest.X;
			dy = pt.Y - closest.Y;
		}
		
		return Math.Sqrt(dx * dx + dy * dy);
	}


	public static Point closestPointOfPolyline(Point[] polyline, Point pt)
	{
		PointF[] Ptsf = PointsToPointsF(polyline);
		PointF Ptf = PointToPointF (pt);

		double distanceClosest = Double.PositiveInfinity;
		PointF closestPoint = new PointF(); 

		for(int x=0; x< Ptsf.Length-1; x++)
		{
			PointF closest;
			double dist = FindDistanceToSegment(Ptf,
			                                    Ptsf[x],
			                                    Ptsf[x+1],
			                                    out closest);

			if(dist < distanceClosest)
			{
				distanceClosest = dist;
				closestPoint = closest;
			}
		}

		return PointFToPoint (closestPoint);
	}



	// GL functions
	public static void DrawShapeOnTexture(Texture2D tex,Point[] pts, Color col)
	{
		for (int x =0; x < pts.Length; x++) {
			Point pt = pts[x];
			if(x < pts.Length-1)
			{
				Point ptNext = pts[x+1];
				DrawLine (tex, pt.X, pt.Y, ptNext.X, ptNext.Y, col);
			}
			else
			{
				DrawLine (tex, pt.X, pt.Y, pts[0].X, pts[0].Y, col);
			}
		}
	}

	public static void DrawRectOnTexture(Texture2D tex, Rect r, Color col)
	{
		DrawLine (tex, r.x, r.y, r.x + r.width, r.y, col); // top left top right
		DrawLine (tex, r.x + r.width, r.y , r.x + r.width, r.y + r.height, col); // top right bottom right
		DrawLine (tex, r.x + r.width, r.y + r.height, r.x , r.y + r.height, col); // bottom right bottom left
		DrawLine (tex, r.x , r.y + r.height, r.x, r.y , col); // bottom left top left
	}
	public static void DrawLine(Texture2D tex,Point pt0,Point Pt1, Color col)
	{
		DrawLine (tex, pt0.X, pt0.Y, Pt1.X, Pt1.Y,col);
	}

	
	public static void DrawLine(Texture2D tex, float x0, float y0, float x1, float y1, Color col)
	{
		int dy = (int)(y1-y0);
		int dx = (int)(x1-x0);
		int stepx, stepy;
		
		if (dy < 0) {dy = -dy; stepy = -1;}
		else {stepy = 1;}
		if (dx < 0) {dx = -dx; stepx = -1;}
		else {stepx = 1;}
		dy <<= 1;
		dx <<= 1;
		
		float fraction = 0;
		
		tex.SetPixel((int)x0, (int)y0, col);
		if (dx > dy) {
			fraction = dy - (dx >> 1);
			while (Mathf.Abs(x0 - x1) > 1) {
				if (fraction >= 0) {
					y0 += stepy;
					fraction -= dx;
				}
				x0 += stepx;
				fraction += dy;

				//Color final_color = Color.Lerp(t, wmColor, wmColor.a / 1.0f);
				tex.SetPixel((int)x0, (int)y0, col);
			}
		}
		else {
			fraction = dx - (dy >> 1);
			while (Mathf.Abs(y0 - y1) > 1) {
				if (fraction >= 0) {
					x0 += stepx;
					fraction -= dy;
				}
				y0 += stepy;
				fraction += dx;
				tex.SetPixel((int)x0, (int)y0, col);
			}
		}
	}

	public static Texture2D MergeTextures(Texture2D background, Texture2D watermark)
	{
		
		int startX = 0;
		int startY = 0;
		Color firstColor = watermark.GetPixel (0, 0);
		Debug.Log (firstColor);
		for (int x = startX; x < background.width; x++)
		{
			for (int y = startY; y < background.height; y++)
			{
				Color bgColor = background.GetPixel(x, y);
				Color wmColor = watermark.GetPixel(x - startX, y - startY);
				if(firstColor != wmColor)
				{
					Color final_color = Color.Lerp(bgColor, wmColor, wmColor.a / 1.0f);
				
					background.SetPixel(x, y, final_color);
				} else
					background.SetPixel(x, y, bgColor);
			}
		}
		
		background.Apply();
		return background;
	}
	

	public static List<Point> DouglasPeuckerReduction
		(List<Point> Points, Double Tolerance)
	{
		if (Points == null || Points.Count < 3)
			return Points;
		
		Int32 firstPoint = 0;
		Int32 lastPoint = Points.Count - 1;
		List<Int32> pointIndexsToKeep = new List<Int32>();
		
		//Add the first and last index to the keepers
		pointIndexsToKeep.Add(firstPoint);
		pointIndexsToKeep.Add(lastPoint);
		
		//The first and the last point cannot be the same
		while (Points[firstPoint].Equals(Points[lastPoint]))
		{
			lastPoint--;
		}
		
		DouglasPeuckerReduction(Points, firstPoint, lastPoint, 
		                        Tolerance, ref pointIndexsToKeep);
		
		List<Point> returnPoints = new List<Point>();
		pointIndexsToKeep.Sort();
		foreach (Int32 index in pointIndexsToKeep)
		{
			returnPoints.Add(Points[index]);
		}
		
		return returnPoints;
	}

	private static void DouglasPeuckerReduction(List<Point> 
	                                            points, Int32 firstPoint, Int32 lastPoint, Double tolerance, 
	                                            ref List<Int32> pointIndexsToKeep)
	{
		Double maxDistance = 0;
		Int32 indexFarthest = 0;
		
		for (Int32 index = firstPoint; index < lastPoint; index++)
		{
			Double distance = PerpendicularDistance
				(points[firstPoint], points[lastPoint], points[index]);
			if (distance > maxDistance)
			{
				maxDistance = distance;
				indexFarthest = index;
			}
		}
		
		if (maxDistance > tolerance && indexFarthest != 0)
		{
			//Add the largest point that exceeds the tolerance
			pointIndexsToKeep.Add(indexFarthest);
			
			DouglasPeuckerReduction(points, firstPoint, 
			                        indexFarthest, tolerance, ref pointIndexsToKeep);
			DouglasPeuckerReduction(points, indexFarthest, 
			                        lastPoint, tolerance, ref pointIndexsToKeep);
		}
	}

	public static Double PerpendicularDistance
		(Point Point1, Point Point2, Point Point)
	{
		//Area = |(1/2)(x1y2 + x2y3 + x3y1 - x2y1 - x3y2 - x1y3)|   *Area of triangle
		//Base = v((x1-x2)²+(x1-x2)²)                               *Base of Triangle*
		//Area = .5*Base*H                                          *Solve for height
		//Height = Area/.5/Base
		
		Double area = Math.Abs(.5 * (Point1.X * Point2.Y + Point2.X * 
		                             Point.Y + Point.X * Point1.Y - Point2.X * Point1.Y - Point.X * 
		                             Point2.Y - Point1.X * Point.Y));
		Double bottom = Math.Sqrt(Math.Pow(Point1.X - Point2.X, 2) + 
		                          Math.Pow(Point1.Y - Point2.Y, 2));
		Double height = area / bottom * 2;
		
		return height;
		
		//Another option
		//Double A = Point.X - Point1.X;
		//Double B = Point.Y - Point1.Y;
		//Double C = Point2.X - Point1.X;
		//Double D = Point2.Y - Point1.Y;
		
		//Double dot = A * C + B * D;
		//Double len_sq = C * C + D * D;
		//Double param = dot / len_sq;
		
		//Double xx, yy;
		
		//if (param < 0)
		//{
		//    xx = Point1.X;
		//    yy = Point1.Y;
		//}
		//else if (param > 1)
		//{
		//    xx = Point2.X;
		//    yy = Point2.Y;
		//}
		//else
		//{
		//    xx = Point1.X + param * C;
		//    yy = Point1.Y + param * D;
		//}
		
		//Double d = DistanceBetweenOn2DPlane(Point, new Point(xx, yy));
	}



}
