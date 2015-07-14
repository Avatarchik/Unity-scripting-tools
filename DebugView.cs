using UnityEngine;
using System.Collections;
using System.Drawing;

public class DebugView  {

	public static Texture2D onePixel = new Texture2D(1,1);
	public static Texture2D oneLine = new Texture2D(1,1);

	public static DebugView instance;

	public DebugView()
	{

	}

	public static void init()
	{
		instance = new DebugView ();
	}

	public static void setPixel(Rect r, Color c)
	{
		onePixel.SetPixel(0,0,c);
		onePixel.Apply();
		GUI.skin.box.normal.background = onePixel;
		GUI.Box(r, GUIContent.none);
	}
	public static void drawLine(Point begin, Point end, Color c)
	{
		int width = begin.X - end.X;
		int height = begin.Y - end.Y;

		if (oneLine.width != width || oneLine.height != height)
			oneLine.Resize (width, height);

		GlobalFunctions.DrawLine (oneLine, begin, end, c);
		oneLine.Apply ();

		GUI.skin.box.normal.background = oneLine;

		GUI.Box(new Rect(begin.X,begin.Y,width,height), GUIContent.none);
	}

	public static void drawRectangle(Rect r, Color c)
	{
		onePixel.SetPixel(0,0,c);
		onePixel.Apply();
		GUI.skin.box.normal.background = onePixel;
		GUI.Box(r, GUIContent.none);
	}
}
