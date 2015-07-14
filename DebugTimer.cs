#if UNITY_EDITOR
#define SHOW_DEBUG_TIME
#endif
using UnityEngine;
using System.Collections;
using System.Diagnostics;

public class DebugTimer {

	private static Hashtable beginTimer = new Hashtable();


	// Use this for initialization
	public static void StartMicros (string name)
	{
#if SHOW_DEBUG_TIME
		Stopwatch sw = new Stopwatch();
		sw.Start();
		beginTimer.Add (name, sw);
#endif
	}


	// Use this for initialization
	public static void StartMillis (string name)
	{
#if SHOW_DEBUG_TIME
		Stopwatch sw = new Stopwatch();
		sw.Start();
		beginTimer.Add (name, sw);
#endif
	}
	
	public static void StopMillis(string name)
	{
#if SHOW_DEBUG_TIME
		Stopwatch sw = (Stopwatch) beginTimer [name];
		sw.Stop();
		
		long microseconds = sw.ElapsedTicks / (Stopwatch.Frequency / (1000L));
		UnityEngine.Debug.Log ("elapsed time " + name + " : " + microseconds.ToString());
		beginTimer.Remove (name);
#endif
	}



	public static void StopMicros(string name)
	{
#if SHOW_DEBUG_TIME
		Stopwatch sw = (Stopwatch) beginTimer [name];
		sw.Stop();
		
		long microseconds = sw.ElapsedTicks / (Stopwatch.Frequency / (1000L*1000L));
		UnityEngine.Debug.Log ("elapsed time " + name + " : " + microseconds.ToString());
		beginTimer.Remove (name);
#endif
	}


}
