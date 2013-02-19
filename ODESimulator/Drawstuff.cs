using System;
using System.Runtime.InteropServices;
using CsOde;

namespace CsDrawstuff
{
#if dDOUBLE
	using dReal = System.Double;
#else
	using dReal = System.Single;
#endif

	public static class Ds
	{
		public const int VERSION = 2;

		public enum Texture
		{
			None,
			Wood
		}

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void CallbackFunction(IntPtr data);

		[StructLayout(LayoutKind.Sequential)]
		public struct Functions
		{
			public int version;
			public CallbackFunction start;
			public CallbackFunction step;
			public CallbackFunction command;
			public CallbackFunction stop;
			public string path_to_textures;
		}

		[DllImport("drawstuff", EntryPoint="dsDrawBox")]
		public static extern void DrawBox(ref Ode.Vector3 pos, ref Ode.Matrix3 R, ref Ode.Vector3 sides);

		[DllImport("drawstuff", EntryPoint = "dsDrawCapsule")]
		public static extern void DrawCapsule(ref Ode.Vector3 pos, ref Ode.Matrix3 R, dReal length, dReal radius);

		[DllImport("drawstuff", EntryPoint = "dsDrawSphere")]
		public static extern void DrawSphere(ref Ode.Vector3 pos, ref Ode.Matrix3 R, dReal radius);

		[DllImport("drawstuff", EntryPoint = "dsDrawConvex")]
		public static extern void DrawConvex(ref Ode.Vector3 pos, ref Ode.Matrix3 R, dReal[] planes, int planeCount, dReal[] points, int pointCount, int[] polygons);

		[DllImport("drawstuff", EntryPoint="dsSetColor")]
		public static extern void SetColor(float red, float green, float blue);

		[DllImport("drawstuff", EntryPoint="dsSetTexture")]
		public static extern void SetTexture(Texture texture);

		[DllImport("drawstuff", EntryPoint="dsSetViewpoint")]
		public static extern void SetViewpoint(ref Ode.Vector3 xyz, ref Ode.Vector3 hpr);

		[DllImport("drawstuff", EntryPoint="dsSimulationLoop")]
		public static extern void SimulationLoop(int argc, string[] argv, int window_width, int window_height, ref Functions fn);
	}
}
