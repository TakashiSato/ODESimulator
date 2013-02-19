using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Linq;
using System.Text;
using CsOde;
using CsDrawstuff;

namespace ODESimulator
{
	#region entry point
	/// <summary>
	/// entry point
	/// </summary>
	static class Program
	{
		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		static void Main(string[] args)
		{
			ODEWorld Sim = new ODEWorld();

			Ds.Functions fn = new Ds.Functions();
			fn.version = Ds.VERSION;
			fn.start = Sim.StartFunctionCallback;
			fn.step = Sim.StepFunctionCallback;
			fn.command = Sim.CommandFunctionCallback;
			fn.stop = Sim.StopFunctionCallback;
			fn.path_to_textures = "../../drawstuff/textures";

			Sim.Initialize();

			Ds.SimulationLoop(0, args, 352, 288, ref fn);
		}
	}
	#endregion
}
