using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;


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
			Simulation Sim = new Simulation();
			Sim.Start(args);
		}
	}
	#endregion
}
