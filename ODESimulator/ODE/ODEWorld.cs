using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Linq;
using System.Text;
using CsOde;
using CsDrawstuff;

namespace ODESimulator.ODE
{
	#region typedef
	using WorldID = IntPtr;
	using SpaceID = IntPtr;
	using BodyID = IntPtr;
	using GeomID = IntPtr;
	using JointID = IntPtr;
	using JointGroupID = IntPtr;
	using HeightfieldDataID = IntPtr;
	using TriMeshDataID = IntPtr;
	using Real = Single;
	#endregion

	/// <summary>
	/// ODE Simulation World
	/// </summary>
	public abstract class World
	{
		#region fields
		/// <summary>ODE - ワールドID</summary>
		protected WorldID _world = WorldID.Zero;
		/// <summary>ODE - スペースID</summary>
		protected SpaceID _space = SpaceID.Zero;
		/// <summary>地面ジオメトリ</summary>
		protected GeomID _ground = GeomID.Zero;
		/// <summary>ODE - 接触ジョイントグループID</summary>
		protected JointGroupID _contactGroup = JointGroupID.Zero;

		/// <summary>Drawstuff Start Function Callback</summary>
		protected Ds.CallbackFunction _StartCallback = null;
		/// <summary>Drawstuff Step Function Callback</summary>
		protected Ds.CallbackFunction _StepCallback = null;
		/// <summary>Drawstuff Command Function Callback</summary>
		protected Ds.CallbackFunction _CommandCallback = null;
		/// <summary>Drawstuff Stop Function Callback</summary>
		protected Ds.CallbackFunction _StopCallback = null;
		/// <summary>衝突判定コールバック</summary>
		protected Ode.NearCallback _CollisionCallback = null;

		/// <summary>Drawstuffの基幹関数</summary>
		protected Ds.Functions _dsFunc;
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		public World(Real gravity = -9.8f, Real erp = 0.2f, Real cfm = 1e-3f)
		{
			Ode.InitODE();										// ODEの初期化
			_world = Ode.WorldCreate();							// 世界の生成
			_space = Ode.HashSpaceCreate(SpaceID.Zero);			// 空間の生成
			_ground = Ode.CreatePlane(_space, 0, 0, 1, 0);		// 地面の生成
			_contactGroup = Ode.JointGroupCreate(0);			// 接触用ジョイントグループの生成


			Ode.WorldSetGravity(_world, 0.0f, 0.0f, gravity);	// 重力の設定
			Ode.WorldSetERP(_world, erp);						// ジョイント誤差修正パラメータ(推奨値は0.1~0.8)
			Ode.WorldSetCFM(_world, cfm);						// 拘束力混合パラメータ(0:ハード拘束，大きくなるほどソフト拘束)

			// 衝突判定コールバック関数を設定
			_CollisionCallback = new Ode.NearCallback(DefaultCollisionCallback);
		}

		/// <summary>
		/// Destructor
		/// </summary>
		~World()
		{
			// 世界の破壊
			Ode.JointGroupDestroy(_contactGroup);
			Ode.SpaceDestroy(_space);
			Ode.WorldDestroy(_world);

			_contactGroup = JointGroupID.Zero;
			_space = SpaceID.Zero;
			_world = WorldID.Zero;

			// ODEの終了
			Ode.CloseODE();

			Console.WriteLine("END of ODE World");
		}

		/// <summary>
		/// 衝突判定コールバック関数
		/// </summary>
		/// <param name="data">dSpaceCollide関数へ渡したデータ</param>
		/// <param name="geom0">ジオメトリ0</param>
		/// <param name="geom1">ジオメトリ1</param>
		private void DefaultCollisionCallback(IntPtr data, GeomID geom0, GeomID geom1)
		{
			const int N = 10;
			//Ode.Contact[] contact = new Ode.Contact[N];
			Ode.ContactGeom[] contacts = new Ode.ContactGeom[N];

			BodyID b0 = Ode.GeomGetBody(geom0);
			BodyID b1 = Ode.GeomGetBody(geom1);

			if (b0 != BodyID.Zero && b1 != BodyID.Zero && Ode.AreConnected(b0, b1) == true)
				return;
			if (b0 != BodyID.Zero && b1 != BodyID.Zero && Ode.AreConnectedExcluding(b0, b1, Ode.JointType.Contact) == true)
				return;

			Ode.Contact contact = new Ode.Contact();
			//contact.surface.mode = Ode.ContactFlags.Bounce;
			//contact.surface.mu = Real.MaxValue;
			//contact.surface.mu2 = Real.MaxValue;
			//contact.surface.bounce = 0.5f;		// 反発係数[0,1]
			//contact.surface.bounce_vel = 0.01f;	// 反発に必要な最低速度
			contact.surface.mode = Ode.ContactFlags.Slip1 | Ode.ContactFlags.Slip2 |
			Ode.ContactFlags.SoftERP | Ode.ContactFlags.SoftCFM | Ode.ContactFlags.Approx1;
			contact.surface.mu = Real.MaxValue;
			contact.surface.slip1 = 0.1f;
			contact.surface.slip2 = 0.1f;
			contact.surface.soft_erp = 0.8f;
			contact.surface.soft_cfm = 1e-5f;

			int n = Ode.Collide(geom0, geom1, N, contacts, Marshal.SizeOf(contacts[0]));
			for (int i = 0; i < n; ++i)
			{
				contact.geom = contacts[i];
				JointID c = Ode.JointCreateContact(_world, _contactGroup, ref contact);
				Ode.JointAttach(c, b0, b1);
			}
		}
	}
}
