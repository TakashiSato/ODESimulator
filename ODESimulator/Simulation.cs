using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CsOde;
using CsDrawstuff;

namespace ODESimulator
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

	public class Simulation : ODE.World
	{
		#region fields
		/// <summary>車体</summary>
		private ODE.Object.Box _carBody;
		/// <summary>URG</summary>
		private ODE.Object.Box _urg;
		/// <summary>車輪</summary>
		private ODE.Object.Cylinder[] _wheel = new ODE.Object.Cylinder[4];

		/// <summary>シミューレション周期(秒)</summary>
		private Real _simInterval = 0.01f;

		/// <summary>車輪の回転速度</summary>
		private Real _speedR = 0.0f;
		private Real _speedL = 0.0f;
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		public Simulation()
		{
			// コールバック関数を設定
			_StartCallback = new Ds.CallbackFunction(StartFunctionCallback);
			_StepCallback = new Ds.CallbackFunction(StepFunctionCallback);
			_CommandCallback = new Ds.CallbackFunction(CommandFunctionCallback);
			_StopCallback = new Ds.CallbackFunction(StopFunctionCallback);

			// Drawstuff基幹関数の設定
			_dsFunc = new Ds.Functions();
			_dsFunc.version = Ds.VERSION;
			_dsFunc.start = StartFunctionCallback;
			_dsFunc.step = StepFunctionCallback;
			_dsFunc.command = CommandFunctionCallback;
			_dsFunc.stop = StopFunctionCallback;
			_dsFunc.path_to_textures = "../../ODE/Drawstuff/textures";

			// シミューレションの準備をする
			PrepSimulation();
		}

		/// <summary>
		/// Destructor
		/// </summary>
		~Simulation()
		{
		}

		#region private method
		/// <summary>
		/// シミューレションの準備をする
		/// </summary>
		private void PrepSimulation()
		{
			CreateCar();
		}

		/// <summary>
		/// Drawstuffの初期視点設定
		/// </summary>
		private void InitializeViewpoint()
		{
			//Ode.Vector3 viewPos = new Ode.Vector3();		// 視点の位置(x, y, z)
			//viewPos.X = 0.0f;
			//viewPos.Y = 3.0f;
			//viewPos.Z = 1.0f;
			//Ode.Vector3 viewDir = new Ode.Vector3();		// 視点の方向(ヘッド，ピッチ，ロール)
			//viewDir.X = -90.0f;
			//viewDir.Y = 0.0f;
			//viewDir.Z = 0.0f;
			Ode.Vector3 viewPos = new Ode.Vector3();		// 視点の位置(x, y, z)
			viewPos.X = 0.0f;
			viewPos.Y = 0.0f;
			viewPos.Z = 10.0f;
			Ode.Vector3 viewDir = new Ode.Vector3();		// 視点の方向(ヘッド，ピッチ，ロール)
			viewDir.X = 90.0f;
			viewDir.Y = -90.0f;
			viewDir.Z = 0.0f;

			Ds.SetViewpoint(ref viewPos, ref viewDir);		// 視点の設定
		}

		/// <summary>
		/// 車の生成
		/// </summary>
		private void CreateCar()
		{
			Real PI = (float)Math.PI;
			Real STARTZ = 0.5f;		// 車体の中心高さ
			Real LENGTH = 0.7f;		// 車体の長さ
			Real WIDTH = 0.5f;		// 車体の幅
			Real HEIGHT = 0.8f;		// 車体の高さ
			Real RADIUS_R = 0.18f;	// 後輪の半径
			Real THICKNESS_R = 0.08f;// 後輪の厚み
			Real RADIUS_L = 0.10f;	// 前輪の半径
			Real THICKNESS_L = 0.08f;// 前輪の厚み

			// 車体
			_carBody = new ODE.Object.Box(
				_world, _space,
				0.0f, 0.0f, STARTZ,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				9.0f,
				WIDTH, LENGTH, HEIGHT
			);

			// URG
			_urg = new ODE.Object.Box(
				_world, _space,
				0.0f, LENGTH/2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, LENGTH / 2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
				1.0f, 0.0f, 0.0f,
				0.1f,
				0.1f, 0.1f, 0.1f
			);
			_urg.CreateJoint(Ode.JointType.Hinge, _carBody);

			// 後左輪
			_wheel[0] = new ODE.Object.Cylinder(
				_world, _space,
				-WIDTH / 2 - THICKNESS_R / 2, -LENGTH / 2, RADIUS_R,
				0.0f, 1.0f, 0.0f,
				PI/2,
				WIDTH / 2 + THICKNESS_R / 2, -LENGTH / 2, RADIUS_R,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_R,
				THICKNESS_R
			);

			// 後右輪
			_wheel[1] = new ODE.Object.Cylinder(
				_world, _space,
				WIDTH / 2 + THICKNESS_R / 2, -LENGTH / 2, RADIUS_R,
				0.0f, 1.0f, 0.0f,
				PI / 2,
				-WIDTH / 2 - THICKNESS_R / 2, -LENGTH / 2, RADIUS_R,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_R,
				THICKNESS_R
			);

			// 前左輪
			_wheel[2] = new ODE.Object.Cylinder(
				_world, _space,
				-WIDTH / 2 - THICKNESS_L / 2, LENGTH / 2, RADIUS_L,
				0.0f, 1.0f, 0.0f,
				PI / 2,
				-WIDTH / 2 -THICKNESS_L / 2, LENGTH / 2, RADIUS_L,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_L,
				THICKNESS_L
			);

			// 全右輪
			_wheel[3] = new ODE.Object.Cylinder(
				_world, _space,
				WIDTH / 2 + THICKNESS_L / 2, LENGTH / 2, RADIUS_L,
				0.0f, 1.0f, 0.0f,
				PI / 2,
				WIDTH / 2 + THICKNESS_L / 2, LENGTH / 2, RADIUS_L,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_L,
				THICKNESS_L
			);
			_wheel[0].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[1].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[2].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[3].CreateJoint(Ode.JointType.Hinge, _carBody);
		}

		/// <summary>
		/// シミューレション世界の描画
		/// </summary>
		private void DrawSimulationWorld()
		{
			_carBody.DrawByDrawstuff(1.0f, 1.0f, 1.0f);
			_urg.DrawByDrawstuff(0.0f, 0.0f, 1.0f);
			for (int i = 0; i < 4; i++)
			{
				_wheel[i].DrawByDrawstuff(0.0f, 0.0f, 0.0f);
			}
		}

		/// <summary>
		/// 車の制御
		/// </summary>
		private void Control()
		{
			Real fmax = 100f;

			_wheel[0].SetJointParam(Ode.JointParam.Vel, _speedL);
			_wheel[0].SetJointParam(Ode.JointParam.FMax, fmax);
			_wheel[1].SetJointParam(Ode.JointParam.Vel, _speedR);
			_wheel[1].SetJointParam(Ode.JointParam.FMax, fmax);
		}
		#endregion

		#region callback finction
		/// <summary>
		/// Drawstuffの前処理コールバック関数
		/// <param name="dummy">ダミー引数</param>
		/// </summary>
		private void StartFunctionCallback(IntPtr dummy)
		{
			InitializeViewpoint();
			Console.WriteLine("-----------------------------------------");
			Console.WriteLine("| User Command                          |");
			Console.WriteLine("-----------------------------------------");
			Console.WriteLine("| Ctrl+T : Switch Texture               |");
			Console.WriteLine("| Ctrl+S : Switch Shadow                |");
			Console.WriteLine("| Ctrl+X : End of Simulation            |");
			Console.WriteLine("| D, F   : Decel or Accel Left  Wheel |");
			Console.WriteLine("| K, J   : Decel or Accel Right Wheel |");
			Console.WriteLine("| I      : Stop                         |");
			Console.WriteLine("-----------------------------------------");
		}

		/// <summary>
		/// Drawstuffのシミュレーションループコールバック関数
		/// <param name="pause"></param>
		/// </summary>
		private void StepFunctionCallback(IntPtr pause)
		{
			Ode.SpaceCollide(_space, IntPtr.Zero, _CollisionCallback);
			Ode.WorldStep(_world, _simInterval);	// シミュレーションを1ステップ進める(第2引数は進める時間[秒])
			Ode.JointGroupEmpty(_contactGroup);

			// 描画
			DrawSimulationWorld();
			// 制御
			Control();
		}

		/// <summary>
		/// Drawstuffのコマンド入力時コールバック関数
		/// <param name="cmd">コマンド入力</param>
		/// </summary>
		private void CommandFunctionCallback(IntPtr cmd)
		{
			switch ((char)cmd)
			{
				case 'f':
					_speedL += 0.5f;
					break;
				case 'd':
					_speedL -= 0.5f;
					break;
				case 'j':
					_speedR += 0.5f;
					break;
				case 'k':
					_speedR -= 0.5f;
					break;
				case 'i':
					_speedR = _speedL =  0.0f;
					break;
			}
			Console.WriteLine("Speed: " + _speedL.ToString() + ", " + _speedR.ToString());
		}

		/// <summary>
		/// Drawstuffの終了時コールバック関数
		/// <param name="dummy">ダミー引数</param>
		/// </summary>
		private void StopFunctionCallback(IntPtr dummy)
		{
		}
		#endregion

		#region public method
		/// <summary>
		/// シミュレーションを開始する
		/// </summary>
		/// <param name="args"></param>
		public void Start(string[] args)
		{
			Ds.SimulationLoop(0, args, 640, 480, ref _dsFunc);
		}
		#endregion
	}
}
