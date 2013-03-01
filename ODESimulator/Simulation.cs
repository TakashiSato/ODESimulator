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

	#region enum
	/// <summary>
	/// カメラ視点の方式
	/// </summary>
	enum ViewpointMode
	{
		Fixed,				// 固定視点
		Trace,				// 車追従視点
		FirstPerson			// 車一人称視点
	}
	#endregion

	/// <summary>
	/// Simulation Class
	/// </summary>
	public class Simulation : ODE.World
	{
		#region fields
		/// <summary>シミューレション周期(秒)</summary>
		private Real _simInterval = 0.01f;

		/// <summary>カメラ視点モード</summary>
		private ViewpointMode _viewPointMode = ViewpointMode.Trace;

		/// <summary>車クラス</summary>
		private Car _car;
		private Car _car2;
		/// <summary>壁</summary>
		private ODE.Object.Box[] _wall = new ODE.Object.Box[4];
		/// <summary>ボール</summary>
		private ODE.Object.Sphere _ball;
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
			_car = new Car(ref _world, ref _space, _simInterval, 0.0f, 0.0f);
			_car2 = new Car(ref _world, ref _space, _simInterval, 3.0f, 4.0f);
			CreateWall();
			CreateBall();
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
		/// 壁の生成
		/// </summary>
		private void CreateWall()
		{
			Real HEIGHT = 1.0f;

			// 壁
			_wall[0] = new ODE.Object.Box(
				_world, _space,
				-2.0f, 2.0f, HEIGHT/2,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				10.0f,
				3.0f, 1.0f, HEIGHT
			);
			_wall[1] = new ODE.Object.Box(
				_world, _space,
				1.0f, 8.0f, HEIGHT/2,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				10.0f,
				1.0f, 2.0f, HEIGHT
			);
			_wall[2] = new ODE.Object.Box(
				_world, _space,
				-2.0f, 12.0f, HEIGHT/2,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				10.0f,
				2.0f, 1.0f, HEIGHT
			);
			_wall[3] = new ODE.Object.Box(
				_world, _space,
				0.0f, 15.0f, HEIGHT/2,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				10.0f,
				2.0f, 1.0f, HEIGHT
			);
			for (int i = 0; i < 4; i++)
			{
				_wall[i].CreateJoint(Ode.JointType.Fixed);
			}
		}

		/// <summary>
		/// ボールの生成
		/// </summary>
		private void CreateBall()
		{
			// ボール
			_ball = new ODE.Object.Sphere(
				_world, _space,
				0.0f, 5.0f, 2.0f,
				0.0f, 0.0f, 0.0f,
				0.0f,
				0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f,
				0.1f,
				0.2f
			);
		}

		/// <summary>
		/// シミューレション世界の描画
		/// </summary>
		private void DrawSimulationWorld()
		{
			_car.DrawByDrawstuff();
			_car2.DrawByDrawstuff();

			for (int i = 0; i < 4; i++)
			{
				_wall[i].DrawByDrawstuff(0.2f, 0.2f, 0.2f);
				_ball.DrawByDrawstuff(1.0f, 0.0f, 0.0f);
			}
		}

		/// <summary>
		/// カメラ視点の更新処理
		/// </summary>
		/// <param name="vm">カメラ視点モード</param>
		private void ViewpointUpdate(ViewpointMode vm)
		{
			Ode.Vector3 viewPos = new Ode.Vector3();		// 視点の位置(x, y, z)
			Ode.Vector3 viewDir = new Ode.Vector3();		// 視点の方向(ヘッド，ピッチ，ロール)

			// 車の位置行列・回転行列を取得
			Ode.Vector3 pos; ;
			Ode.Matrix3 R;
			_car.GetStatus(out pos, out R);

			switch (vm)
			{
				case ViewpointMode.Fixed:
					break;

				case ViewpointMode.Trace:
					viewPos.X = pos.X;
					viewPos.Y = pos.Y;
					viewPos.Z = 10.0f;
					viewDir.X = 90.0f;
					viewDir.Y = -90.0f;
					viewDir.Z = 0.0f;
					Ds.SetViewpoint(ref viewPos, ref viewDir);		// 視点の設定
					break;

				case ViewpointMode.FirstPerson:
					Real yaw = (float)Math.Atan2(R.M21, R.M11) / Ode.PI * 180.0f;

					viewPos.X = pos.X;
					viewPos.Y = pos.Y;
					viewPos.Z = 0.5f;
					viewDir.X = 90.0f - yaw;
					viewDir.Y = 0.0f;
					viewDir.Z = 0.0f;
					Ds.SetViewpoint(ref viewPos, ref viewDir);		// 視点の設定
					break;
			}
		}

		/// <summary>
		/// 制御
		/// </summary>
		private void Control()
		{
			_car.Control();
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
			Console.WriteLine("-----------------------------------------");
			Console.WriteLine("| D, F   : Decel or Accel Left  Wheel   |");
			Console.WriteLine("| K, J   : Decel or Accel Right Wheel   |");
			Console.WriteLine("| S      : Stop                         |");
			Console.WriteLine("-----------------------------------------");
			Console.WriteLine("| Z, X, C: Change Viewpoint              ");
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
			// カメラ視点の更新
			ViewpointUpdate(_viewPointMode);
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
					_car.speedL += 0.5f;
					break;
				case 'd':
					_car.speedL -= 0.5f;
					break;
				case 'j':
					_car.speedR += 0.5f;
					break;
				case 'k':
					_car.speedR -= 0.5f;
					break;
				case 'u':
					_car.v += 1.0f;
					break;
				case 'i':
					_car.v -= 1.0f;
					break;
				case 'y':
					_car.omega += 0.4f;
					break;
				case 'o':
					_car.omega -= 0.4f;
					break;
				case 's':
					_car.speedR = _car.speedL = 0.0f;
					break;
				// カメラ視点：固定
				case 'z':
					_viewPointMode = ViewpointMode.Fixed;
					break;
				// カメラ視点：追従
				case 'x':
					_viewPointMode = ViewpointMode.Trace;
					break;
				// カメラ視点：一人称視点
				case 'c':
					_viewPointMode = ViewpointMode.FirstPerson;
					break;
			}
			Console.WriteLine("SpeedL: " + _car.speedL.ToString() + ", SpeedR: " + _car.speedR.ToString());
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
