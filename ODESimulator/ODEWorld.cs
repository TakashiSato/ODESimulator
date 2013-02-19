using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
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

	public class ODEWorld
	{
		/// <summary>ODE - ワールドID</summary>
		private WorldID world = WorldID.Zero;
		/// <summary>ODE - スペースID</summary>
		private SpaceID space = SpaceID.Zero;
		/// <summary>ODE - 接触ジョイントグループID</summary>
		private JointGroupID contactGroup = JointGroupID.Zero;

		/// <summary>地面ジオメトリ</summary>
		private GeomID ground = GeomID.Zero;
		/// <summary>球体ボディ</summary>
		private BodyID ball = BodyID.Zero;
		/// <summary>球体ジオメトリ</summary>
		private GeomID ballGeom = GeomID.Zero;

		/// <summary>Drawstuff Start Function Callback</summary>
		Ds.CallbackFunction StartCallback = null;
		/// <summary>Drawstuff Step Function Callback</summary>
		Ds.CallbackFunction StepCallback = null;
		/// <summary>Drawstuff Command Function Callback</summary>
		Ds.CallbackFunction CommandCallback = null;
		/// <summary>Drawstuff Stop Function Callback</summary>
		Ds.CallbackFunction StopCallback = null;
		/// <summary>衝突判定コールバック</summary>
		Ode.NearCallback CollisionCallback = null;


		/// <summary>
		/// 基本的な初期化処理
		/// </summary>
		public void Initialize()
		{
			/* シンプルなODE初期化コード
			 *  - シミュレーション空間の生成
			 *  - 重力の設定
			 *  - 衝突判定空間の誠意製
			 *  - 接触ジョイントグループ
			 *  - 衝突判定コールバックデリゲート
			 */
			Ode.InitODE();
			world = Ode.WorldCreate();
			Ode.WorldSetGravity(world, 0.0f, 0.0f, -0.098f);
			space = Ode.HashSpaceCreate(SpaceID.Zero);
			contactGroup = Ode.JointGroupCreate(0);

			// コールバック関数を設定
			StartCallback = new Ds.CallbackFunction(StartFunctionCallback);
			StepCallback = new Ds.CallbackFunction(StepFunctionCallback);
			CommandCallback = new Ds.CallbackFunction(CommandFunctionCallback);
			StopCallback = new Ds.CallbackFunction(StopFunctionCallback);
			CollisionCallback = new Ode.NearCallback(DefaultCollisionCallback);

			// 地面
			ground = Ode.CreatePlane(space, 0, 0, 1, 0);

			Ode.Mass m;
			ball = Ode.BodyCreate(world);		// 球を作る
			Ode.MassSetSphereTotal(out m, 1.0f, 0.2f);
			Ode.BodySetMass(ball, ref m);
			ballGeom = Ode.CreateSphere(space, 0.2f);
			Ode.GeomSetBody(ballGeom, ball);

			// 初期姿勢の設定
			Ode.BodySetPosition(ball, 0.0f, 0.0f, 2.0f);
			//base.Initialize();
		}

		/// <summary>
		/// Drawstuffの前処理コールバック関数
		/// <param name="dummy">ダミー引数</param>
		/// </summary>
		public void StartFunctionCallback(IntPtr dummy)
		{
			Ode.Vector3 viewPos = new Ode.Vector3();		// 視点の位置(x, y, z)
			viewPos.X = 3.0f;
			viewPos.Y = 0.0f;
			viewPos.Z = 1.0f;
			Ode.Vector3 viewDir = new Ode.Vector3();		// 視点の方向(ヘッド，ピッチ，ロール)
			viewDir.X = -180.0f;
			viewDir.Y = 0.0f;
			viewDir.Z = 0.0f;

			Ds.SetViewpoint(ref viewPos, ref viewDir);		// 視点の設定
		}

		/// <summary>
		/// Drawstuffのシミュレーションループコールバック関数
		/// <param name="pause"></param>
		/// </summary>
		public void StepFunctionCallback(IntPtr pause)
		{
			Ode.Vector3 pos;				// 位置行列
			Ode.Matrix3 R;					// 回転行列

			Ode.SpaceCollide(space, IntPtr.Zero, CollisionCallback);
			Ode.WorldStep(world, 0.01f);	// シミュレーションを1ステップ進める(第2引数は進める時間[秒])
			Ode.JointGroupEmpty(contactGroup);

			//Ds.SetTexture(Ds.Texture.Wood);
			Ds.SetColor(0.0f, 0.0f, 1.0f);			// 色の設定(R,G,B) [0,1]で設定
			Ode.BodyCopyPosition(ball, out pos);	// 位置を取得
			Ode.BodyCopyRotation(ball, out R);		// 姿勢を取得
			Ds.DrawSphere(ref pos, ref R, 0.2f);	// 描画
		}

		/// <summary>
		/// Drawstuffのコマンド入力時コールバック関数
		/// <param name="cmd">コマンド入力</param>
		/// </summary>
		public void CommandFunctionCallback(IntPtr cmd)
		{
			switch ((char)cmd)
			{
				case 'a':
					Ode.BodySetPosition(ball, 0.0f, 0.0f, 2.0f);
					break;
			}
		}

		/// <summary>
		/// Drawstuffの終了時コールバック関数
		/// <param name="dummy">ダミー引数</param>
		/// </summary>
		public void StopFunctionCallback(IntPtr dummy)
		{
			// 世界の破壊
			Ode.JointGroupDestroy(contactGroup);
			Ode.SpaceDestroy(space);
			Ode.WorldDestroy(world);

			contactGroup = JointGroupID.Zero;
			space = SpaceID.Zero;
			world = WorldID.Zero;

			// ODEの終了
			Ode.CloseODE();
		}

		/// <summary>
		/// 衝突判定コールバック関数
		/// </summary>
		/// <param name="data">dSpaceCollide関数へ渡したデータ</param>
		/// <param name="geom0">ジオメトリ0</param>
		/// <param name="geom1">ジオメトリ1</param>
		public void DefaultCollisionCallback(IntPtr data, GeomID geom0, GeomID geom1)
		{
			const int N = 20;
			//Ode.Contact[] contact = new Ode.Contact[N];
			Ode.ContactGeom[] contacts = new Ode.ContactGeom[N];

			BodyID b0 = Ode.GeomGetBody(geom0);
			BodyID b1 = Ode.GeomGetBody(geom1);

			if (b0 != BodyID.Zero && b1 != BodyID.Zero && Ode.AreConnected(b0, b1) == true)
				return;
			if (b0 != BodyID.Zero && b1 != BodyID.Zero && Ode.AreConnectedExcluding(b0, b1, Ode.JointType.Contact) == true)
				return;

			Ode.Contact contact = new Ode.Contact();
			contact.surface.mode = Ode.ContactFlags.Bounce;
			contact.surface.mu = Real.MaxValue;
			contact.surface.mu2 = Real.MaxValue;
			contact.surface.bounce = 0.8f;		// 反発係数[0,1]
			contact.surface.bounce_vel = 0.01f;	// 反発に必要な最低速度

			int n = Ode.Collide(geom0, geom1, N, contacts, Marshal.SizeOf(contacts[0]));
			for (int i = 0; i < n; ++i)
			{
				contact.geom = contacts[i];
				JointID c = Ode.JointCreateContact(world, contactGroup, ref contact);
				Ode.JointAttach(c, b0, b1);
			}
		}
	}
}
