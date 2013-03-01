using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CsOde;

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

	/// <summary>
	/// ODE上の車クラス
	/// </summary>
	public class Car
	{
		#region const value
		/// <summary>開始時の車体のX位置</summary>
		public readonly Real STARTX;
		/// <summary>開始時の車体のY位置</summary>
		public readonly Real STARTY;
		/// <summary>開始時の車体の中心高さ</summary>
		public const Real STARTZ = 0.5f;
		/// <summary>車体の長さ</summary>
		public const Real LENGTH = 0.7f;
		/// <summary>車体の幅</summary>
		public const Real WIDTH = 0.5f;
		/// <summary>車体の高さ</summary>
		public const Real HEIGHT = 0.8f;
		/// <summary>後輪の半径</summary>
		public const Real RADIUS_REAR = 0.18f;
		/// <summary>後輪の厚み</summary>
		public const Real THICKNESS_REAR = 0.08f;
		/// <summary>前輪の半径</summary>
		public const Real RADIUS_FRONT = 0.10f;
		/// <summary>前輪の厚み</summary>
		public const Real THICKNESS_FRONT = 0.08f;

		/// <summary>シミュレーション周期</summary>
		private readonly Real _simInterval;
		#endregion

		#region fields
		/// <summary>ODE - ワールドID</summary>
		private WorldID _world;
		/// <summary>ODE - スペースID</summary>
		private SpaceID _space;

		/// <summary>車体</summary>
		private ODE.Object.Box _carBody;
		/// <summary>URG</summary>
		private ODE.Object.Box _urg;
		/// <summary>車輪</summary>
		private ODE.Object.Cylinder[] _wheel = new ODE.Object.Cylinder[4];

		/// <summary>車体中心の速度</summary>
		private Real _v = 0.0f;
		/// <summary>車体中心の角速度</summary>
		private Real _omega = 0.0f;
		/// <summary>車体中心の旋回半径</summary>
		private Real _rho = 0.0f;
		/// <summary>車体中心の回転角</summary>
		private Real _theta = 0.0f;

		/// <summary>車輪の回転速度</summary>
		private Real _speedR = 0.0f;
		private Real _speedL = 0.0f;
		/// <summary>車輪の回転角度</summary>
		private Real _angleR = 0.0f;
		private Real _angleL = 0.0f;
		#endregion

		#region property
		public Real speedL
		{
			set { _speedL = value; }
			get { return _speedL; }
		}
		public Real speedR
		{
			set { _speedR = value; }
			get { return _speedR; }
		}
		public Real v
		{
			set { _v = value; }
			get { return _v; }
		}
		public Real omega
		{
			set { _omega = value; }
			get { return _omega; }
		}
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="world">ODE - ワールドID</param>
		/// <param name="space">ODE - スペースID</param>
		public Car(ref WorldID world, ref SpaceID space, Real simInterval, Real startX, Real startY)
		{
			_world = world;
			_space = space;
			_simInterval = simInterval;

			STARTX = startX;
			STARTY = startY;

			CreateCar();
		}

		#region private method
		/// <summary>
		/// 車の生成
		/// </summary>
		private void CreateCar()
		{
			// 車体
			_carBody = new ODE.Object.Box(
				_world, _space,
				STARTX, STARTY, STARTZ,
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
				STARTX, STARTY + LENGTH / 2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
				0.0f, 0.0f, 0.0f,
				0.0f,
				STARTX, STARTY + LENGTH / 2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
				1.0f, 0.0f, 0.0f,
				0.1f,
				0.1f, 0.1f, 0.1f
			);
			_urg.CreateJoint(Ode.JointType.Hinge, _carBody);

			// 後左輪
			_wheel[0] = new ODE.Object.Cylinder(
				_world, _space,
				STARTX - WIDTH / 2 - THICKNESS_REAR / 2, STARTY - LENGTH / 2, RADIUS_REAR,
				0.0f, 1.0f, 0.0f,
				Ode.PI / 2,
				STARTX + WIDTH / 2 + THICKNESS_REAR / 2, STARTY - LENGTH / 2, RADIUS_REAR,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_REAR,
				THICKNESS_REAR
			);

			// 後右輪
			_wheel[1] = new ODE.Object.Cylinder(
				_world, _space,
				STARTX + WIDTH / 2 + THICKNESS_REAR / 2, STARTY - LENGTH / 2, RADIUS_REAR,
				0.0f, 1.0f, 0.0f,
				Ode.PI / 2,
				STARTX - WIDTH / 2 - THICKNESS_REAR / 2, STARTY - LENGTH / 2, RADIUS_REAR,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_REAR,
				THICKNESS_REAR
			);

			// 前左輪
			_wheel[2] = new ODE.Object.Cylinder(
				_world, _space,
				STARTX - WIDTH / 2 - THICKNESS_FRONT / 2, STARTY + LENGTH / 2, RADIUS_FRONT,
				0.0f, 1.0f, 0.0f,
				Ode.PI / 2,
				STARTX - WIDTH / 2 - THICKNESS_FRONT / 2, STARTY + LENGTH / 2, RADIUS_FRONT,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_FRONT,
				THICKNESS_FRONT
			);

			// 全右輪
			_wheel[3] = new ODE.Object.Cylinder(
				_world, _space,
				STARTX + WIDTH / 2 + THICKNESS_FRONT / 2, STARTY + LENGTH / 2, RADIUS_FRONT,
				0.0f, 1.0f, 0.0f,
				Ode.PI / 2,
				STARTX + WIDTH / 2 + THICKNESS_FRONT / 2, STARTY + LENGTH / 2, RADIUS_FRONT,
				1.0f, 0.0f, 0.0f,
				0.2f,
				RADIUS_FRONT,
				THICKNESS_FRONT
			);
			_wheel[0].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[1].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[2].CreateJoint(Ode.JointType.Hinge, _carBody);
			_wheel[3].CreateJoint(Ode.JointType.Hinge, _carBody);

			//// URG
			//_urg = new ODE.Object.Box(
			//    _world, _space,
			//    0.0f, LENGTH / 2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
			//    0.0f, 0.0f, 0.0f,
			//    0.0f,
			//    0.0f, LENGTH / 2 - 0.1f, STARTZ + HEIGHT / 2 + 0.05f,
			//    1.0f, 0.0f, 0.0f,
			//    0.1f,
			//    0.1f, 0.1f, 0.1f
			//);
			//_urg.CreateJoint(Ode.JointType.Hinge, _carBody);

			//// 後左輪
			//_wheel[0] = new ODE.Object.Cylinder(
			//    _world, _space,
			//    -WIDTH / 2 - THICKNESS_REAR / 2, -LENGTH / 2, RADIUS_REAR,
			//    0.0f, 1.0f, 0.0f,
			//    Ode.PI / 2,
			//    WIDTH / 2 + THICKNESS_REAR / 2, -LENGTH / 2, RADIUS_REAR,
			//    1.0f, 0.0f, 0.0f,
			//    0.2f,
			//    RADIUS_REAR,
			//    THICKNESS_REAR
			//);

			//// 後右輪
			//_wheel[1] = new ODE.Object.Cylinder(
			//    _world, _space,
			//    WIDTH / 2 + THICKNESS_REAR / 2, -LENGTH / 2, RADIUS_REAR,
			//    0.0f, 1.0f, 0.0f,
			//    Ode.PI / 2,
			//    -WIDTH / 2 - THICKNESS_REAR / 2, -LENGTH / 2, RADIUS_REAR,
			//    1.0f, 0.0f, 0.0f,
			//    0.2f,
			//    RADIUS_REAR,
			//    THICKNESS_REAR
			//);

			//// 前左輪
			//_wheel[2] = new ODE.Object.Cylinder(
			//    _world, _space,
			//    -WIDTH / 2 - THICKNESS_FRONT / 2, LENGTH / 2, RADIUS_FRONT,
			//    0.0f, 1.0f, 0.0f,
			//    Ode.PI / 2,
			//    -WIDTH / 2 - THICKNESS_FRONT / 2, LENGTH / 2, RADIUS_FRONT,
			//    1.0f, 0.0f, 0.0f,
			//    0.2f,
			//    RADIUS_FRONT,
			//    THICKNESS_FRONT
			//);

			//// 全右輪
			//_wheel[3] = new ODE.Object.Cylinder(
			//    _world, _space,
			//    WIDTH / 2 + THICKNESS_FRONT / 2, LENGTH / 2, RADIUS_FRONT,
			//    0.0f, 1.0f, 0.0f,
			//    Ode.PI / 2,
			//    WIDTH / 2 + THICKNESS_FRONT / 2, LENGTH / 2, RADIUS_FRONT,
			//    1.0f, 0.0f, 0.0f,
			//    0.2f,
			//    RADIUS_FRONT,
			//    THICKNESS_FRONT
			//);
			//_wheel[0].CreateJoint(Ode.JointType.Hinge, _carBody);
			//_wheel[1].CreateJoint(Ode.JointType.Hinge, _carBody);
			//_wheel[2].CreateJoint(Ode.JointType.Hinge, _carBody);
			//_wheel[3].CreateJoint(Ode.JointType.Hinge, _carBody);
		}

		/// <summary>
		/// エンコーダ
		/// </summary>
		/// <param name="angleL"></param>
		/// <param name="angleR"></param>
		/// <param name="omegaL"></param>
		/// <param name="omegaR"></param>
		private void Encorder(out Real angleL, out Real angleR, out Real omegaL, out Real omegaR)
		{
			angleL = _wheel[0].GetJointAnglePosition();
			angleR = _wheel[1].GetJointAnglePosition();

			if (angleL - _angleL >= 1.9f*Ode.PI)
			{
				omegaL = ((angleL - _angleL) - 2.0f * Ode.PI) / _simInterval;
			}
			else if (angleL - _angleL <= -1.9f * Ode.PI)
			{
				omegaL = ((angleL - _angleL) + 2.0f * Ode.PI) / _simInterval;
			}
			else
			{
				omegaL = (angleL - _angleL) / _simInterval;
			}
			if (angleR - _angleR >= 1.9f * Ode.PI)
			{
				omegaR = ((angleR - _angleR) - 2.0f * Ode.PI) / _simInterval;
			}
			else if (angleR - _angleR <= -1.9f * Ode.PI)
			{
				omegaR = ((angleR - _angleR) + 2.0f * Ode.PI) / _simInterval;
			}
			else
			{
				omegaR = (angleR - _angleR) / _simInterval;
			}

			_angleL = angleL;
			_angleR = angleR;
		}
		#endregion

		#region public method
		/// <summary>
		/// Drawstuffによる描画
		/// </summary>
		public void DrawByDrawstuff()
		{
			_carBody.DrawByDrawstuff(1.0f, 1.0f, 1.0f);
			_urg.DrawByDrawstuff(0.0f, 0.0f, 1.0f);
			for (int i = 0; i < 4; i++)
			{
				_wheel[i].DrawByDrawstuff(0.0f, 0.0f, 0.0f);
			}
		}

		/// <summary>
		/// 位置行列・回転行列を取得
		/// </summary>
		/// <param name="pos">位置行列</param>
		/// <param name="R">回転行列</param>
		public void GetStatus(out Ode.Vector3 pos, out Ode.Matrix3 R)
		{
			pos = _carBody.GetBodyPosition();
			R = _carBody.GetBodyRotation();
		}

		/// <summary>
		/// 車の制御
		/// </summary>
		public void Control()
		{
			Real fmax = 100f;

			//_speedL = _v + (_omega * WIDTH / 2.0f);
			//_speedR = _v - (_omega * WIDTH / 2.0f);

			Real angleL, angleR, omegaL, omegaR;
			Encorder(out angleL, out angleR, out omegaL, out omegaR);
			//Console.WriteLine(omegaL.ToString() + ", " + omegaR.ToString());

			Real vL = RADIUS_REAR * omegaL;
			Real vR = RADIUS_REAR * omegaR;
			//Console.WriteLine(vL.ToString() + ", " + vR.ToString());

			_v = (vR + vL) / 2.0f;
			_omega = (vR - vL) / WIDTH;
			_rho = (WIDTH / 2.0f) * (vR + vL) / (vR - vL);

			_theta += _omega * _simInterval; 

			_wheel[0].SetJointParam(Ode.JointParam.Vel, _speedL);
			_wheel[0].SetJointParam(Ode.JointParam.FMax, fmax);
			_wheel[1].SetJointParam(Ode.JointParam.Vel, _speedR);
			_wheel[1].SetJointParam(Ode.JointParam.FMax, fmax);

			Console.WriteLine("[v, omega, rho, theta] = ["  + _v.ToString() + ", " +_omega.ToString() + ", " + _rho.ToString() + ", " + _theta.ToString()+ "]");
		}
		#endregion
	}
}
