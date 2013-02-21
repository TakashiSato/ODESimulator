using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CsOde;
using CsDrawstuff;

namespace ODESimulator.ODE.Object
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
	/// ODE世界の軸の種類
	/// </summary>
	public enum AxisType
	{
		X = 1,
		Y = 2,
		Z = 3
	}
	#endregion

	/// <summary>
	/// ODEのオブジェクトに共通する要素を持つ抽象クラス
	/// </summary>
	public abstract class ObjectSettings
	{
		#region fields
		/// <summary>ODE - ワールドID</summary>
		protected WorldID _worldID = WorldID.Zero;
		/// <summary>ODE - スペースID</summary>
		protected SpaceID _spaceID = SpaceID.Zero;
		/// <summary>ボディ</summary>
		protected BodyID _body = BodyID.Zero;
		/// <summary>ジオメトリ</summary>
		protected GeomID _geom = GeomID.Zero;
		/// <summary>ODE - ジョイントID</summary>
		protected SpaceID _joint = JointID.Zero;

		/// <summary>オブジェクトの位置</summary>
		protected Ode.Vector3 _pos;
		/// <summary>オブジェクトの回転軸</summary>
		protected Ode.Vector3 _rotAxis;
		/// <summary>オブジェクトの回転角</summary>
		protected Real _rotAngle;
		/// <summary>オブジェクト関節の回転中心</summary>
		protected Ode.Vector3 _jointRotCenter;
		/// <summary>オブジェクト関節の回転軸</summary>
		protected Ode.Vector3 _jointRotAxis;
		/// <summary>オブジェクトの質量</summary>
		protected Real _mass;
		/// <summary>オブジェクト関節の種類</summary>
		protected Ode.JointType _jointType;
		#endregion

		/// <summary>
		/// Destructor
		/// </summary>
		~ObjectSettings()
		{
			// ボディの破壊
			if (_body != BodyID.Zero)
			{
				Ode.BodyDestroy(_body);
			}
			_body = BodyID.Zero;

			// ジオメトリの破壊
			if (_geom != GeomID.Zero)
			{
				Ode.GeomDestroy(_geom);
			}
			_geom = GeomID.Zero;

			// ジョイントの破壊
			if (_joint != JointID.Zero)
			{
				Ode.JointDestroy(_joint);
			}
			_joint = JointID.Zero;
		}

		#region abstract method
		/// <summary>
		/// オブジェクトをDrawstuffにより描画
		/// </summary>
		/// <param name="red">描画色赤成分[0,1]</param>
		/// <param name="green">描画色緑成分[0,1]</param>
		/// <param name="blue">描画色青成分[0,1]</param>
		public abstract void DrawByDrawstuff(Real red, Real green, Real blue);

		/// <summary>
		/// オブジェクトをDrawstuffにより描画(パラメータ指定)
		/// </summary>
		/// <param name="red">描画色赤成分[0,1]</param>
		/// <param name="green">描画色緑成分[0,1]</param>
		/// <param name="blue">描画色青成分[0,1]</param>
		/// <param name="param">描画パラメータ</param>
		public abstract void DrawByDrawstuff(Real red, Real green, Real blue, Real param);
		#endregion

		#region public method
		/// <summary>
		/// オブジェクトの位置行列を取得
		/// </summary>
		/// <param name="pos">位置行列</param>
		public void GetBodyPosition(out Ode.Vector3 pos)
		{
			Ode.BodyCopyPosition(_body, out pos);
		}

		/// <summary>
		/// オブジェクトの回転行列を取得
		/// </summary>
		/// <param name="R">回転行列</param>
		public void GetBodyRotation(out Ode.Matrix3 R)
		{
			Ode.BodyCopyRotation(_body, out R);
		}

		/// <summary>
		/// オブジェクト関節を生成する
		/// <param name="type">生成する関節の種類</param>
		/// <param name="attachObject">関節を接続するオブジェクト</param>
		/// </summary>
		public void CreateJoint(Ode.JointType type, ObjectSettings attachObject = null)
		{
			switch (type)
			{
				// 固定関節
				case Ode.JointType.Fixed:
					_joint = Ode.JointCreateFixed(_worldID, JointGroupID.Zero);	// 固定関節（土台と地面の固定）
					Ode.JointAttach(_joint, _body, JointGroupID.Zero);			// 固定関節の取付け
					Ode.JointSetFixed(_joint);									// 固定関節の設定
					_jointType = Ode.JointType.Fixed;
					break;

				// ヒンジ関節
				case Ode.JointType.Hinge:
					_joint = Ode.JointCreateHinge(_worldID, JointGroupID.Zero);	// ヒンジ関節生成
					Ode.JointAttach(_joint, attachObject._body , _body);		// 関節を取り付ける
					Ode.JointSetHingeAnchor(_joint, _jointRotCenter.X, _jointRotCenter.Y, _jointRotCenter.Z);	// 関節中心の設定
					Ode.JointSetHingeAxis(_joint, _jointRotAxis.X, _jointRotAxis.Y, _jointRotAxis.Z);			// 関節回転軸の設定
					_jointType = Ode.JointType.Hinge;
					break;

				// スライダー関節
				case Ode.JointType.Slider:
					_joint = Ode.JointCreateSlider(_worldID, JointGroupID.Zero);// スライダー関節生成
					Ode.JointAttach(_joint, attachObject._body , _body);		// 関節を取り付ける
					Ode.JointSetSliderAxis(_joint, _jointRotAxis.X, _jointRotAxis.Y, _jointRotAxis.Z);	// 関節スライド軸の設定
					_jointType = Ode.JointType.Slider;
					break;

				default:
					break;
			}
		}

		/// <summary>
		/// オブジェクト関節のパラメータを設定
		/// </summary>
		/// <param name="parameter">設定を行うパラメータ</param>
		/// <param name="value">パラメータ値</param>
		public void SetJointParam(Ode.JointParam parameter, Real value)
		{
			switch (_jointType)
			{
				case Ode.JointType.Hinge:
					Ode.JointSetHingeParam(_joint, (int)parameter,  value);
					break;

				case Ode.JointType.Slider:
					Ode.JointSetSliderParam(_joint, (int)parameter,  value);
					break;

				default:
					break;
			}
		}

		/// <summary>
		/// オブジェクト関節のパラメータを取得
		/// </summary>
		/// <param name="parameter">取得するパラメータ</param>
		/// <returns>パラメータ値</returns>
		public Real GetJointParam(Ode.JointParam parameter)
		{
			switch (_jointType)
			{
				case Ode.JointType.Hinge:
					return Ode.JointGetHingeParam(_joint, (int)parameter);

				case Ode.JointType.Slider:
					return Ode.JointGetSliderParam(_joint, (int)parameter);

				default:
					return 0;
			}
		}

		/// <summary>
		/// オブジェクト関節の回転角を取得
		/// </summary>
		/// <returns>関節角</returns>
		public Real GetJointAnglePosition()
		{
			switch (_jointType)
			{
				case Ode.JointType.Hinge:
					return Ode.JointGetHingeAngle(_joint);

				case Ode.JointType.Slider:
					return Ode.JointGetSliderPosition(_joint);

				default:
					return 0;
			}
		}
		#endregion
	}

	/// <summary>
	/// 球オブジェクトクラス
	/// </summary>
	public class Sphere : ObjectSettings
	{
		#region fields
		Real _radius;
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="worldID"></param>
		/// <param name="spaceID"></param>
		/// <param name="posX"></param>
		/// <param name="posY"></param>
		/// <param name="posZ"></param>
		/// <param name="rotAxisX"></param>
		/// <param name="rotAxisY"></param>
		/// <param name="rotAxisZ"></param>
		/// <param name="rotAngle"></param>
		/// <param name="jointRotX"></param>
		/// <param name="jointRotY"></param>
		/// <param name="jointRotZ"></param>
		/// <param name="jointAxisX"></param>
		/// <param name="jointAxisY"></param>
		/// <param name="jointAxisZ"></param>
		/// <param name="mass"></param>
		/// <param name="radius"></param>
		public Sphere
		(
		    WorldID worldID, SpaceID spaceID,
		    Real posX, Real posY, Real posZ,
		    Real rotAxisX, Real rotAxisY, Real rotAxisZ,
		    Real rotAngle,
		    Real jointRotX, Real jointRotY, Real jointRotZ,
		    Real jointAxisX, Real jointAxisY, Real jointAxisZ,
		    Real mass,
		    Real radius
		)
		{
			Ode.Mass m;

			_worldID = worldID; _spaceID = spaceID;
			_pos.X = posX; _pos.Y = posY; _pos.Z = posZ;
			_rotAxis.X = rotAxisX; _rotAxis.Y = rotAxisY; _rotAxis.Z = rotAxisZ;
			_rotAngle = rotAngle;
			_jointRotCenter.X = jointRotX; _jointRotCenter.Y = jointRotY; _jointRotCenter.Z = jointRotZ;
			_jointRotAxis.X = jointAxisX; _jointRotAxis.Y = jointAxisY; _jointRotAxis.Z = jointAxisZ;
			_mass = mass;
			_radius = radius;

			_body = Ode.BodyCreate(_worldID);

			Ode.MassSetZero(out m);											// 質量パラメータの初期化
			Ode.MassSetSphereTotal(out m, _mass, _radius);					// 質量計算
			Ode.BodySetMass(_body, ref m);									// 質量の設定
			Ode.BodySetPosition(_body, _pos.X, _pos.Y, _pos.Z);				// 位置の設定

			Ode.Matrix3 R;
			Ode.RFromAxisAndAngle(out R, _rotAxis.X, _rotAxis.Y, _rotAxis.Z, _rotAngle);	// 回転軸・回転角から回転行列を計算
			Ode.BodySetRotation(_body, ref R);				// 回転行列を設定
			_geom = Ode.CreateSphere(_spaceID, _radius);	// ジオオメトリの生成
			Ode.GeomSetBody(_geom, _body);					// ボディとジオメトリの関連付け
		}

		/// <summary>
		/// 球オブジェクトをDrawstuffにより描画
		/// </summary>
		/// <param name="red">描画色赤成分[0,1]</param>
		/// <param name="green">描画色緑成分[0,1]</param>
		/// <param name="blue">描画色青成分[0,1]</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);			// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawSphere(ref pos, ref R, _radius);	// オブジェクトをDrawstuffで描画
		}

		/// <summary>
		/// 球オブジェクトをDrawstuffにより描画(半径指定)
		/// </summary>
		/// <param name="param">見かけ上の球オブジェクトの半径</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue, Real param)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);			// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawSphere(ref pos, ref R, param);	// オブジェクトをDrawstuffで描画
		}
	}

	/// <summary>
	/// 円柱オブジェクトクラス
	/// </summary>
	public class Cylinder : ObjectSettings
	{
		#region fields
		Real _radius;
		Real _length;
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="worldID"></param>
		/// <param name="spaceID"></param>
		/// <param name="posX"></param>
		/// <param name="posY"></param>
		/// <param name="posZ"></param>
		/// <param name="rotAxisX"></param>
		/// <param name="rotAxisY"></param>
		/// <param name="rotAxisZ"></param>
		/// <param name="rotAngle"></param>
		/// <param name="jointRotX"></param>
		/// <param name="jointRotY"></param>
		/// <param name="jointRotZ"></param>
		/// <param name="jointAxisX"></param>
		/// <param name="jointAxisY"></param>
		/// <param name="jointAxisZ"></param>
		/// <param name="mass"></param>
		/// <param name="length"></param>
		/// <param name="radius"></param>
		public Cylinder
		(
			WorldID worldID, SpaceID spaceID,
			Real posX, Real posY, Real posZ,
			Real rotAxisX, Real rotAxisY, Real rotAxisZ,
			Real rotAngle,
			Real jointRotX, Real jointRotY, Real jointRotZ,
			Real jointAxisX, Real jointAxisY, Real jointAxisZ,
			Real mass,
			Real radius,
			Real length
		)
		{
			Ode.Mass m;

			_worldID = worldID; _spaceID = spaceID;
			_pos.X = posX; _pos.Y = posY; _pos.Z = posZ;
			_rotAxis.X = rotAxisX; _rotAxis.Y = rotAxisY; _rotAxis.Z = rotAxisZ;
			_rotAngle = rotAngle;
			_jointRotCenter.X = jointRotX; _jointRotCenter.Y = jointRotY; _jointRotCenter.Z = jointRotZ;
			_jointRotAxis.X = jointAxisX; _jointRotAxis.Y = jointAxisY; _jointRotAxis.Z = jointAxisZ;
			_mass = mass;
			_radius = radius;
			_length = length;

			_body = Ode.BodyCreate(_worldID);

			Ode.MassSetZero(out m);														// 質量パラメータの初期化
			Ode.MassSetCylinderTotal(out m, _mass, (int)AxisType.Z, _radius, _length);	// 質量計算
			Ode.BodySetMass(_body, ref m);									// 質量の設定
			Ode.BodySetPosition(_body, _pos.X, _pos.Y, _pos.Z);				// 位置の設定

			Ode.Matrix3 R;
			Ode.RFromAxisAndAngle(out R, _rotAxis.X, _rotAxis.Y, _rotAxis.Z, _rotAngle);	// 回転軸・回転角から回転行列を計算
			Ode.BodySetRotation(_body, ref R);						// 回転行列を設定
			_geom = Ode.CreateCylinder(_spaceID, _radius, _length);	// ジオオメトリの生成
			Ode.GeomSetBody(_geom, _body);							// ボディとジオメトリの関連付け
		}

		/// <summary>
		/// 円柱オブジェクトをDrawstuffにより描画
		/// </summary>
		/// <param name="red">描画色赤成分[0,1]</param>
		/// <param name="green">描画色緑成分[0,1]</param>
		/// <param name="blue">描画色青成分[0,1]</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);						// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawCylinder(ref pos, ref R, _length, _radius);	// オブジェクトをDrawstuffで描画
		}

		/// <summary>
		/// 円柱オブジェクトをDrawstuffにより描画(半径指定)
		/// </summary>
		/// <param name="param">見かけ上の球オブジェクトの半径</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue, Real param)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);						// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawCylinder(ref pos, ref R, param, _radius);	// オブジェクトをDrawstuffで描画
		}
	}

	/// <summary>
	/// 直方体オブジェクトクラス
	/// </summary>
	public class Box : ObjectSettings
	{
		#region fields
		Ode.Vector3 _side;	// 辺
		#endregion

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="worldID"></param>
		/// <param name="spaceID"></param>
		/// <param name="posX"></param>
		/// <param name="posY"></param>
		/// <param name="posZ"></param>
		/// <param name="rotAxisX"></param>
		/// <param name="rotAxisY"></param>
		/// <param name="rotAxisZ"></param>
		/// <param name="rotAngle"></param>
		/// <param name="jointRotX"></param>
		/// <param name="jointRotY"></param>
		/// <param name="jointRotZ"></param>
		/// <param name="jointAxisX"></param>
		/// <param name="jointAxisY"></param>
		/// <param name="jointAxisZ"></param>
		/// <param name="mass"></param>
		/// <param name="sideX"></param>
		/// <param name="sideY"></param>
		/// <param name="sideZ"></param>
		public Box
		(
			WorldID worldID, SpaceID spaceID,
			Real posX, Real posY, Real posZ,
			Real rotAxisX, Real rotAxisY, Real rotAxisZ,
			Real rotAngle,
			Real jointRotX, Real jointRotY, Real jointRotZ,
			Real jointAxisX, Real jointAxisY, Real jointAxisZ,
			Real mass,
			Real sideX, Real sideY, Real sideZ
		)
		{
			Ode.Mass m;

			_worldID = worldID; _spaceID = spaceID;
			_pos.X = posX; _pos.Y = posY; _pos.Z = posZ;
			_rotAxis.X = rotAxisX; _rotAxis.Y = rotAxisY; _rotAxis.Z = rotAxisZ;
			_rotAngle = rotAngle;
			_jointRotCenter.X = jointRotX; _jointRotCenter.Y = jointRotY; _jointRotCenter.Z = jointRotZ;
			_jointRotAxis.X = jointAxisX; _jointRotAxis.Y = jointAxisY; _jointRotAxis.Z = jointAxisZ;
			_mass = mass;
			_side.X = sideX; _side.Y = sideY; _side.Z = sideZ;

			_body = Ode.BodyCreate(_worldID);

			Ode.MassSetZero(out m);											// 質量パラメータの初期化
			Ode.MassSetBoxTotal(out m, _mass, _side.X, _side.Y, _side.Z);	// 質量計算
			Ode.BodySetMass(_body, ref m);									// 質量の設定
			Ode.BodySetPosition(_body, _pos.X, _pos.Y, _pos.Z);				// 位置の設定

			Ode.Matrix3 R;
			Ode.RFromAxisAndAngle(out R, _rotAxis.X, _rotAxis.Y, _rotAxis.Z, _rotAngle);	// 回転軸・回転角から回転行列を計算
			Ode.BodySetRotation(_body, ref R);							// 回転行列を設定
			_geom = Ode.CreateBox(_spaceID, _side.X, _side.Y, _side.Z);	// ジオオメトリの生成
			Ode.GeomSetBody(_geom, _body);								// ボディとジオメトリの関連付け
		}

		/// <summary>
		/// 直方体オブジェクトをDrawstuffにより描画
		/// </summary>
		/// <param name="red">描画色赤成分[0,1]</param>
		/// <param name="green">描画色緑成分[0,1]</param>
		/// <param name="blue">描画色青成分[0,1]</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);			// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawBox(ref pos, ref R, ref _side);	// オブジェクトをDrawstuffで描画
		}

		/// <summary>
		/// 円柱オブジェクトをDrawstuffにより描画(半径指定)
		/// </summary>
		/// <param name="param">見かけ上の球オブジェクトの半径</param>
		public override void DrawByDrawstuff(Real red, Real green, Real blue, Real param)
		{
			Ode.Vector3 pos;
			GetBodyPosition(out pos);
			Ode.Matrix3 R;
			GetBodyRotation(out R);

			Ds.SetColor(red, green, blue);			// 色の設定(R,G,B) [0,1]で設定
			Ds.DrawBox(ref pos, ref R, ref _side);	// オブジェクトをDrawstuffで描画
		}
	}
}
