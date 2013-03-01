using System;
using System.Runtime.InteropServices;
using System.Security;

namespace CsOde
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
	/// Open Dynamics Engineのラッパークラス
	/// </summary>
	/// <remarks>オリジナルコードは ODE.NET-0.9 (http://sourceforge.net/project/showfiles.php?group_id=24884 )</remarks>
	public static class Ode
	{
		public static Real Infinity = Real.MaxValue;

		/// <summary>円周率</summary>
		#if Real == Single
		public static Real PI = (float)Math.PI;
		#elif Real == Double
		public static Real PI = Math.PI;
		#endif

		#region flag and enumeration
		[Flags]
		public enum ContactFlags : int
		{
			Mu2 = 0x001,
			FDir1 = 0x002,
			Bounce = 0x004,
			SoftERP = 0x008,
			SoftCFM = 0x010,
			Motion1 = 0x020,
			Motion2 = 0x040,
			Slip1 = 0x080,
			Slip2 = 0x100,
			Approx0 = 0x0000,
			Approx1_1 = 0x1000,
			Approx1_2 = 0x2000,
			Approx1 = 0x3000
		}

		public enum GeomClassID : int
		{
			SphereClass,
			BoxClass,
			CapsuleClass,
			CylinderClass,
			PlaneClass,
			RayClass,
			ConvexClass,
			GeomTransformClass,
			TriMeshClass,
			HeightfieldClass,
			FirstSpaceClass,
			SimpleSpaceClass = FirstSpaceClass,
			HashSpaceClass,
			QuadTreeSpaceClass,
			LastSpaceClass = QuadTreeSpaceClass,
			FirstUserClass,
			LastUserClass = FirstUserClass + MaxUserClasses - 1,
			NumClasses,
			MaxUserClasses = 4
		}

		public enum JointType : int
		{
			None,
			Ball,
			Hinge,
			Slider,
			Contact,
			Universal,
			Hinge2,
			Fixed,
			Null,
			AMotor,
			LMotor,
			Plane2D
		}

		public enum JointParam : int
		{
			LoStop,
			HiStop,
			Vel,
			FMax,
			FudgeFactor,
			Bounce,
			CFM,
			StopERP,
			StopCFM,
			SuspensionERP,
			SuspensionCFM,
			LoStop2 = 256,
			HiStop2,
			Vel2,
			FMax2,
			FudgeFactor2,
			Bounce2,
			CFM2,
			StopERP2,
			StopCFM2,
			SuspensionERP2,
			SuspensionCFM2,
			LoStop3 = 512,
			HiStop3,
			Vel3,
			FMax3,
			FudgeFactor3,
			Bounce3,
			CFM3,
			StopERP3,
			StopCFM3,
			SuspensionERP3,
			SuspensionCFM3
		}
		#endregion

		#region callback
		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate int AABBTestFn(GeomID o1, GeomID o2, ref AABB aabb);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate int ColliderFn(GeomID o1, GeomID o2, int flags, out ContactGeom contact, int skip);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void GetAABBFn(GeomID geom, out AABB aabb);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate ColliderFn GetColliderFnFn(int num);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void GeomDtorFn(GeomID o);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate Real HeightfieldGetHeight(IntPtr p_user_data, int x, int z);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate void NearCallback(IntPtr data, GeomID geom1, GeomID geom2);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate int TriCallback(GeomID trimesh, GeomID refObject, int triangleIndex);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate int TriArrayCallback(GeomID trimesh, GeomID refObject, int[] triangleIndex, int triCount);

		[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
		public delegate int TriRayCallback(GeomID trimesh, GeomID ray, int triangleIndex, Real u, Real v);
		#endregion

		#region structure
		[StructLayout(LayoutKind.Sequential)]
		public struct AABB
		{
			public Real MinX, MaxX;
			public Real MinY, MaxY;
			public Real MinZ, MaxZ;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Contact
		{
			public SurfaceParameters surface;
			public ContactGeom geom;
			public Vector3 fdir1;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct ContactGeom
		{
			public static readonly int SizeOf = Marshal.SizeOf(typeof(ContactGeom));

			public Vector3 pos;
			public Vector3 normal;
			public Real depth;
			public GeomID g1;
			public GeomID g2;
			public int side1;
			public int side2;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct GeomClass
		{
			public int bytes;
			public GetColliderFnFn collider;
			public GetAABBFn aabb;
			public AABBTestFn aabb_test;
			public GeomDtorFn dtor;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct JointFeedback
		{
			public Vector3 f1;
			public Vector3 t1;
			public Vector3 f2;
			public Vector3 t2;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Mass
		{
			public Real mass;
			public Vector4 c;
			public Matrix3 I;
		}

		/// <summary>modified by cherub</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Matrix3
		{
			public Real M11, M21, M31, M41;
			public Real M12, M22, M32, M42;
			public Real M13, M23, M33, M43;
		}
		/// <summary>modified by cherub</summary>
		[StructLayout(LayoutKind.Sequential)]
		public struct Matrix4
		{
			public Real M11, M21, M31, M41;
			public Real M12, M22, M32, M42;
			public Real M13, M23, M33, M43;
			public Real M14, M24, M34, M44;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Quaternion
		{
			public Real W, X, Y, Z;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct SurfaceParameters
		{
			public ContactFlags mode;
			public Real mu;
			public Real mu2;
			public Real bounce;
			public Real bounce_vel;
			public Real soft_erp;
			public Real soft_cfm;
			public Real motion1;
			public Real motion2;
			public Real slip1;
			public Real slip2;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Vector3
		{
			public Real X, Y, Z, W;
		}

		[StructLayout(LayoutKind.Sequential)]
		public struct Vector4
		{
			public Real X, Y, Z, W;
		}
		#endregion

		#region initialize/close
		[DllImport("ode", EntryPoint = "dInitODE"), SuppressUnmanagedCodeSecurity]
		public static extern void InitODE();

		[DllImport("ode", EntryPoint = "dCloseODE"), SuppressUnmanagedCodeSecurity]
		public static extern void CloseODE();
		#endregion

		#region world
		[DllImport("ode", EntryPoint = "dWorldCreate"), SuppressUnmanagedCodeSecurity]
		public static extern WorldID WorldCreate();

		[DllImport("ode", EntryPoint = "dWorldDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldDestroy(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoDisableAngularThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetAutoDisableAngularThreshold(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoDisableFlag"), SuppressUnmanagedCodeSecurity]
		public static extern bool WorldGetAutoDisableFlag(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoDisableLinearThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetAutoDisableLinearThreshold(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoDisableSteps"), SuppressUnmanagedCodeSecurity]
		public static extern int WorldGetAutoDisableSteps(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoDisableTime"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetAutoDisableTime(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetAutoEnableDepthSF1"), SuppressUnmanagedCodeSecurity]
		public static extern int WorldGetAutoEnableDepthSF1(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetCFM"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetCFM(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetERP"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetERP(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetGravity"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldGetGravity(WorldID world, out Vector3 gravity);

		[DllImport("ode", EntryPoint = "dWorldGetGravity"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldGetGravity(WorldID world, out Real X);

		[DllImport("ode", EntryPoint = "dWorldGetContactMaxCorrectingVel"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetContactMaxCorrectingVel(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetContactSurfaceLayer"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetContactSurfaceLayer(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetQuickStepNumIterations"), SuppressUnmanagedCodeSecurity]
		public static extern int WorldGetQuickStepNumIterations(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldGetQuickStepW"), SuppressUnmanagedCodeSecurity]
		public static extern Real WorldGetQuickStepW(WorldID world);

		[DllImport("ode", EntryPoint = "dWorldImpulseToForce"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldImpulseToForce(WorldID world, Real stepsize, Real ix, Real iy, Real iz, out Vector3 force);

		[DllImport("ode", EntryPoint = "dWorldImpulseToForce"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldImpulseToForce(WorldID world, Real stepsize, Real ix, Real iy, Real iz, out Real forceX);

		[DllImport("ode", EntryPoint = "dWorldQuickStep"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldQuickStep(WorldID world, Real stepsize);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableAngularThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoDisableAngularThreshold(WorldID world, Real angular_threshold);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableFlag"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoDisableFlag(WorldID world, bool do_auto_disable);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableLinearThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoDisableLinearThreshold(WorldID world, Real linear_threshold);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableSteps"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoDisableSteps(WorldID world, int steps);

		[DllImport("ode", EntryPoint = "dWorldSetAutoDisableTime"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoDisableTime(WorldID world, Real time);

		[DllImport("ode", EntryPoint = "dWorldSetAutoEnableDepthSF1"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetAutoEnableDepthSF1(WorldID world, int autoEnableDepth);

		[DllImport("ode", EntryPoint = "dWorldSetCFM"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetCFM(WorldID world, Real cfm);

		[DllImport("ode", EntryPoint = "dWorldSetContactMaxCorrectingVel"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetContactMaxCorrectingVel(WorldID world, Real vel);

		[DllImport("ode", EntryPoint = "dWorldSetContactSurfaceLayer"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetContactSurfaceLayer(WorldID world, Real depth);

		[DllImport("ode", EntryPoint = "dWorldSetERP"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetERP(WorldID world, Real erp);

		[DllImport("ode", EntryPoint = "dWorldSetGravity"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetGravity(WorldID world, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dWorldSetQuickStepNumIterations"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetQuickStepNumIterations(WorldID world, int num);

		[DllImport("ode", EntryPoint = "dWorldSetQuickStepW"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldSetQuickStepW(WorldID world, Real over_relaxation);

		[DllImport("ode", EntryPoint = "dWorldStep"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldStep(WorldID world, Real stepsize);

		[DllImport("ode", EntryPoint = "dWorldStepFast1"), SuppressUnmanagedCodeSecurity]
		public static extern void WorldStepFast1(WorldID world, Real stepsize, int maxiterations);
		#endregion

		#region body
		[DllImport("ode", EntryPoint = "dAreConnected"), SuppressUnmanagedCodeSecurity]
		public static extern bool AreConnected(BodyID b1, BodyID b2);

		[DllImport("ode", EntryPoint = "dAreConnectedExcluding"), SuppressUnmanagedCodeSecurity]
		public static extern bool AreConnectedExcluding(BodyID b1, BodyID b2, JointType joint_type);

		[DllImport("ode", EntryPoint = "dBodyAddForce"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddForce(BodyID body, Real fx, Real fy, Real fz);

		[DllImport("ode", EntryPoint = "dBodyAddForceAtPos"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddForceAtPos(BodyID body, Real fx, Real fy, Real fz, Real px, Real py, Real pz);

		[DllImport("ode", EntryPoint = "dBodyAddForceAtRelPos"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddForceAtRelPos(BodyID body, Real fx, Real fy, Real fz, Real px, Real py, Real pz);

		[DllImport("ode", EntryPoint = "dBodyAddRelForce"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddRelForce(BodyID body, Real fx, Real fy, Real fz);

		[DllImport("ode", EntryPoint = "dBodyAddRelForceAtPos"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddRelForceAtPos(BodyID body, Real fx, Real fy, Real fz, Real px, Real py, Real pz);

		[DllImport("ode", EntryPoint = "dBodyAddRelForceAtRelPos"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddRelForceAtRelPos(BodyID body, Real fx, Real fy, Real fz, Real px, Real py, Real pz);

		[DllImport("ode", EntryPoint = "dBodyAddRelTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddRelTorque(BodyID body, Real fx, Real fy, Real fz);

		[DllImport("ode", EntryPoint = "dBodyAddTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyAddTorque(BodyID body, Real fx, Real fy, Real fz);

		[DllImport("ode", EntryPoint = "dBodyCopyPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyPosition(BodyID body, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dBodyCopyPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyPosition(BodyID body, out Real X);

		[DllImport("ode", EntryPoint = "dBodyCopyQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyQuaternion(BodyID body, out Quaternion quat);

		[DllImport("ode", EntryPoint = "dBodyCopyQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyQuaternion(BodyID body, out Real X);

		[DllImport("ode", EntryPoint = "dBodyCopyRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyRotation(BodyID body, out Matrix3 R);

		[DllImport("ode", EntryPoint = "dBodyCopyRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyCopyRotation(BodyID body, out Real M00);

		[DllImport("ode", EntryPoint = "dBodyCreate"), SuppressUnmanagedCodeSecurity]
		public static extern BodyID BodyCreate(BodyID world);

		[DllImport("ode", EntryPoint = "dBodyDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyDestroy(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyDisable"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyDisable(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyEnable"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyEnable(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAutoDisableAngularThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern Real BodyGetAutoDisableAngularThreshold(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAutoDisableFlag"), SuppressUnmanagedCodeSecurity]
		public static extern bool BodyGetAutoDisableFlag(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAutoDisableLinearThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern Real BodyGetAutoDisableLinearThreshold(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAutoDisableSteps"), SuppressUnmanagedCodeSecurity]
		public static extern int BodyGetAutoDisableSteps(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAutoDisableTime"), SuppressUnmanagedCodeSecurity]
		public static extern Real BodyGetAutoDisableTime(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetAngularVel"), SuppressUnmanagedCodeSecurity]
		/// <summary>modified by cherub</summary>
		public static extern IntPtr BodyGetAngularVel(BodyID body);
		/// <summary>added by cherub</summary>
		public static void BodyCopyAngularVel(BodyID body, out Vector3 vel)
		{
			IntPtr src = BodyGetAngularVel(body);
			Real[] buf = new Real[4];
			Marshal.Copy(src, buf, 0, 4);
			vel.X = buf[0];
			vel.Y = buf[1];
			vel.Z = buf[2];
			vel.W = buf[3];
		}

		[DllImport("ode", EntryPoint = "dBodyGetData"), SuppressUnmanagedCodeSecurity]
		public static extern BodyID BodyGetData(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetFiniteRotationMode"), SuppressUnmanagedCodeSecurity]
		public static extern int BodyGetFiniteRotationMode(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetFiniteRotationAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetFiniteRotationAxis(BodyID body, out Vector3 result);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dBodyGetForce"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr BodyGetForce(BodyID body);
		/// <summary>added by cherub</summary>
		public static void BodyCopyForce(BodyID body, out Vector3 force)
		{
			IntPtr src = BodyGetForce(body);
			Real[] buf = new Real[4];
			Marshal.Copy(src, buf, 0, 4);
			force.X = buf[0];
			force.Y = buf[1];
			force.Z = buf[2];
			force.W = buf[3];
		}

		[DllImport("ode", EntryPoint = "dBodyGetGravityMode"), SuppressUnmanagedCodeSecurity]
		public static extern bool BodyGetGravityMode(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetJoint"), SuppressUnmanagedCodeSecurity]
		public static extern BodyID BodyGetJoint(BodyID body, int index);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dBodyGetLinearVel"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr BodyGetLinearVel(BodyID body);
		/// <summary>added by cherub</summary>
		public static void BodyCopyLinearVel(BodyID body, out Vector3 vel)
		{
			IntPtr src = BodyGetLinearVel(body);
			Real[] buf = new Real[4];
			Marshal.Copy(src, buf, 0, 4);
			vel.X = buf[0];
			vel.Y = buf[1];
			vel.Z = buf[2];
			vel.W = buf[3];
		}

		[DllImport("ode", EntryPoint = "dBodyGetMass"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetMass(BodyID body, out Mass mass);

		[DllImport("ode", EntryPoint = "dBodyGetNumJoints"), SuppressUnmanagedCodeSecurity]
		public static extern int BodyGetNumJoints(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetPointVel"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetPointVel(BodyID body, Real px, Real py, Real pz, out Vector3 result);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dBodyGetPosition"), SuppressUnmanagedCodeSecurityAttribute]
		public static extern IntPtr BodyGetPosition(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetPosRelPoint"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetPosRelPoint(BodyID body, Real px, Real py, Real pz, out Vector3 result);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dBodyGetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr BodyGetQuaternion(BodyID body);

		[DllImport("ode", EntryPoint = "dBodyGetRelPointPos"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetRelPointPos(BodyID body, Real px, Real py, Real pz, out Vector3 result);

		[DllImport("ode", EntryPoint = "dBodyGetRelPointVel"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyGetRelPointVel(BodyID body, Real px, Real py, Real pz, out Vector3 result);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dBodyGetTorque"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr BodyGetTorque(BodyID body);
		/// <summary>added by cherub</summary>
		public static void BodyCopyTorque(BodyID body, out Vector3 torque)
		{
			IntPtr src = BodyGetTorque(body);
			Real[] buf = new Real[4];
			Marshal.Copy(src, buf, 0, 4);
			torque.X = buf[0];
			torque.Y = buf[1];
			torque.Z = buf[2];
			torque.W = buf[3];
		}

		[DllImport("ode", EntryPoint = "dBodyIsEnabled"), SuppressUnmanagedCodeSecurity]
		public static extern bool BodyIsEnabled(BodyID body);

		[DllImport("ode", EntryPoint = "dBodySetAngularVel"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAngularVel(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableAngularThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableAngularThreshold(BodyID body, Real angular_threshold);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableDefaults"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableDefaults(BodyID body);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableFlag"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableFlag(BodyID body, bool do_auto_disable);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableLinearThreshold"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableLinearThreshold(BodyID body, Real linear_threshold);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableSteps"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableSteps(BodyID body, int steps);

		[DllImport("ode", EntryPoint = "dBodySetAutoDisableTime"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetAutoDisableTime(BodyID body, Real time);

		[DllImport("ode", EntryPoint = "dBodySetData"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetData(BodyID body, IntPtr data);

		[DllImport("ode", EntryPoint = "dBodySetFiniteRotationMode"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetFiniteRotationMode(BodyID body, int mode);

		[DllImport("ode", EntryPoint = "dBodySetFiniteRotationModeAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetFiniteRotationModeAxis(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodySetForce"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetForce(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodySetGravityMode"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetGravityMode(BodyID body, bool mode);

		[DllImport("ode", EntryPoint = "dBodySetLinearVel"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetLinearVel(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodySetMass"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetMass(BodyID body, ref Mass mass);

		[DllImport("ode", EntryPoint = "dBodySetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetPosition(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodySetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetQuaternion(BodyID body, ref Quaternion q);

		[DllImport("ode", EntryPoint = "dBodySetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetQuaternion(BodyID body, ref Real w);

		[DllImport("ode", EntryPoint = "dBodySetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetRotation(BodyID body, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dBodySetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetRotation(BodyID body, ref Real M00);

		[DllImport("ode", EntryPoint = "dBodySetTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void BodySetTorque(BodyID body, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dBodyVectorFromWorld"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyVectorFromWorld(BodyID body, Real px, Real py, Real pz, out Vector3 result);

		[DllImport("ode", EntryPoint = "dBodyVectorToWorld"), SuppressUnmanagedCodeSecurity]
		public static extern void BodyVectorToWorld(BodyID body, Real px, Real py, Real pz, out Vector3 result);
		#endregion

		#region geometry
		[DllImport("ode", EntryPoint = "dCollide"), SuppressUnmanagedCodeSecurity]
		public static extern int Collide(GeomID o1, GeomID o2, int flags, [In, Out] ContactGeom[] contact, int skip);

		[DllImport("ode", EntryPoint = "dBoxBox"), SuppressUnmanagedCodeSecurity]
		public static extern void BoxBox(ref Vector3 p1, ref Matrix3 R1,
			ref Vector3 side1, ref Vector3 p2,
			ref Matrix3 R2, ref Vector3 side2,
			ref Vector3 normal, out Real depth, out int return_code,
			int maxc, out ContactGeom contact, int skip);

		[DllImport("ode", EntryPoint = "dBoxTouchesBox"), SuppressUnmanagedCodeSecurity]
		public static extern void BoxTouchesBox(ref Vector3 _p1, ref Matrix3 R1,
			ref Vector3 side1, ref Vector3 _p2,
			ref Matrix3 R2, ref Vector3 side2);

		[DllImport("ode", EntryPoint = "dClosestLineSegmentPoints"), SuppressUnmanagedCodeSecurity]
		public static extern void ClosestLineSegmentPoints(ref Vector3 a1, ref Vector3 a2,
			ref Vector3 b1, ref Vector3 b2,
			ref Vector3 cp1, ref Vector3 cp2);

		[DllImport("ode", EntryPoint = "dConnectingJoint"), SuppressUnmanagedCodeSecurity]
		public static extern JointID ConnectingJoint(JointID joint1, JointID joint2);

		[DllImport("ode", EntryPoint = "dCreateBox"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateBox(SpaceID space, Real lx, Real ly, Real lz);

		[DllImport("ode", EntryPoint = "dCreateCapsule"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateCapsule(SpaceID space, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dCreateConvex"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateConvex(SpaceID space, Real[] planes, int planeCount, Real[] points, int pointCount, int[] polygons);

		[DllImport("ode", EntryPoint = "dCreateCylinder"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateCylinder(SpaceID space, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dCreateHeightfield"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateHeightfield(SpaceID space, IntPtr data, int bPlaceable);

		[DllImport("ode", EntryPoint = "dCreateGeom"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateGeom(int classnum);

		[DllImport("ode", EntryPoint = "dCreateGeomClass"), SuppressUnmanagedCodeSecurity]
		public static extern int CreateGeomClass(ref GeomClass classptr);

		[DllImport("ode", EntryPoint = "dCreateGeomTransform"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateGeomTransform(SpaceID space);

		[DllImport("ode", EntryPoint = "dCreatePlane"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreatePlane(SpaceID space, Real a, Real b, Real c, Real d);

		[DllImport("ode", EntryPoint = "dCreateRay"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateRay(SpaceID space, Real length);

		[DllImport("ode", EntryPoint = "dCreateSphere"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateSphere(SpaceID space, Real radius);

		[DllImport("ode", EntryPoint = "dCreateTriMesh"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID CreateTriMesh(SpaceID space, IntPtr data,
			TriCallback callback, TriArrayCallback arrayCallback, TriRayCallback rayCallback);

		[DllImport("ode", EntryPoint = "dGeomBoxGetLengths"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomBoxGetLengths(GeomID geom, out Vector3 len);

		[DllImport("ode", EntryPoint = "dGeomBoxGetLengths"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomBoxGetLengths(GeomID geom, out Real x);

		[DllImport("ode", EntryPoint = "dGeomBoxPointDepth"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomBoxPointDepth(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomBoxSetLengths"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomBoxSetLengths(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomCapsuleGetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCapsuleGetParams(GeomID geom, out Real radius, out Real length);

		[DllImport("ode", EntryPoint = "dGeomCapsulePointDepth"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomCapsulePointDepth(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomCapsuleSetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCapsuleSetParams(GeomID geom, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dGeomClearOffset"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomClearOffset(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomCopyOffsetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomCopyOffsetPosition(GeomID geom, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dGeomCopyOffsetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomCopyOffsetPosition(GeomID geom, out Real X);

		[DllImport("ode", EntryPoint = "dGeomGetOffsetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyOffsetQuaternion(GeomID geom, out Quaternion Q);

		[DllImport("ode", EntryPoint = "dGeomGetOffsetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyOffsetQuaternion(GeomID geom, out Real X);

		[DllImport("ode", EntryPoint = "dGeomCopyOffsetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomCopyOffsetRotation(GeomID geom, out Matrix3 R);

		[DllImport("ode", EntryPoint = "dGeomCopyOffsetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomCopyOffsetRotation(GeomID geom, out Real M00);

		[DllImport("ode", EntryPoint = "dGeomCopyPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyPosition(GeomID geom, out Vector3 pos);

		[DllImport("ode", EntryPoint = "dGeomCopyPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyPosition(GeomID geom, out Real X);

		[DllImport("ode", EntryPoint = "dGeomCopyRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyRotation(GeomID geom, out Matrix3 R);

		[DllImport("ode", EntryPoint = "dGeomCopyRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyRotation(GeomID geom, out Real M00);

		[DllImport("ode", EntryPoint = "dGeomCylinderGetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCylinderGetParams(GeomID geom, out Real radius, out Real length);

		[DllImport("ode", EntryPoint = "dGeomCylinderSetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCylinderSetParams(GeomID geom, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dGeomDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomDestroy(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomDisable"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomDisable(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomEnable"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomEnable(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetAABB"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomGetAABB(GeomID geom, out AABB aabb);

		[DllImport("ode", EntryPoint = "dGeomGetAABB"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomGetAABB(GeomID geom, out Real minX);

		[DllImport("ode", EntryPoint = "dGeomGetBody"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomGetBody(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetCategoryBits"), SuppressUnmanagedCodeSecurity]
		public static extern int GeomGetCategoryBits(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetClassData"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomGetClassData(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetCollideBits"), SuppressUnmanagedCodeSecurity]
		public static extern int GeomGetCollideBits(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetClass"), SuppressUnmanagedCodeSecurity]
		public static extern GeomClassID GeomGetClass(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetData"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomGetData(GeomID geom);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomGetOffsetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomGetOffsetPosition(GeomID geom);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomGetOffsetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomGetOffsetRotation(GeomID geom);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomGetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomGetPosition(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyQuaternion(GeomID geom, out Quaternion q);

		[DllImport("ode", EntryPoint = "dGeomGetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomCopyQuaternion(GeomID geom, out Real X);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomGetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomGetRotation(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomGetSpace"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomGetSpace(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomIsEnabled"), SuppressUnmanagedCodeSecurity]
		public static extern bool GeomIsEnabled(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomIsOffset"), SuppressUnmanagedCodeSecurity]
		public static extern bool GeomIsOffset(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomIsSpace"), SuppressUnmanagedCodeSecurity]
		public static extern bool GeomIsSpace(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomPlaneGetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomPlaneGetParams(GeomID geom, out Vector4 result);

		[DllImport("ode", EntryPoint = "dGeomPlaneGetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomPlaneGetParams(GeomID geom, out Real A);

		[DllImport("ode", EntryPoint = "dGeomPlanePointDepth"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomPlanePointDepth(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomPlaneSetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomPlaneSetParams(GeomID plane, Real a, Real b, Real c, Real d);

		[DllImport("ode", EntryPoint = "dGeomRayGet"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRayGet(GeomID ray, out Vector3 start, out Vector3 dir);

		[DllImport("ode", EntryPoint = "dGeomRayGet"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRayGet(GeomID ray, out Real startX, out Real dirX);

		[DllImport("ode", EntryPoint = "dGeomRayGetClosestHit"), SuppressUnmanagedCodeSecurity]
		public static extern int GeomRayGetClosestHit(GeomID ray);

		[DllImport("ode", EntryPoint = "dGeomRayGetLength"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomRayGetLength(GeomID ray);

		[DllImport("ode", EntryPoint = "dGeomRayGetParams"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomRayGetParams(GeomID g, out int firstContact, out int backfaceCull);

		[DllImport("ode", EntryPoint = "dGeomRaySet"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRaySet(GeomID ray, Real px, Real py, Real pz, Real dx, Real dy, Real dz);

		[DllImport("ode", EntryPoint = "dGeomRaySetClosestHit"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRaySetClosestHit(GeomID ray, int closestHit);

		[DllImport("ode", EntryPoint = "dGeomRaySetLength"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRaySetLength(GeomID ray, Real length);

		[DllImport("ode", EntryPoint = "dGeomRaySetParams"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomRaySetParams(GeomID ray, int firstContact, int backfaceCull);

		[DllImport("ode", EntryPoint = "dGeomSetBody"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetBody(GeomID geom, BodyID body);

		[DllImport("ode", EntryPoint = "dGeomSetCategoryBits"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetCategoryBits(GeomID geom, int bits);

		[DllImport("ode", EntryPoint = "dGeomSetCollideBits"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetCollideBits(GeomID geom, int bits);

		[DllImport("ode", EntryPoint = "dGeomSetConvex"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomSetConvex(GeomID geom, Real[] planes, int planeCount, Real[] points, int pointCount, int[] polygons);

		[DllImport("ode", EntryPoint = "dGeomSetData"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetData(GeomID geom, IntPtr data);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetPosition(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetQuaternion(GeomID geom, ref Quaternion Q);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetQuaternion(GeomID geom, ref Real X);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetRotation(GeomID geom, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetRotation(GeomID geom, ref Real M00);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetWorldPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetWorldPosition(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetWorldQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetWorldQuaternion(GeomID geom, ref Quaternion Q);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetWorldQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetWorldQuaternion(GeomID geom, ref Real X);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetWorldRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetWorldRotation(GeomID geom, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dGeomSetOffsetWorldRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetOffsetWorldRotation(GeomID geom, ref Real M00);

		[DllImport("ode", EntryPoint = "dGeomSetPosition"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetPosition(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomSetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetQuaternion(GeomID geom, ref Quaternion quat);

		[DllImport("ode", EntryPoint = "dGeomSetQuaternion"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetQuaternion(GeomID geom, ref Real w);

		[DllImport("ode", EntryPoint = "dGeomSetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetRotation(GeomID geom, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dGeomSetRotation"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSetRotation(GeomID geom, ref Real M00);

		[DllImport("ode", EntryPoint = "dGeomSphereGetRadius"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomSphereGetRadius(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomSpherePointDepth"), SuppressUnmanagedCodeSecurity]
		public static extern Real GeomSpherePointDepth(GeomID geom, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dGeomSphereSetRadius"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomSphereSetRadius(GeomID geom, Real radius);

		[DllImport("ode", EntryPoint = "dGeomTransformGetCleanup"), SuppressUnmanagedCodeSecurity]
		public static extern int GeomTransformGetCleanup(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomTransformGetGeom"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomTransformGetGeom(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomTransformGetInfo"), SuppressUnmanagedCodeSecurity]
		public static extern int GeomTransformGetInfo(GeomID geom);

		[DllImport("ode", EntryPoint = "dGeomTransformSetCleanup"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTransformSetCleanup(GeomID geom, int mode);

		[DllImport("ode", EntryPoint = "dGeomTransformSetGeom"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTransformSetGeom(GeomID geom, GeomID obj);

		[DllImport("ode", EntryPoint = "dGeomTransformSetInfo"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTransformSetInfo(GeomID geom, int info);
		#endregion

		#region geometry heightfield data
		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildByte"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildByte(HeightfieldDataID d, byte[] pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildByte"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildByte(HeightfieldDataID d, IntPtr pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildCallback"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildCallback(HeightfieldDataID d, IntPtr pUserData, HeightfieldGetHeight pCallback,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[CLSCompliant(false)]
		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildShort"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildShort(HeightfieldDataID d, ushort[] pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildShort"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildShort(HeightfieldDataID d, short[] pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildShort"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildShort(HeightfieldDataID d, IntPtr pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildSingle"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildSingle(HeightfieldDataID d, float[] pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildSingle"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildSingle(HeightfieldDataID d, IntPtr pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildDouble"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildDouble(HeightfieldDataID d, double[] pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataBuildDouble"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataBuildDouble(HeightfieldDataID d, IntPtr pHeightData, int bCopyHeightData,
				Real width, Real depth, int widthSamples, int depthSamples,
				Real scale, Real offset, Real thickness, int bWrap);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataCreate"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomHeightfieldDataCreate();

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataDestroy(HeightfieldDataID d);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldDataSetBounds"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldDataSetBounds(HeightfieldDataID d, Real minHeight, Real maxHeight);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldGetHeightfieldData"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomHeightfieldGetHeightfieldData(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomHeightfieldSetHeightfieldData"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomHeightfieldSetHeightfieldData(GeomID g, HeightfieldDataID d);
		#endregion

		#region geometry trimesh data
		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildDouble"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildDouble(TriMeshDataID d,
			double[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildDouble"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildDouble(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildDouble1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildDouble1(TriMeshDataID d,
			double[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride,
			double[] normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildDouble1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildDouble(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride,
			IntPtr normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSimple"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSingle(TriMeshDataID d,
			Real[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSimple"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSingle(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSimple1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSingle1(TriMeshDataID d,
			Real[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride,
			Real[] normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSimple1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSingle1(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride,
			IntPtr normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSingle"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSimple(TriMeshDataID d,
			float[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSingle"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSimple(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSingle1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSimple1(TriMeshDataID d,
			float[] vertices, int vertexStride, int vertexCount,
			int[] indices, int indexCount, int triStride,
			float[] normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataBuildSingle1"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataBuildSimple1(TriMeshDataID d,
			IntPtr vertices, int vertexStride, int vertexCount,
			IntPtr indices, int indexCount, int triStride,
			IntPtr normals);

		[DllImport("ode", EntryPoint = "dGeomTriMeshClearTCCache"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshClearTCCache(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataCreate"), SuppressUnmanagedCodeSecurity]
		public static extern TriMeshDataID GeomTriMeshDataCreate();

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataDestroy(TriMeshDataID d);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomTriMeshDataGet"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomTriMeshDataGet(TriMeshDataID d, int data_id);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataPreprocess"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataPreprocess(TriMeshDataID d);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataSet"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataSet(TriMeshDataID d, int data_id, IntPtr in_data);

		[DllImport("ode", EntryPoint = "dGeomTriMeshDataUpdate"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshDataUpdate(TriMeshDataID d);

		[DllImport("ode", EntryPoint = "dGeomTriMeshEnableTC"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshEnableTC(GeomID g, int geomClass, bool enable);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetArrayCallback"), SuppressUnmanagedCodeSecurity]
		public static extern TriArrayCallback GeomTriMeshGetArrayCallback(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetCallback"), SuppressUnmanagedCodeSecurity]
		public static extern TriCallback GeomTriMeshGetCallback(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetData"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomTriMeshGetData(GeomID g);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dGeomTriMeshGetLastTransform"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr GeomTriMeshGetLastTransform(GeomID geom);
		/// <summary>added by cherub</summary>
		public static void GeomTriMeshCopyLastTransform(GeomID geom, out Matrix4 trans)
		{
			IntPtr src = GeomTriMeshGetLastTransform(geom);
			Real[] buf = new Real[16];
			Marshal.Copy(src, buf, 0, 16);

			trans.M11 = buf[0];
			trans.M21 = buf[1];
			trans.M31 = buf[2];
			trans.M41 = buf[3];
			trans.M12 = buf[4];
			trans.M22 = buf[5];
			trans.M32 = buf[6];
			trans.M42 = buf[7];
			trans.M13 = buf[8];
			trans.M23 = buf[9];
			trans.M33 = buf[10];
			trans.M43 = buf[11];
			trans.M14 = buf[12];
			trans.M24 = buf[13];
			trans.M34 = buf[14];
			trans.M44 = buf[15];
		}

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetPoint"), SuppressUnmanagedCodeSecurity]
		public extern static void GeomTriMeshGetPoint(GeomID g, int index, Real u, Real v, out Vector3 outVec);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetRayCallback"), SuppressUnmanagedCodeSecurity]
		public static extern TriRayCallback GeomTriMeshGetRayCallback(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetTriangle"), SuppressUnmanagedCodeSecurity]
		public extern static void GeomTriMeshGetTriangle(GeomID g, int index, out Vector3 v0, out Vector3 v1, out Vector3 v2);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetTriangleCount"), SuppressUnmanagedCodeSecurity]
		public extern static int GeomTriMeshGetTriangleCount(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshGetTriMeshDataID"), SuppressUnmanagedCodeSecurity]
		public static extern GeomID GeomTriMeshGetTriMeshDataID(GeomID g);

		[DllImport("ode", EntryPoint = "dGeomTriMeshIsTCEnabled"), SuppressUnmanagedCodeSecurity]
		public static extern bool GeomTriMeshIsTCEnabled(GeomID g, int geomClass);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetArrayCallback"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetArrayCallback(GeomID g, TriArrayCallback arrayCallback);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetCallback"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetCallback(GeomID g, TriCallback callback);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetData"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetData(GeomID g, IntPtr data);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetLastTransform"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetLastTransform(GeomID g, ref Matrix4 last_trans);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetLastTransform"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetLastTransform(GeomID g, ref Real M00);

		[DllImport("ode", EntryPoint = "dGeomTriMeshSetRayCallback"), SuppressUnmanagedCodeSecurity]
		public static extern void GeomTriMeshSetRayCallback(GeomID g, TriRayCallback callback);
		#endregion

		#region space
		[DllImport("ode", EntryPoint = "dSpaceAdd"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceAdd(SpaceID space, GeomID geom);

		[DllImport("ode", EntryPoint = "dSpaceClean"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceClean(SpaceID space);

		[DllImport("ode", EntryPoint = "dSpaceCollide"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceCollide(SpaceID space, IntPtr data, NearCallback callback);

		[DllImport("ode", EntryPoint = "dSpaceCollide2"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceCollide2(SpaceID space1, SpaceID space2, IntPtr data, NearCallback callback);

		[DllImport("ode", EntryPoint = "dSpaceDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceDestroy(SpaceID space);

		[DllImport("ode", EntryPoint = "dSpaceGetCleanup"), SuppressUnmanagedCodeSecurity]
		public static extern bool SpaceGetCleanup(SpaceID space);

		[DllImport("ode", EntryPoint = "dSpaceGetNumGeoms"), SuppressUnmanagedCodeSecurity]
		public static extern int SpaceGetNumGeoms(SpaceID space);

		[DllImport("ode", EntryPoint = "dSpaceGetGeom"), SuppressUnmanagedCodeSecurity]
		public static extern SpaceID SpaceGetGeom(SpaceID space, int i);

		[DllImport("ode", EntryPoint = "dSpaceQuery"), SuppressUnmanagedCodeSecurity]
		public static extern bool SpaceQuery(SpaceID space, GeomID geom);

		[DllImport("ode", EntryPoint = "dSpaceRemove"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceRemove(SpaceID space, GeomID geom);

		[DllImport("ode", EntryPoint = "dSpaceSetCleanup"), SuppressUnmanagedCodeSecurity]
		public static extern void SpaceSetCleanup(SpaceID space, bool mode);

		[DllImport("ode", EntryPoint = "dSimpleSpaceCreate"), SuppressUnmanagedCodeSecurity]
		public static extern SpaceID SimpleSpaceCreate(SpaceID space);

		[DllImport("ode", EntryPoint = "dHashSpaceCreate"), SuppressUnmanagedCodeSecurity]
		public static extern SpaceID HashSpaceCreate(SpaceID space);

		[DllImport("ode", EntryPoint = "dHashSpaceGetLevels"), SuppressUnmanagedCodeSecurity]
		public static extern void HashSpaceGetLevels(SpaceID space, out int minlevel, out int maxlevel);

		[DllImport("ode", EntryPoint = "dHashSpaceSetLevels"), SuppressUnmanagedCodeSecurity]
		public static extern void HashSpaceSetLevels(SpaceID space, int minlevel, int maxlevel);

		[DllImport("ode", EntryPoint = "dQuadTreeSpaceCreate"), SuppressUnmanagedCodeSecurity]
		public static extern SpaceID QuadTreeSpaceCreate(SpaceID space, ref Vector3 center, ref Vector3 extents, int depth);

		[DllImport("ode", EntryPoint = "dQuadTreeSpaceCreate"), SuppressUnmanagedCodeSecurity]
		public static extern SpaceID QuadTreeSpaceCreate(SpaceID space, ref Real centerX, ref Real extentsX, int depth);
		#endregion

		#region joint
		[DllImport("ode", EntryPoint = "dJointAddAMotorTorques"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddAMotorTorques(JointID joint, Real torque1, Real torque2, Real torque3);

		[DllImport("ode", EntryPoint = "dJointAddHingeTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddHingeTorque(JointID joint, Real torque);

		[DllImport("ode", EntryPoint = "dJointAddHinge2Torque"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddHinge2Torques(JointID joint, Real torque1, Real torque2);

		[DllImport("ode", EntryPoint = "dJointAddPRTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddPRTorque(JointID joint, Real torque);

		[DllImport("ode", EntryPoint = "dJointAddUniversalTorque"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddUniversalTorques(JointID joint, Real torque1, Real torque2);

		[DllImport("ode", EntryPoint = "dJointAddSliderForce"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAddSliderForce(JointID joint, Real force);

		[DllImport("ode", EntryPoint = "dJointAttach"), SuppressUnmanagedCodeSecurity]
		public static extern void JointAttach(JointID joint, BodyID body1, BodyID body2);

		[DllImport("ode", EntryPoint = "dJointCreateAMotor"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateAMotor(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateBall"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateBall(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateContact"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateContact(WorldID world, JointGroupID group, ref Contact contact);

		[DllImport("ode", EntryPoint = "dJointCreateFixed"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateFixed(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateHinge"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateHinge(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateHinge2"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateHinge2(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateLMotor"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateLMotor(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateNull"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateNull(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreatePR"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreatePR(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreatePlane2D"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreatePlane2D(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateSlider"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateSlider(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointCreateUniversal"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointCreateUniversal(WorldID world, JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void JointDestroy(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetAMotorAngle"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetAMotorAngle(JointID joint, int anum);

		[DllImport("ode", EntryPoint = "dJointGetAMotorAngleRate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetAMotorAngleRate(JointID joint, int anum);

		[DllImport("ode", EntryPoint = "dJointGetAMotorAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetAMotorAxis(JointID joint, int anum, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetAMotorAxisRel"), SuppressUnmanagedCodeSecurity]
		public static extern int JointGetAMotorAxisRel(JointID joint, int anum);

		[DllImport("ode", EntryPoint = "dJointGetAMotorMode"), SuppressUnmanagedCodeSecurity]
		public static extern int JointGetAMotorMode(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetAMotorNumAxes"), SuppressUnmanagedCodeSecurity]
		public static extern int JointGetAMotorNumAxes(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetAMotorParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetAMotorParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetBallAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetBallAnchor(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetBallAnchor2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetBallAnchor2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetBody"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointGetBody(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetData"), SuppressUnmanagedCodeSecurity]
		public static extern JointID JointGetData(JointID joint);

		/// <summary>modified by cherub</summary>
		[DllImport("ode", EntryPoint = "dJointGetFeedback"), SuppressUnmanagedCodeSecurity]
		public static extern IntPtr JointGetFeedback(JointID joint);
		/// <summary>added by cherub</summary>
		public static void JointCopyFeedback(JointID joint, out JointFeedback feedback)
		{
			Real[] p = new Real[16];
			IntPtr ptr = JointGetFeedback(joint);
			Marshal.Copy(ptr, p, 0, 16);
			feedback.f1.X = p[0];
			feedback.f1.Y = p[1];
			feedback.f1.Z = p[2];
			feedback.f1.W = p[3];
			feedback.t1.X = p[4];
			feedback.t1.Y = p[5];
			feedback.t1.Z = p[6];
			feedback.t1.W = p[7];
			feedback.f2.X = p[8];
			feedback.f2.Y = p[9];
			feedback.f2.Z = p[10];
			feedback.f2.W = p[11];
			feedback.t2.X = p[12];
			feedback.t2.Y = p[13];
			feedback.t2.Z = p[14];
			feedback.t2.W = p[15];
		}

		[DllImport("ode", EntryPoint = "dJointGetHingeAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHingeAnchor(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHingeAngle"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHingeAngle(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetHingeAngleRate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHingeAngleRate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetHingeAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHingeAxis(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHingeParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHingeParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Angle1"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHinge2Angle1(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Angle1Rate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHinge2Angle1Rate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Angle2Rate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHinge2Angle2Rate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetHingeAnchor2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHingeAnchor2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Anchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHinge2Anchor(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Anchor2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHinge2Anchor2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Axis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHinge2Axis1(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Axis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetHinge2Axis2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetHinge2Param"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetHinge2Param(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetLMotorAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetLMotorAxis(JointID joint, int anum, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetLMotorNumAxes"), SuppressUnmanagedCodeSecurity]
		public static extern int JointGetLMotorNumAxes(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetLMotorParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetLMotorParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetPRAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetPRAnchor(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetPRAxis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetPRAxis1(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetPRAxis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetPRAxis2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetPRParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetPRParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetPRPosition"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetPRPosition(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetPRPositionRate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetPRPositionRate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetSliderAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetSliderAxis(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetSliderParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetSliderParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGetSliderPosition"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetSliderPosition(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetSliderPositionRate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetSliderPositionRate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetType"), SuppressUnmanagedCodeSecurity]
		public static extern JointType JointGetType(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetUniversalAnchor(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAnchor2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetUniversalAnchor2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAngle1"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetUniversalAngle1(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAngle1Rate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetUniversalAngle1Rate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAngle2"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetUniversalAngle2(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAngle2Rate"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetUniversalAngle2Rate(JointID joint);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAngles"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetUniversalAngles(JointID joint, out Real angle1, out Real angle2);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAxis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetUniversalAxis1(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetUniversalAxis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGetUniversalAxis2(JointID joint, out Vector3 result);

		[DllImport("ode", EntryPoint = "dJointGetUniversalParam"), SuppressUnmanagedCodeSecurity]
		public static extern Real JointGetUniversalParam(JointID joint, int parameter);

		[DllImport("ode", EntryPoint = "dJointGroupCreate"), SuppressUnmanagedCodeSecurity]
		public static extern JointGroupID JointGroupCreate(int max_size);

		[DllImport("ode", EntryPoint = "dJointGroupDestroy"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGroupDestroy(JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointGroupEmpty"), SuppressUnmanagedCodeSecurity]
		public static extern void JointGroupEmpty(JointGroupID group);

		[DllImport("ode", EntryPoint = "dJointSetAMotorAngle"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetAMotorAngle(JointID joint, int anum, Real angle);

		[DllImport("ode", EntryPoint = "dJointSetAMotorAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetAMotorAxis(JointID joint, int anum, int rel, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetAMotorMode"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetAMotorMode(JointID joint, int mode);

		[DllImport("ode", EntryPoint = "dJointSetAMotorNumAxes"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetAMotorNumAxes(JointGroupID group, int num);

		[DllImport("ode", EntryPoint = "dJointSetAMotorParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetAMotorParam(JointGroupID group, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetBallAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetBallAnchor(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetBallAnchor2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetBallAnchor2(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetData"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetData(JointID joint, IntPtr data);

		[DllImport("ode", EntryPoint = "dJointSetFeedback"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetFeedback(JointID joint, ref JointFeedback feedback);

		[DllImport("ode", EntryPoint = "dJointSetFixed"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetFixed(JointID joint);

		[DllImport("ode", EntryPoint = "dJointSetHingeAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHingeAnchor(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetHingeAnchorDelta"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHingeAnchorDelta(JointID joint, Real x, Real y, Real z, Real ax, Real ay, Real az);

		[DllImport("ode", EntryPoint = "dJointSetHingeAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHingeAxis(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetHingeParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHingeParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetHinge2Anchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHinge2Anchor(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetHinge2Axis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHinge2Axis1(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetHinge2Axis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHinge2Axis2(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetHinge2Param"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetHinge2Param(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetLMotorAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetLMotorAxis(JointID joint, int anum, int rel, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetLMotorNumAxes"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetLMotorNumAxes(JointID joint, int num);

		[DllImport("ode", EntryPoint = "dJointSetLMotorParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetLMotorParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetPlane2DAngleParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPlane2DAngleParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetPlane2DXParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPlane2DXParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetPlane2DYParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPlane2DYParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetPRAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPRAnchor(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetPRAxis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPRAxis1(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetPRAxis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPRAxis2(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetPRParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetPRParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetSliderAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetSliderAxis(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetSliderAxisDelta"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetSliderAxisDelta(JointID joint, Real x, Real y, Real z, Real ax, Real ay, Real az);

		[DllImport("ode", EntryPoint = "dJointSetSliderParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetSliderParam(JointID joint, int parameter, Real value);

		[DllImport("ode", EntryPoint = "dJointSetUniversalAnchor"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetUniversalAnchor(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetUniversalAxis1"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetUniversalAxis1(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetUniversalAxis2"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetUniversalAxis2(JointID joint, Real x, Real y, Real z);

		[DllImport("ode", EntryPoint = "dJointSetUniversalParam"), SuppressUnmanagedCodeSecurity]
		public static extern void JointSetUniversalParam(JointID joint, int parameter, Real value);
		#endregion

		#region mass
		[DllImport("ode", EntryPoint = "dMassAdd"), SuppressUnmanagedCodeSecurity]
		public static extern void MassAdd(ref Mass a, ref Mass b);

		[DllImport("ode", EntryPoint = "dMassAdjust"), SuppressUnmanagedCodeSecurity]
		public static extern void MassAdjust(ref Mass m, Real newmass);

		[DllImport("ode", EntryPoint = "dMassCheck"), SuppressUnmanagedCodeSecurity]
		public static extern bool MassCheck(ref Mass m);

		[DllImport("ode", EntryPoint = "dMassRotate"), SuppressUnmanagedCodeSecurity]
		public static extern void MassRotate(out Mass mass, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dMassRotate"), SuppressUnmanagedCodeSecurity]
		public static extern void MassRotate(out Mass mass, ref Real M00);

		[DllImport("ode", EntryPoint = "dMassSetBox"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetBox(out Mass mass, Real density, Real lx, Real ly, Real lz);

		[DllImport("ode", EntryPoint = "dMassSetBoxTotal"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetBoxTotal(out Mass mass, Real total_mass, Real lx, Real ly, Real lz);

		[DllImport("ode", EntryPoint = "dMassSetCapsule"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetCapsule(out Mass mass, Real density, int direction, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dMassSetCapsuleTotal"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetCapsuleTotal(out Mass mass, Real total_mass, int direction, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dMassSetCylinder"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetCylinder(out Mass mass, Real density, int direction, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dMassSetCylinderTotal"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetCylinderTotal(out Mass mass, Real total_mass, int direction, Real radius, Real length);

		[DllImport("ode", EntryPoint = "dMassSetParameters"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetParameters(out Mass mass, Real themass,
			 Real cgx, Real cgy, Real cgz,
			 Real i11, Real i22, Real i33,
			 Real i12, Real i13, Real i23);

		[DllImport("ode", EntryPoint = "dMassSetSphere"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetSphere(out Mass mass, Real density, Real radius);

		[DllImport("ode", EntryPoint = "dMassSetSphereTotal"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetSphereTotal(out Mass mass, Real total_mass, Real radius);

		[DllImport("ode", EntryPoint = "dMassSetTrimesh"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetTrimesh(out Mass mass, Real density, GeomID g);

		[DllImport("ode", EntryPoint = "dMassSetZero"), SuppressUnmanagedCodeSecurity]
		public static extern void MassSetZero(out Mass mass);

		[DllImport("ode", EntryPoint = "dMassTranslate"), SuppressUnmanagedCodeSecurity]
		public static extern void MassTranslate(out Mass mass, Real x, Real y, Real z);
		#endregion

		#region basic geometry and mathematics
		[DllImport("ode", EntryPoint = "dDot"), SuppressUnmanagedCodeSecurity]
		public static extern Real Dot(ref Real X0, ref Real X1, int n);

		[DllImport("ode", EntryPoint = "dDQfromW"), SuppressUnmanagedCodeSecurity]
		public static extern void DQfromW(Real[] dq, ref Vector3 w, ref Quaternion q);

		[DllImport("ode", EntryPoint = "dFactorCholesky"), SuppressUnmanagedCodeSecurity]
		public static extern int FactorCholesky(ref Real A00, int n);

		[DllImport("ode", EntryPoint = "dFactorLDLT"), SuppressUnmanagedCodeSecurity]
		public static extern void FactorLDLT(ref Real A, out Real d, int n, int nskip);

		[DllImport("ode", EntryPoint = "dMultiply0"), SuppressUnmanagedCodeSecurity]
		public static extern void Multiply0(out Real A00, ref Real B00, ref Real C00, int p, int q, int r);

		[DllImport("ode", EntryPoint = "dMultiply1"), SuppressUnmanagedCodeSecurity]
		public static extern void Multiply1(out Real A00, ref Real B00, ref Real C00, int p, int q, int r);

		[DllImport("ode", EntryPoint = "dMultiply2"), SuppressUnmanagedCodeSecurity]
		public static extern void Multiply2(out Real A00, ref Real B00, ref Real C00, int p, int q, int r);

		[DllImport("ode", EntryPoint = "dQFromAxisAndAngle"), SuppressUnmanagedCodeSecurity]
		public static extern void QFromAxisAndAngle(out Quaternion q, Real ax, Real ay, Real az, Real angle);

		[DllImport("ode", EntryPoint = "dQfromR"), SuppressUnmanagedCodeSecurity]
		public static extern void QfromR(out Quaternion q, ref Matrix3 R);

		[DllImport("ode", EntryPoint = "dQMultiply0"), SuppressUnmanagedCodeSecurity]
		public static extern void QMultiply0(out Quaternion qa, ref Quaternion qb, ref Quaternion qc);

		[DllImport("ode", EntryPoint = "dQMultiply1"), SuppressUnmanagedCodeSecurity]
		public static extern void QMultiply1(out Quaternion qa, ref Quaternion qb, ref Quaternion qc);

		[DllImport("ode", EntryPoint = "dQMultiply2"), SuppressUnmanagedCodeSecurity]
		public static extern void QMultiply2(out Quaternion qa, ref Quaternion qb, ref Quaternion qc);

		[DllImport("ode", EntryPoint = "dQMultiply3"), SuppressUnmanagedCodeSecurity]
		public static extern void QMultiply3(out Quaternion qa, ref Quaternion qb, ref Quaternion qc);

		[DllImport("ode", EntryPoint = "dQSetIdentity"), SuppressUnmanagedCodeSecurity]
		public static extern void QSetIdentity(out Quaternion q);

		[DllImport("ode", EntryPoint = "dRanReal"), SuppressUnmanagedCodeSecurity]
		public static extern Real RanReal();

		[DllImport("ode", EntryPoint = "dRFrom2Axes"), SuppressUnmanagedCodeSecurity]
		public static extern void RFrom2Axes(out Matrix3 R, Real ax, Real ay, Real az, Real bx, Real by, Real bz);

		[DllImport("ode", EntryPoint = "dRFromAxisAndAngle"), SuppressUnmanagedCodeSecurity]
		public static extern void RFromAxisAndAngle(out Matrix3 R, Real x, Real y, Real z, Real angle);

		[DllImport("ode", EntryPoint = "dRFromEulerAngles"), SuppressUnmanagedCodeSecurity]
		public static extern void RFromEulerAngles(out Matrix3 R, Real phi, Real theta, Real psi);

		[DllImport("ode", EntryPoint = "dRfromQ"), SuppressUnmanagedCodeSecurity]
		public static extern void RfromQ(out Matrix3 R, ref Quaternion q);

		[DllImport("ode", EntryPoint = "dRFromZAxis"), SuppressUnmanagedCodeSecurity]
		public static extern void RFromZAxis(out Matrix3 R, Real ax, Real ay, Real az);

		[DllImport("ode", EntryPoint = "dRSetIdentity"), SuppressUnmanagedCodeSecurity]
		public static extern void RSetIdentity(out Matrix3 R);

		[DllImport("ode", EntryPoint = "dSetValue"), SuppressUnmanagedCodeSecurity]
		public static extern void SetValue(out Real a, int n);

		[DllImport("ode", EntryPoint = "dSetZero"), SuppressUnmanagedCodeSecurity]
		public static extern void SetZero(out Real a, int n);

		[DllImport("ode", EntryPoint = "dSolveCholesky"), SuppressUnmanagedCodeSecurity]
		public static extern void SolveCholesky(ref Real L, out Real b, int n);

		[DllImport("ode", EntryPoint = "dSolveL1"), SuppressUnmanagedCodeSecurity]
		public static extern void SolveL1(ref Real L, out Real b, int n, int nskip);

		[DllImport("ode", EntryPoint = "dSolveL1T"), SuppressUnmanagedCodeSecurity]
		public static extern void SolveL1T(ref Real L, out Real b, int n, int nskip);

		[DllImport("ode", EntryPoint = "dSolveLDLT"), SuppressUnmanagedCodeSecurity]
		public static extern void SolveLDLT(ref Real L, ref Real d, out Real b, int n, int nskip);

		[DllImport("ode", EntryPoint = "dVectorScale"), SuppressUnmanagedCodeSecurity]
		public static extern void VectorScale(out Real a, ref Real d, int n);

		[DllImport("ode", EntryPoint = "dIsPositiveDefinite"), SuppressUnmanagedCodeSecurity]
		public static extern int IsPositiveDefinite(ref Real A, int n);

		[DllImport("ode", EntryPoint = "dInvertPDMatrix"), SuppressUnmanagedCodeSecurity]
		public static extern int InvertPDMatrix(ref Real A, out Real Ainv, int n);

		[DllImport("ode", EntryPoint = "dInfiniteAABB"), SuppressUnmanagedCodeSecurity]
		public static extern void InfiniteAABB(GeomID geom, out AABB aabb);

		[DllImport("ode", EntryPoint = "dLDLTAddTL"), SuppressUnmanagedCodeSecurity]
		public static extern void LDLTAddTL(ref Real L, ref Real d, ref Real a, int n, int nskip);
		#endregion
	}
}
