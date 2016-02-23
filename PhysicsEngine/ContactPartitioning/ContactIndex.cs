﻿using System;
using PhysicsEngineMathUtility;

namespace MonoPhysicsEngine
{
	public struct ContactIndex
	{
		#region Public Properties

		public readonly int IndexA;
		public readonly int IndexB;
		public readonly ContactType Type;

		#endregion

		#region Constructor

		public ContactIndex(
			int indexA,
			int indexB,
			ContactType type)
		{
			this.IndexA = indexA;
			this.IndexB = indexB;
			this.Type = type;
		}

		#endregion
	}
}

