using System;
using System.Collections.Generic;
using ObjectDefinition;

namespace MonoPhysicsEngine
{
	public class SpatialPartition
	{
		#region Public Fields

		public List<ContactIndex> ObjectList;

		#endregion

		#region Constructor

		public SpatialPartition (
			List<ContactIndex> objectList)
		{
			this.ObjectList = objectList;
		}

		#endregion
	}
}

