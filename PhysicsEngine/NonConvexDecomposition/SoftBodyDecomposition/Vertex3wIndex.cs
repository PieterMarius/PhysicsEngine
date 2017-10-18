﻿using SharpEngineMathUtility;

namespace SharpPhysicsEngine.NonConvexDecomposition.SoftBodyDecomposition
{
    public class Vertex3Index
    {
        #region Fields

        public Vector3 Vector3 { get; private set; }
        public int[] Indexes { get; private set; }
        public int ID { get; private set; }

        #endregion

        #region Constructor

        public Vertex3Index(
            Vector3 v, 
            int[] index,
            int id)
        {
            Vector3 = v;
            Indexes = index;
            ID = id;
        }

        #endregion

    }
}