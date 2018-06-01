using System.Collections.Generic;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class BruteForceBroadPhase : IBroadPhase
    {
        #region Fields

        CollisionEngineParameters collisionEngineParameters;

        #endregion

        #region Contructor

        public BruteForceBroadPhase(CollisionEngineParameters collisionEngineParameters)
        {
            this.collisionEngineParameters = collisionEngineParameters;
        }

        #endregion

        #region Public Methods

        public List<CollisionPair> Execute(
            AABB[] boxs, 
            double distanceTolerance)
        {
            var result = new List<CollisionPair>();

            for (int i = 0; i < boxs.Length; i++)
            {
                for (int j = i; j < boxs.Length; j++)
                    result.Add(new CollisionPair(i, j));
            }

            return result;
        }

        #endregion
    }
}
