using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using SharpPhysicsEngine.ShapeDefinition;

namespace SharpPhysicsEngine.CollisionEngine
{
    internal sealed class SweepAndPrune : IBroadPhase
    {
        #region Fields

        private CollisionEngineParameters collisionEngineParameters;
        
         
            
        #endregion

        #region Contructor

        public SweepAndPrune(CollisionEngineParameters collisionEngineParameters)
        {
            this.collisionEngineParameters = collisionEngineParameters;
        }

        #endregion

        #region Public Methods

        public List<CollisionPair> Execute(AABB[] boxs, double distanceTolerance)
        {
            throw new NotImplementedException();
        }

        #endregion
    }
}
