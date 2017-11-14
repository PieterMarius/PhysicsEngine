using SharpPhysicsEngine.CollisionEngine;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.ContactPartitioning
{
    internal sealed class Partition
    {
        public List<CollisionPointStructure> PartitionedCollisionPoints { get; private set; }
        public List<IConstraint> PartitionedJoints { get; private set; }

        public Partition()
        {
            PartitionedCollisionPoints = new List<CollisionPointStructure>();
            PartitionedJoints = new List<IConstraint>();
        }

        #region Public Methods

        //public void AddCollisionPoint(CollisionPointStructure cps)
        //{
        //    CollisionPoint.Add(cps);
        //}

        //public void Addjoint(IConstraint constraint)
        //{
        //    partitionedJoint.Add(constraint);
        //}

        //public List<CollisionPointStructure> GetCollisionPoints()
        //{
        //    return CollisionPoint;
        //}

        #endregion
    }
}
