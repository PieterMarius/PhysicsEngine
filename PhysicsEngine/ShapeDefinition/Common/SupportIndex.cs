using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SharpPhysicsEngine.ShapeDefinition
{
    internal sealed class SupportIndex
    {
        #region Fields

        public int ID { get; set; }
        
        private HashSet<int> GlobalAdjacencyHash;
        private HashSet<int> LocalAdjacencyHash;

        #endregion

        #region Constructor

        public SupportIndex(int id)
        {
            ID = id;

            if (GlobalAdjacencyHash == null)
            {
                GlobalAdjacencyHash = new HashSet<int>();
                LocalAdjacencyHash = new HashSet<int>();
            }
        }

        #endregion

        #region Public Methods

        public void AddVertexToGlobalAdjList(int vertex)
        {
            GlobalAdjacencyHash.Add(vertex);
        }

        public void AddVertexToLocalAdjList(int vertex)
        {
            LocalAdjacencyHash.Add(vertex);
        }

        public HashSet<int> GetGlobalAdjacencyList()
        {
            return GlobalAdjacencyHash;
        }

        public HashSet<int> GetLocalAdjacencyList()
        {
            return LocalAdjacencyHash;
        }

        #endregion

        #region Private Methods

        #endregion
    }
}
