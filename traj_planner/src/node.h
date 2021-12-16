#include <Eigen/Eigen>
#include <vector>  
#include <memory>



struct Node;

typedef Node* NodePtr;
struct Node
{     
      Eigen::Vector3d pos;
      float radius; // radius of this node

      bool valid;
      bool best;
      bool change;
      bool visit;
     
      NodePtr parent_node;
      std::vector<NodePtr> child_nodes;
      
      int rel_id;
      int rel_dis;
      float g; // total cost of the shortest path from this node to the root
      float f; // heuristic value of the node to the target point
      
      Node( Eigen::Vector3d pos_, float radius_, float g_, float f_)
      {		
		pos  = pos_;
		radius = radius_;
		g      = g_;  
		f      = f_;		
		valid  = true;
		best   = false; 
            change = false;
            visit = false;
		parent_node = NULL;
      	child_nodes.clear();
      }
};


