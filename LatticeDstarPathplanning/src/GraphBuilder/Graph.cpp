#include <C:/Users/conne/Desktop/HSR_2023-24/LatticeDstarPathplanning/src/headers/GraphUtils/GraphFuncs.h>

#include <vector>
using namespace std;

class Graph
{
public:

    // there exists a one dimensional list of nodes which corresponds to the adjacency matrix
    // I :heart: adj. matrices
    
    int num_nodes; // nodes+1 is the rows and cols, where adj_mat sub nodes is the node weight of node
    float **ground_adj_mat; // pointer to 2d arr, unalterable ground truth, any non zero entry should be treated as adjacent, each entry is the weight of the connection
    float **dynamic_adj_mat; // all the node dropping should be done here
    node *nodes; // one d arr of nodes

    Graph(node *nodes, int numNodes, float **mat, float**matCopy)
    {
        this->nodes = nodes;
        this->num_nodes = numNodes;
        this->ground_adj_mat = mat;
        this->dynamic_adj_mat = matCopy;
    }

    void remove_node_adj_matrix(int index)
    {
        for (int j = 0; j < this->num_nodes; j++)
        {
            this->dynamic_adj_mat[index][j] = -1.0f;
        }
    }
    
    void add_node_adj_matrix(int index)
    {
        for (int j = 0; j < this->num_nodes; j++)
        {
            this->dynamic_adj_mat[index][j] = this->ground_adj_mat[index][j];
        }
    }

    int loc_to_index(pair<float,float> a)
    {
        for (int i = 0; i < this->num_nodes; i++)
        {
            if (this->nodes[i].get_loc() == a)
            {
                return i;
            }
        }

        return -1;
        
    }

    void set_node_value(float val, int index)
    {
        if (val < 0)
        {
            remove_node_adj_matrix(index); // maybe just 0 out all in adjacency mat
        }

        else
        {
            if (this->nodes[index].get_cost() < 0)
            {
                add_node_adj_matrix(index);
            }
        }

        this->nodes[index].set_cost(val);
        
    }
    
    void set_node_value(float val, pair<float,float> a)
    {
        this->set_node_value(val, this->loc_to_index(a));
    }

    // insert and node deletions methods are not required because of the static nature of the graph

    float get_node_value(int index) // starts at 0
    {
        
        if (index > this->num_nodes-1) {return numeric_limits<float>::quiet_NaN();}
        return nodes[index].get_cost();
        
    }
    
    float get_node_value(pair<float,float> a)
    {
        return this->get_node_value(this->loc_to_index(a));
    }
    
};