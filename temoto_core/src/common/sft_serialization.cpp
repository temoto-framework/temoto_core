// This is broken as hell

#include "temoto_core/common/sft_serialization.h"

namespace temoto_core
{

// TODO: create function that does JSON file creation
    
// TODO: move SemanticFrameTree definitions into temoto_core namespace

// TODO: ask @rvalner if these serialization functions should belong to the SFT
// class (which holds the root of the sft)

// TODO: return type on SFTToJson should be directory string of JSON file?
void SFTToJson(TaskTree& sft)
{
    // BFS traversal to copy tree nodes
    
    std::queue<TaskTreeNode> sft_traversal_queue;
    
    sft_traversal_queue.push(sft.getRootNode());

    while(!sft_traversal_queue.empty())
    {
        TaskTreeNode current_node = sft_traversal_queue.front();
        
        // TODO: return type on SFTNodeToJson should write to JSON file 
        SFTNodeToJson(current_node);

        std::vector<TaskTreeNode> current_node_children = current_node.getChildren();
        // TODO verify behavior when a childless node has "getChildren" called
        for( int i=0; i<current_node_children.size(); i++){
            // push all kiddos onto the queue
            sft_traversal_queue.push(current_node_children[i]);
        }
        sft_traversal_queue.pop();
    }
}

void SFTNodeToJson(TaskTreeNode& node){
    // converts a node to a string for JSON text
    // open-source-parsers.github.io/jsoncpp-docs/doxygen/index.html
    //
    // then calls function that does search for parent node to inject the 
    // SFTNode string into the JSON file with appropriate organization
}

// TODO: create function that does node placement in the JSON file

void JsonToSFT(float reliability)
{
    // TODO
}

} // temoto_core namespace
