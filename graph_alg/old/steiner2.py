# This is a generalized implementation of the Kou algorithm for creating Steiner Trees.  It is not
# tied to GOGrapher and can be used with any networkx wieghted graph.
 
from heapq import *
from networkx import *
 
## Extract a Steiner tree from a weighted graph, given a list of vertices of interest
# @param G  An XGraph
# @param voi  A list of vertices of interest
# @param generator A method to make a new XGraph instance (in the case that you've extended XGraph)
# \returns a new graph if no errors, None otherwise
def make_steiner_tree(G, voi, generator=None):
        mst = Graph()
        for v in voi:
                if not v in G:
                        raise ValueError, "make_steiner_tree(): Some vertice not in original graph"
        if len(voi) == 0:
                return mst
        if len(voi) == 1:
                mst.add_node(voi[0])
                return mst

        # Initially, use (a version of) Kruskal's algorithm to extract a minimal spanning tree
        # from a weighted graph.  This algorithm differs in that only a subset of vertices are
        # going to be present in the final subgraph (which is not truely a MST - must use Prim's
        # algorithm later.

        # extract all shortest paths among the voi
        heapq = []
        paths = {}

        # load all the paths bwteen the Steiner vertices. Store them in a heap queue
        # and reconstruct the MST of the complete graph using Kruskal's algorithm
        for i in range(len(voi) - 1):
                v1 = voi[i]
                for v2  in voi[i+1:]:
                        result = bidirectional_dijkstra(G, v1, v2)
                        if result == False:
                                raise RuntimeError, "The two vertices given (%s, %s) don't exist on the same connected graph" % (v1, v2)
                        distance, vertList = result
                        keys = [v1, v2]
                        keys.sort()
                        key = "%s:%s" % tuple(keys)
                        paths[key] = (vertList)
                        heappush(heapq, (distance, v1, v2))

                                
        # construct the minimum spanning tree of the complete graph
        while heapq:
                w, v1, v2 = heappop(heapq)
                # if no path exists yet between v1 and v2, add this one
                if v1 not in mst or v2 not in mst or not shortest_path(mst, v1, v2):
                        print v1,v2,w		
                        mst.add_edge(v1, v2, weight=w)


        # check if the graph is tree and correct
        sTree = set(mst.nodes())
        sSteiner = set(voi)
        if sTree ^ sSteiner:
                raise RuntimeError, 'Failed to construct MST spanning tree'

        
        # reconstruct subgraph of origGraph using the paths
        if generator is None:
                subgraph  = Graph()
        else:
                subgraph = generator()
        for edge in mst.edges_iter():
                keys = [edge[0],edge[1]]
                keys.sort()
                key = "%s:%s" % tuple(keys)
                vList = paths[key]
                for i in range(len(vList) - 1):
                        v1 = vList[i]
                        v2 = vList[i+1]
                        w = G.get_edge_data(v1, v2)
                        print w
                        subgraph.add_edge(v1, v2, w)

        # get rid of possible loops - result will be a true MST
        subgraph = make_prim_mst(subgraph, generator)
        # remove intermediate nodes in paths that are not in list of voi
        return _trimTree(subgraph, voi)


# remove intermediate nodes in paths that are not in list of voi in given graph
def _trimTree(graph, voi):
        trimKeepTrack = []
        firstNode = voi[0]
        if len(graph.neighbors(firstNode)) < 2:
                trimKeepTrack.append(firstNode)
                firstNeighbor = graph.neighbors(firstNode)[0]
                trimKeepTrack.append(firstNeighbor)
                graph = _trim(firstNeighbor, graph, trimKeepTrack, voi)
        else:
                trimKeepTrack.append(firstNode)
                graph = _trim(firstNode, graph, trimKeepTrack, voi)
        return graph


def _trim(node, graph, trimKeepTrack, voi):
         if len(graph.adj[node].keys()) > 1:
                 for nodeNeighbor in graph.adj[node].keys():
                         if nodeNeighbor not in trimKeepTrack:
                                 trimKeepTrack.append(nodeNeighbor)
                                 graph = _trim(nodeNeighbor, graph, trimKeepTrack, voi)
         if len(graph.adj[node].keys()) < 2:
                 if node not in voi:
                         graph.delete_node(node)
         return graph
 
 
"""
Prim's algorithm: constructs the minimum spanning tree (MST) from an instance of XGraph
@param G An XGraph()
@param generator A method to make a new XGraph instance (in the case that you've extended XGraph)
\returns A MST verstion of G
"""
def make_prim_mst(G, generator=None):
         if generator is None:
                 mst = Graph()
         else:
                 mst = generator()       
         #priorityQ is a list of list (the reverse of the edge tuple with the weight in the front)
         priorityQ = []
         firstNode = G.nodes()[0]
         mst.add_node(firstNode)
         for edge in G.edges_iter(firstNode, data=True):
                 print edge
                 if len(edge) != 3 or edge[2] is None:
                         raise ValueError, "make_prim_mst accepts a weighted graph only (with numerical weights)"
                 heappush(priorityQ, (edge[2], edge))
                 
         while len(mst.edges()) < (G.order()-1):
                 w, minEdge = heappop(priorityQ)
                 if len(minEdge) != 3 or minEdge[2] is None:
                         raise ValueError, "make_prim_mst accepts a weighted graph only (with numerical weights)"             
                 v1, v2, w = minEdge
                 if v1 not in mst:
                         for edge in G.edges_iter(v1, data=True):
                                 if edge == minEdge:
                                         continue
                                 heappush(priorityQ, (edge[2], edge))
                 elif v2 not in mst:
                         for edge in G.edges_iter(v2,data=True):
                                 if edge == minEdge:
                                         continue
                                 heappush(priorityQ, (edge[2], edge))
                 else:
                         # non-crossing edge 
                         continue
                 print minEdge
                 mst.add_edge(*tuple(minEdge[0:2]), weight=minEdge[2])
         return mst
 
 
 
 
if __name__ == "__main__":
         def stree(edges, voi):
                 g = Graph()
                 for edge in edges:
					 g.add_edge(edge[0], edge[1], weight=edge[2])
                 return make_steiner_tree(g, voi)
 
         edges = [("a", "b", 1), ("a", "c", 5), ("a", "e", 2), ("a", "d", 2), ("b", "c", 2), ("c", "d", 3), ("e", "d", 6)]
         st = stree(edges, ['c', 'e', 'a'])
         from draw import *
         print st.nodes()
         draw_custom(st)
         assert(st.edges() == [('a', 'b', 1), ('a', 'e', 2), ('c', 'b', 2)])
         
         edges = [('a', 'b', 3), ('b', 'c', 4), ('c', 'd', 5), ('a', 'e', 1), ('e', 'd', 1)]
         st = stree(edges, ['b', 'd'])
         assert(st.edges() == [('a', 'b', 3), ('a', 'e', 1), ('e', 'd', 1)])
 
         edges = [('a', 'b', 4), ('a', 'c', 4), ('b', 'c', 4)]
         st = stree(edges, ['a', 'b', 'c'])
         assert(st.edges() == [('a', 'c', 4), ('a', 'b', 4)])
 
         # from the markowsky paper
         edges = [('v1', 'v9', 1), ('v1', 'v2', 10), ('v8', 'v9', .5), ('v9', 'v5', 1), ('v8', 'v7', .5), ('v7', 'v6', 1), ('v6', 'v5', 1), ('v2', 'v6', 1),
                  ('v2', 'v3', 8), ('v3', 'v5', 2), ('v5', 'v4', 2), ('v3', 'v4', 9)]
         st = stree(edges, ['v1', 'v2', 'v3', 'v4'])
         assert(st.edges() == [('v1', 'v9', 1), ('v2', 'v6', 1), ('v3', 'v5', 2), ('v4', 'v5', 2), ('v5', 'v9', 1), ('v5', 'v6', 1)])
 
         edges = [('a', 'b', 0), ('b', 'c', 0), ('a', 'd', 3), ('b', 'd', 3), ('c', 'd', 3)]
         st = stree(edges, ['a', 'b', 'c', 'd'])
         assert(st.edges() == [('a', 'b', 0), ('a', 'd', 3), ('c', 'b', 0)])
                    
         edges = [('a', 'b', 0), ('b', 'c', 0), ('a', 'd', 3), ('b', 'd', 3), ('c', 'd', 3), ('d', 'e', 1)]
         st = stree(edges, ['a', 'b', 'c', 'e'])
         assert(st.edges() == [('a', 'b', 0), ('a', 'd', 3), ('c', 'b', 0), ('e', 'd', 1)])
