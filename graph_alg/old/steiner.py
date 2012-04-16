#!~linus/Downloads/sage-4.7.1/sage -python

import networkx as nx
import sys
sys.path.append('/home/linus/Downloads/sage-4.7.1')
print sys.path
from sage.all import *
from sage.numerical.mip import MixedIntegerLinearProgram

def steiner_tree(self,G, weighted = False):
	"""
	Returns a tree of minimum weight connecting the given
	set of vertices.
	+
	+        Definition :
	+
	+        Computing a minimum spanning tree in a graph can be done in `n
	+        \log(n)` time (and in linear time if all weights are
	+        equal). On the other hand, if one is given a large (possibly
	+        weighted) graph and a subset of its vertices, it is NP-Hard to
	+        find a tree of minimum weight connecting the given set of
	+        vertices, which is then called a Steiner Tree.
	+        
	+        `Wikipedia article on Steiner Trees
	+        <http://en.wikipedia.org/wiki/Steiner_tree_problem>`_.
	+
	+        INPUT:
	+
	+        - ``vertices`` -- the vertices to be connected by the Steiner
	+          Tree.
	+          
	+        - ``weighted`` (boolean) -- Whether to consider the graph as 
	+          weighted, and use each edge's label as a weight, considering
	+          ``None`` as a weight of `1`. If ``weighted=False`` (default)
	+          all edges are considered to have a weight of `1`.
	+
	+        .. NOTE::
	+
	+            * This problem being defined on undirected graphs, the
	+              orientation is not considered if the current graph is
	+              actually a digraph.
	+
	+            * The graph is assumed not to have multiple edges.
	+
	+        ALGORITHM:
	+
	+        Solved through Linear Programming.
	+
	+        COMPLEXITY:
	+
	+        NP-Hard. 
	+
	+        Note that this algorithm first checks whether the given 
	+        set of vertices induces a connected graph, returning one of its
	+        spanning trees if ``weighted`` is set to ``False``, and thus
	+        answering very quickly in some cases
	+        
	+        EXAMPLES:
	+
	+        The Steiner Tree of the first 5 vertices in a random graph is,
	+        of course, always a tree ::
	+
	+            sage: g = graphs.RandomGNP(30,.5)
	+            sage: st = g.steiner_tree(g.vertices()[:5])              # optional - requires GLPK, CBC or CPLEX
	+            sage: st.is_tree()                                       # optional - requires GLPK, CBC or CPLEX
	+            True
	+
	+        And all the 5 vertices are contained in this tree ::
	+

	+            sage: all([v in st for v in g.vertices()[:5] ])          # optional - requires GLPK, CBC or CPLEX
	+            True
	+
	+        An exception is raised when the problem is impossible, i.e.
	+        if the given vertices are not all included in the same
	+        connected component ::
	+
	+            sage: g = 2 * graphs.PetersenGraph()
	+            sage: st = g.steiner_tree([5,15])
	+            Traceback (most recent call last):
	+            ...
	+            ValueError: The given vertices do not all belong to the same connected component. This problem has no solution !
	+
	"""
	if G.is_directed():
		g = nx.Graph(G)
	else:
		g = G
	vertices=G.nodes()
	if g.has_multiple_edges():
		raise ValueError("The graph is expected not to have multiple edges.")
	# Can the problem be solved ? Are all the vertices in the same
	# connected component ?
	cc = g.connected_component_containing_vertex(vertices[0])
	if not all([v in cc for v in vertices]):
		raise ValueError("The given vertices do not all belong to the same connected component. This problem has no solution !")
	# Can it be solved using the min spanning tree algorithm ?
	if not weighted:
		gg = g.subgraph(vertices)
	if gg.is_connected():
		st = g.subgraph(edges = gg.min_spanning_tree())
		st.delete_vertices([v for v in g if st.degree(v) == 0])
		return st
	# Then, LP formulation
	p = MixedIntegerLinearProgram(maximization = False)
	
	# Reorder an edge
	R = lambda (x,y) : (x,y) if x<y else (y,x)
	
	
	# edges used in the Steiner Tree
	edges = p.new_variable()
	
	# relaxed edges to test for acyclicity
	r_edges = p.new_variable()

	# Whether a vertex is in the Steiner Tree
	vertex = p.new_variable()
	for v in g:
		for e in g.edges_incident(v, labels=False):
			p.add_constraint(vertex[v] - edges[R(e)], min = 0)
	
	# We must have the given vertices in our tree
	for v in vertices:
		p.add_constraint(sum([edges[R(e)] for e in g.edges_incident(v,labels=False)]), min=1)

	# The number of edges is equal to the number of vertices in our tree minus 1
	p.add_constraint(sum([vertex[v] for v in g]) - sum([edges[R(e)] for e in g.edges(labels=None)]), max = 1, min = 1)
	
	# There are no cycles in our graph
	
	for u,v in g.edges(labels = False):
		p.add_constraint( r_edges[(u,v)]+ r_edges[(v,u)] - edges[R((u,v))] , min = 0 )
		
	eps = 1/(5*Integer(g.order()))
	for v in g:
		p.add_constraint(sum([r_edges[(u,v)] for u in g.neighbors(v)]), max = 1-eps)
		
		
	# Objective
	if weighted:
		w = lambda (x,y) : g.edge_label(x,y) if g.edge_label(x,y) is not None else 1
	else:
		w = lambda (x,y) : 1
		
	p.set_objective(sum([w(e)*edges[R(e)] for e in g.edges(labels = False)]))
	
	p.set_binary(edges)     
	p.solve()
	
	edges = p.get_values(edges)
	
	st =  g.subgraph(edges=[e for e in g.edges(labels = False) if edges[R(e)] == 1])
	st.delete_vertices([v for v in g if st.degree(v) == 0])
	return st

def edge_disjoint_spanning_trees(self,k, root=None, **kwds):
	"""
	Returns the desired number of edge-disjoint spanning 
	"""

if __name__ == '__main__':
	G=nx.Graph()
	G.add_nodes_from([(0,0), (1,1), (-1,1)])
	G.add_edges_from([((0,0), (1,1)), ((1,1), (1,-1)), ((-1,1), (0,0))])
	s=steiner_tree(G, weighted=False)
	import plot
	plot_custom(s)
	
