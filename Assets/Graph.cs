using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class Graph : MonoBehaviour {
	
	GraphMap graph;
	public int maxNodes = 11;
	Node target, start;
	void Start () {
		graph = new GraphMap (maxNodes);
		for (int i = 0; i <= maxNodes; i++) {
			graph.AddNode (i.ToString());
		}

		graph.AddUndirectedEdge(11, 6, 1);
		graph.AddUndirectedEdge(0, 9, 1);
		graph.AddUndirectedEdge(1, 2, 1);
		graph.AddUndirectedEdge(0, 1, 1);
		graph.AddUndirectedEdge(10, 1, 1);
		graph.AddUndirectedEdge(11, 5, 1);
		graph.AddUndirectedEdge(2, 3, 1);
		graph.AddUndirectedEdge(4, 5, 1);
		graph.AddUndirectedEdge(8, 9, 1);
		graph.AddUndirectedEdge(6, 7, 1);
		graph.AddUndirectedEdge(7, 8, 1);
		graph.AddUndirectedEdge(0, 6, 1);
		graph.AddUndirectedEdge(3, 4, 1);
		graph.AddUndirectedEdge(0, 2, 1);
		graph.AddUndirectedEdge(11, 7, 1);
		graph.AddUndirectedEdge(0, 8, 1);
		graph.AddUndirectedEdge(0, 4, 1);
		graph.AddUndirectedEdge(9, 10, 1);
		graph.AddUndirectedEdge(0, 5, 1);
		graph.AddUndirectedEdge(0, 7, 1);
		graph.AddUndirectedEdge(0, 3, 1);
		graph.AddUndirectedEdge(0, 10, 1);
		graph.AddUndirectedEdge(5, 6, 1);

		target = graph.SearchForNode (10.ToString());
		start = graph.SearchForNode (11.ToString());

//		int bfs = BreadthFirstSearch(start, target);
//		Debug.Log ("bfs: " + bfs);
		int dks = Dijkstra (start, target);
		Debug.Log ("dikjstra: " + dks);
		Debug.Log(graph.PrintPathRecursive(target, " "));
	}

	static string CalcCut(Node start, Node target)
	{
		Node currNode = target;
		Node prevNode = null;
		while (currNode != null && currNode != start) {
			prevNode = currNode;
			currNode = currNode.parentNode;
		}

		return currNode.label + " " + prevNode.label;
	}



	int BreadthFirstSearch(Node start, Node target)
	{  

		if (start == null || target == null) {
			return 0;
		}
		if (start.connectedEdges.Count == 0) {
			return 0;
		}

		Queue<Node> myQueue = new Queue<Node> ();
		myQueue.Enqueue (start);
		start.hasBeenVisited = true;
		start.parentNode = null;

		while (myQueue.Count != 0) {
			
			Node top = myQueue.Peek ();
			myQueue.Dequeue ();
			
			if (top.label == target.label) {
				Debug.Log ("found Target" + top.label);
				return graph.CalcPathLength(target, 0);
			}
				
			foreach (var node in top.GetConnectedNodes()) {

				if (node.hasBeenVisited == false) {
					node.parentNode = top;
					myQueue.Enqueue (node);
					node.hasBeenVisited = true;
					// Early exit if direct child is target
					if (node.label == target.label) {
						return graph.CalcPathLength(target, 0);
					}
				}
			}
		}
		return 0;
	}
	int Dijkstra(Node source, Node target)
	{
		
		if (start == null || target == null) {
			return 0;
		}
		if (start.connectedEdges.Count == 0) {
			return 0;
		}
			


		foreach (var node in graph.GraphNodes) {
			node.pathLength = int.MaxValue;
			node.parentNode = null;
			node.hasBeenVisited = false;
		}
		start.pathLength = 0;


		while (graph.GetUnvisitedNodes().Count > 0) {
			
			Node top = graph.GetUnvisitedNodes().OrderBy (y => y.pathLength).First ();
			if (top.label == target.label) {
				Debug.Log ("found Target" + top.pathLength);
				return graph.CalcPathLength(target, 0);
			}
			top.hasBeenVisited = true;

			foreach (var edge in top.connectedEdges) {
				Node neighbour = edge.GetFarNode ();
				if(!neighbour.hasBeenVisited)
				{
					int distance = top.pathLength + edge.GetCost (); // cost
					if (distance < neighbour.pathLength) {
						Debug.Log (top.pathLength);
						neighbour.pathLength = distance;
						neighbour.parentNode = top;
					}
				}
			}


		}
		return 0;
	}

}


[System.Serializable]
public class GraphMap
{
	public List<Node> GraphNodes;
	public Dictionary<string, Node> labelMap;
	public GraphMap(int numNodes)
	{
		GraphNodes = new List<Node> ();
		labelMap = new Dictionary<string, Node> ();
	}
	public void AddNode(object label)
	{
		string nodeLabel = label.ToString ();
		Node n = new Node (nodeLabel);
		GraphNodes.Add (n);
		labelMap.Add (nodeLabel, n);
	}
	public Node SearchForNode(object label)
	{
		Node returnNode;
		labelMap.TryGetValue (label.ToString(), out returnNode);
		return returnNode;
	}
	public List<Node> GetUnvisitedNodes()
	{
		return GraphNodes.Where (x => (x.hasBeenVisited == false)).ToList ();
	}
	public void SetAllNodesUnvisited()
	{
		foreach (var node in GraphNodes) {
			node.hasBeenVisited = false;	
		}
	}
	public bool AddDirectedEdge(object u_label, object v_label, int cost)
	{
		Node  u = SearchForNode(u_label.ToString());
		Node  v = SearchForNode(v_label.ToString());
		if (u != null && v != null) {
			Edge uv = new Edge (u, v, cost);
			u.AddEdge (uv);
			return true;
		} else
			return false;	
	}

	public bool AddUndirectedEdge(object u_label, object v_label, int cost)
	{
		Node  u = SearchForNode(u_label.ToString());
		Node  v = SearchForNode(v_label.ToString());
		if (u != null && v != null) {
			Edge uv = new Edge (u, v, cost);
			Edge vu = new Edge (v, u, cost);
			u.AddEdge (uv);
			v.AddEdge (vu);
			return true;
		} else
			return false;

	}

	public int CalcPathLength(Node target, int pathLength)
	{

		if (target == null) {
			return pathLength - 1;
		}
		return CalcPathLength (target.parentNode, pathLength + 1) ;
	}

	public string PrintPath(Node target)
	{
		string currPath = target.label;
		Node currNode = target.parentNode;
		while (currNode != null) {
			currPath = currPath + " -- " + currNode.label;
			currNode = currNode.parentNode;
		}
		return currPath;

	}
	public string PrintPathRecursive(Node target, string pathString)
	{
		if (target == null) {
			return pathString;
		}
		return PrintPathRecursive (target.parentNode, pathString + " -- " + target.label) ;
	}

}
[System.Serializable]
public class Edge
{
	Node start;
	Node end;
	int cost;

	public Edge(Node u, Node v)
	{
		this.start = u;
		this.end = v;
		this.cost = 0;
	}
	public Edge(Node u, Node v, int cost)
	{
		this.start = u;
		this.end = v;
		this.cost = cost;
	}
	public int GetCost()
	{
		return this.cost;
	}
	public Node GetFarNode()
	{
		return this.end;
	}
}
public class Node 
{
	public string label;
	public List<Edge> connectedEdges;
	public bool hasBeenVisited;
	public bool isGateWay;
	public Node parentNode;
	public int pathLength{ get; set;}
	public Node(object label)
	{
		this.label = label.ToString();
		connectedEdges = new List<Edge>();
		hasBeenVisited = false;
		parentNode = null;
		pathLength = -1;

	}
	public void AddEdge(Edge e)
	{
		connectedEdges.Add (e);
	}

	public List<Node> GetConnectedNodes()
	{
		List<Node> adjacentNodes = new List<Node> ();
		foreach (var edge in connectedEdges) {
			adjacentNodes.Add (edge.GetFarNode ());
		}
		return adjacentNodes;
	}

	public int CompareTo(Node other)
	{   
		if (pathLength > other.pathLength) {
			return 1;
		} else if (pathLength < other.pathLength) {
			return -1;
		} else {
			return 0;
		}
	}


}

