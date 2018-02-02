using System.Collections;
using System.Collections.Generic;
using UnityEngine;



[ExecuteInEditMode]
public class NavVolume : MonoBehaviour
{
	[SerializeField]
	private Bounds localBounds;
	public Bounds globalBounds { get { return new Bounds(localBounds.center + transform.position, localBounds.size); } }


	[SerializeField]
	private float nodeSpacing = 1.0f;
	[SerializeField]
	private Vector3 agentSize = new Vector3(1, 3, 1);

	[SerializeField]
	private bool useDijkstra = false;
	[SerializeField]
	private float agentJumpHeight = 1.0f;

	[SerializeField, HideInInspector]
	private List<NavNode> nodes = new List<NavNode>();



	// TODO - REMOVE
	public bool rebuild;
	private List<NavNode> testPath;
	public Vector3 testStart;
	public Vector3 testEnd;


	void Update()
	{
		if (rebuild)
		{
			PlaceNodes();
			Debug.Log (nodes.Count + " nodes.");
			BakePaths();
			Debug.Log ("Paths baked.");

			// Test Path
			CalculatePath(GetClosestNode(testStart), GetClosestNode(testEnd), out testPath);
			Debug.Log("Test path len: " + (testPath == null ? -1 : testPath.Count));
			rebuild = false;
        }
	}

	void Awake()
	{
		PlaceNodes();
	}

	void PlaceNodes()
	{
		Bounds bounds = globalBounds;

		// Temporarily store nodes in grid, to create neighbours
		int gridSizeX = (int)(bounds.size.x / nodeSpacing);
		int gridSizeY = (int)(bounds.size.y / nodeSpacing);
		int gridSizeZ = (int)(bounds.size.z / nodeSpacing);
		NavNode[,,] grid = new NavNode[gridSizeX, gridSizeY, gridSizeZ];
		

		// Create nodes
		nodes.Clear();

		for (int x = 0; x < gridSizeX; ++x)
			for (int y = 0; y < gridSizeY; ++y)
				for (int z = 0; z < gridSizeZ; ++z)
				{
					Vector3 loc = bounds.min + new Vector3(x, y, z) * nodeSpacing;
					RaycastHit hit;

					// Only store if a valid floor
					if (Physics.Raycast(new Ray(loc, Vector3.down), out hit, nodeSpacing))
					{
						loc = hit.point;

						// Check agent can pass through here
						if (!Physics.CheckBox(loc + new Vector3(0, agentSize.y * 0.51f, 0), agentSize * 0.5f))
						{
							NavNode node = new NavNode(nodes.Count, loc);

							nodes.Add(node);
							grid[x, y, z] = node;
						}
					}
				}


		// Create links between neighbours
		for (int x = 0; x < gridSizeX; ++x)
			for (int y = 0; y < gridSizeY; ++y)
				for (int z = 0; z < gridSizeZ; ++z)
				{
					if (grid[x, y, z] == null)
						continue;

					List<int> neighbours = grid[x, y, z].neighbours;

					for (int dx = -1; dx <= 1; ++dx)
						for (int dy = -1; dy <= 2; ++dy) // Check extra for y (Might be able to jump)
							for (int dz = -1; dz <= 1; ++dz)
							{
								// Ignore self
								if (dx == 0 && dy == 0 && dz == 0)
									continue;

								int cx = x + dx;
								int cy = y + dy;
								int cz = z + dz;

								// Invalid check
								if (cx < 0 || cx >= gridSizeX || cy < 0 || cy >= gridSizeY || cz < 0 || cz >= gridSizeZ)
									continue;

								// Cell is valid
								if (grid[cx, cy, cz] != null)
								{
									// Check cell is in range
									Vector3 diff = grid[x, y, z].location - grid[cx, cy, cz].location;
									diff.x = Mathf.Abs(diff.x);
									diff.y = Mathf.Abs(diff.y);
									diff.z = Mathf.Abs(diff.z);

									if(diff.y < agentJumpHeight)
										neighbours.Add(grid[cx, cy, cz].id);
								}
                            }
                }

	}

	/// <summary>
	/// For each node will calculate the desired next step in order to reach every other node
	/// </summary>
	void BakePaths()
	{
		// Reset all paths
		foreach(NavNode node in nodes)
			node.paths = new NavStep[nodes.Count];

		// Cache best paths
		foreach (NavNode a in nodes)
			foreach (NavNode b in nodes) 
			{
				// Ignore self
				if (a == b)
				{
					NavStep step = new NavStep ();
					step.pathExists = false;
					a.paths [b.id] = step;
				}


				// Path from a -> b already found in previous check
				if (a.paths[b.id] != null)
					continue;


				List<NavNode> path;
				if (CalculatePath (a, b, out path)) 
				{
					// Store best steps throughout path
					for (int i = 0; i < path.Count - 1; ++i) 
					{
						NavStep step = new NavStep();
						step.node = path[i + 1].id;
						step.pathExists = true;
						path[i].paths [path [i + 1].id] = step;
					}
				} 

				// No path found
				else 
				{
					NavStep step = new NavStep ();
					step.pathExists = false;
					a.paths [b.id] = step;
				}
			}
	}

	bool CalculatePath(NavNode source, NavNode target, out List<NavNode> outPath)
	{
		List<NavNode> openNodes = new List<NavNode>();
		HashSet<NavNode> closedNodes = new HashSet<NavNode>();
		NavNode[] cameFromTable = new NavNode[nodes.Count];


		// Add the source node
		source.heuristicCost = useDijkstra ? 0 : target.Distance(source);
		source.travelCost = 0;
        openNodes.Add(source);


		// Search for all avaliable paths
		while (openNodes.Count != 0)
		{
			// Find lowest cost open node
			NavNode current = openNodes[0];
			for (int i = 1; i < openNodes.Count; ++i)
			{
				float a = openNodes[i].totalCost;
				float b = current.totalCost;

				if (a < b || (a == b && openNodes[i].travelCost < current.travelCost))
					current = openNodes[i];
			}

			// Lock current node's states
			openNodes.Remove(current);
			closedNodes.Add(current);


			// Reached target
			if (current == target)
			{
				// Reconstruct path in reverse then flip
				outPath = new List<NavNode>();
				
				while (current != null)
				{
					outPath.Add(current);
					current = cameFromTable[current.id];
				}

				outPath.Reverse();
                return true;
			}


			// Add neighbours
			foreach (int neighbourId in current.neighbours)
			{
				NavNode neighbour = nodes[neighbourId];

				// Ignore locked nodes
				if (closedNodes.Contains(neighbour))
					continue;

				// Check to see if this path is faster for open nodes
				if (openNodes.Contains(neighbour))
				{
					float h = useDijkstra ? 0 : target.Distance(neighbour);
					float g = current.Distance(neighbour) + current.totalCost;

					// New route is better
					if (neighbour.totalCost > h + g)
					{
						neighbour.heuristicCost = h;
						neighbour.travelCost = g;

						openNodes.Add(neighbour);
						cameFromTable[neighbour.id] = current;
					}

				}
				// Not considered yet
				else
				{
					// Workout it's values
					neighbour.heuristicCost = useDijkstra ? 0 : target.Distance(neighbour);
					neighbour.travelCost = current.Distance(neighbour) + current.totalCost;

					openNodes.Add(neighbour);
					cameFromTable[neighbour.id] = current;
				}
			}
		}

		// Has failed to reach target if reached here
		outPath = null;
		return false;
	}

	NavNode GetClosestNode(Vector3 a)
	{
		NavNode node = nodes[0];
		float min = Vector3.SqrMagnitude(node.location - a);

		for (int i = 1; i < nodes.Count; ++i)
		{
			float d = Vector3.SqrMagnitude(nodes[i].location - a);
			if (d < min)
			{
				min = d;
				node = nodes[i];
			}
		}
		
		return node;
	}


#if UNITY_EDITOR
	void OnDrawGizmos()
	{
		// Draw bounds
		Bounds bounds = globalBounds;
		Gizmos.color = Color.green;
		Gizmos.DrawWireCube(bounds.center, bounds.size);
	}

	void OnDrawGizmosSelected()
	{ 
		// Draw bounds
		Bounds bounds = globalBounds;
        Gizmos.color = Color.yellow;
		Gizmos.DrawWireCube(bounds.center, bounds.size);

		// Draw nodes
		if (nodes != null)
		{
			Gizmos.color = Color.green;
			foreach (NavNode c in nodes)
			{
				Gizmos.DrawSphere(c.location, 0.1f);

				// Draw connections
				foreach (int n in c.neighbours)
				{
					if (nodes[n].id < c.id)
						Gizmos.DrawLine(nodes[n].location, c.location);
				}
            }
		}


		// DRAW TEST
		Gizmos.color = Color.green;
		Gizmos.DrawCube(testStart, Vector3.one * 0.5f);
		NavNode a = GetClosestNode(testStart);
		Gizmos.DrawSphere(a.location, 0.5f);


		Gizmos.color = Color.red;
		Gizmos.DrawCube(testEnd, Vector3.one * 0.5f);
		NavNode b = GetClosestNode(testEnd);
		Gizmos.DrawSphere(b.location, 0.5f);

		Gizmos.color = Color.black;
		if (testPath != null)
		{
			foreach(NavNode node in testPath)
				Gizmos.DrawSphere(node.location, 0.25f);


		}
	}
#endif


}