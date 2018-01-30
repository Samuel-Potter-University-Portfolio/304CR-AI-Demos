using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class NavNode
{
	public readonly int id;
	public readonly Vector3 location;
	public readonly List<NavNode> neighbours;

	/// <summary>
	/// Holds the next node to travel to when attempting to reach a specific target node
	/// </summary>
	public readonly NavNode[] paths;


	public float totalCost { get { return heuristicCost + travelCost; } }
	public float heuristicCost; // h cost
	public float travelCost; // g cost


	public NavNode(int id, Vector3 location)
	{
		this.id = id;
		this.location = location;
		neighbours = new List<NavNode>();
    }


	public float Distance(NavNode target)
	{
		return Vector3.Distance(location, target.location);
	}
}