using System.Collections;
using System.Collections.Generic;
using UnityEngine;


[System.Serializable]
public class NavStep
{
	public bool pathExists;
	public int node;
}


[System.Serializable]
public class NavNode
{
	[SerializeField]
	public int id;
	[SerializeField]
	public Vector3 location;
	[SerializeField]
	public List<int> neighbours;

	/// <summary>
	/// Holds the next node to travel to when attempting to reach a specific target node
	/// </summary>
	[SerializeField]
	public NavStep[] paths;


	public float totalCost { get { return heuristicCost + travelCost; } }

	[System.NonSerialized]
	public float heuristicCost; // h cost
	[System.NonSerialized]
	public float travelCost; // g cost


	public NavNode(int id, Vector3 location)
	{
		this.id = id;
		this.location = location;
		neighbours = new List<int>();
    }


	public float Distance(NavNode target)
	{
		return Vector3.Distance(location, target.location);
	}
}