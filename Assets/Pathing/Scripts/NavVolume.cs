using System.Collections;
using System.Collections.Generic;
using UnityEngine;


class NavNode
{
	public Vector3 location { get; internal set; }
}


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


	private List<NavNode> nodes;


	// TODO - REMOVE
	public bool rebuild;


	void Update()
	{
		if (rebuild)
		{
			PlaceNodes();
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
		nodes = new List<NavNode>();

		Vector3 n;
		for (n.x = bounds.min.x; n.x < bounds.max.x; n.x += nodeSpacing)
			for (n.y = bounds.min.y; n.y < bounds.max.y; n.y += nodeSpacing)
				for (n.z = bounds.min.z; n.z < bounds.max.z; n.z += nodeSpacing)
				{
					RaycastHit hit;

					// Check for floor first
					if (Physics.Raycast(new Ray(n, Vector3.down), out hit, nodeSpacing))
					{
						Vector3 p = hit.point;

						// Check agent can pass through here
						if (!Physics.CheckBox(p + new Vector3(0, agentSize.y * 0.51f, 0), agentSize * 0.5f))
						{
							NavNode node = new NavNode();
							node.location = p;
							nodes.Add(node);
						}
					}


				}
	}

	void BakePaths()
	{

	}

	NavNode GetClosestNode(Vector3 a)
	{
		float min = 10000000.0f;
		NavNode node = null;

		foreach (NavNode n in nodes)
		{
			float d = Vector2.SqrMagnitude(n.location - a);
			if (d < min)
			{
				min = d;
				node = n;
			}
		}

		return node;
	}


#if UNITY_EDITOR
	void OnDrawGizmos()
	{
		DrawBounds(globalBounds, Color.green);
	}

	void OnDrawGizmosSelected()
	{
		DrawBounds(globalBounds, Color.yellow);

		Gizmos.color = Color.green;
		foreach (NavNode n in nodes)
			Gizmos.DrawSphere(n.location, 0.1f);
	}

	static void DrawBounds(Bounds b, Color colour)
	{
		Gizmos.color = colour;
		Gizmos.DrawWireCube(b.center, b.size);
	}
#endif


}