using UnityEngine;
using System.Collections;

[System.Serializable]
public class PSBParticle
{
	#region Variables
	
	public int prevInternalSpring = 0;
	public int nextInternalSpring = 0;
	public Vector2 position = Vector2.zero;
	public Vector2 velocity = Vector2.zero;
	public Vector2 normal = Vector2.zero;
	public Vector2 force = Vector2.zero;
	public GameObject gameObj = null;
	public Rigidbody2D rBody = null;
	public CircleCollider2D circleCol = null;
	
	#endregion
}
